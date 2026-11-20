"""
UAV node: fly a Crazyflie through its team's tours, from and back to its UGV.

The node streams the Crazyflie's MoCap position to its Kalman filter and
offers two services to its UGV. Each answers only once its action is done:

  drone_takeoff  run the pre-flight checks and take off; answer once airborne.
                 The UAV then flies the tour's waypoints on its own and hovers
                 above the tour's collect point.
  drone_land     wait until the waypoints are flown, move above the UGV's live
                 MoCap position and land on its deck; answer once landed.

Whoever reaches the collect point first waits for the other: the UGV waits
for the landing, and the UAV hovers until the UGV asks it to land.

Failsafes: the UAV lands where it is if anything fails in flight, if its MoCap
pose stops arriving, if it has been airborne for max_airborne_time, or when
the node is stopped (Ctrl+C). After such an emergency landing it refuses to
take off again until restarted.

With standalone:=true the UAV flies all its tours without a UGV, landing back
at its takeoff spot after each one (bring-up test).

Run one instance per Crazyflie, in a namespace named after it (e.g. /cf1).
"""

import math
import signal
import threading
import time
import traceback

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from uav_ugv_nav.mission import distance, leg_time, load_mission, MissionError, uav_tour_time
from uav_ugv_nav.uav.crazyflie import check_estimate, Crazyflie

# name: default. Every parameter is documented in config/params.yaml.
PARAMETERS = {
    'mission_file': '',
    'team': '',
    'uri': '',
    'mocap_topic': '',
    'ugv_mocap_topic': '',
    'standalone': False,
    'standalone_pause': 5.0,
    'deck_height': 0.39,
    'min_takeoff_voltage': 3.8,
    'max_airborne_time': 240.0,
    'mocap_timeout': 1.0,
    'emergency_land_time': 3.0,
    'estimator_settle_time': 5.0,
    'estimator_check_time': 3.0,
    'estimator_max_error': 0.05,
    'mocap_send_rate': 100.0,
    'cf_cache_dir': '~/.cache/uav_ugv_nav',
}


class FlightAborted(Exception):
    """Raised in the flight thread to give up the tour and land in place."""


class MocapFeed(Node):
    """
    Stream the UAV's MoCap position to its Crazyflie and keep the latest poses.

    A separate node, spun by its own single-threaded executor, so that the
    high-rate MoCap callbacks never queue behind the blocking services of
    uav_node.
    """

    def __init__(self, namespace: str, cf: Crazyflie, mocap_topic: str, ugv_mocap_topic: str,
                 send_rate: float):
        # use_global_arguments=False: the command line's node-name remap targets uav_node only.
        super().__init__('mocap_feed', namespace=namespace, use_global_arguments=False)
        self.cf = cf
        self.send_period = 1.0 / send_rate
        self.pose, self.pose_time = None, -math.inf  # this UAV
        self.ugv, self.ugv_time = None, -math.inf    # its UGV
        self._last_sent = -math.inf
        self.create_subscription(PoseStamped, mocap_topic, self._on_pose, qos_profile_sensor_data)
        if ugv_mocap_topic:
            self.create_subscription(
                PoseStamped, ugv_mocap_topic, self._on_ugv_pose, qos_profile_sensor_data)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        now = time.monotonic()
        self.pose, self.pose_time = (p.x, p.y, p.z), now
        if now - self._last_sent >= self.send_period:
            self._last_sent = now
            try:
                self.cf.send_position(p.x, p.y, p.z)
            except Exception as error:  # noqa: B902 - keep streaming whatever went wrong
                self.get_logger().error(
                    f'could not send the MoCap position: {error}', throttle_duration_sec=1.0)

    def _on_ugv_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.ugv, self.ugv_time = (p.x, p.y, p.z), time.monotonic()


class UavNode(Node):

    def __init__(self):
        super().__init__('uav_node')
        self.uav = self.get_namespace().strip('/')
        if not self.uav:
            raise RuntimeError(
                'run uav_node in a namespace named after the UAV, e.g. --ros-args -r __ns:=/cf1')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        get = self._get

        self.standalone = get('standalone')
        required = ['mission_file', 'team', 'uri', 'mocap_topic']
        if not self.standalone:
            required.append('ugv_mocap_topic')
        for name in required:
            if not get(name):
                raise RuntimeError(f'parameter {name} is required')
        mission = load_mission(get('mission_file'))
        self.model = mission.model
        self.tours = mission.team(get('team')).tours
        self.mocap_topic = get('mocap_topic')
        self.ugv_mocap_topic = get('ugv_mocap_topic')
        self.deck_height = get('deck_height')
        self.min_takeoff_voltage = get('min_takeoff_voltage')
        self.max_airborne_time = get('max_airborne_time')
        self.mocap_timeout = get('mocap_timeout')
        self.emergency_land_time = get('emergency_land_time')

        self._lock = threading.Lock()
        self._tour_index = 0
        self._flight = None          # thread flying the current tour
        self._airborne = threading.Event()
        self._finished = threading.Event()
        self._land_requested = threading.Event()
        self._result = (False, 'no flight yet')
        self._abort_reason = None    # set by other threads to make the flight land
        self._grounded = None        # reason of the last emergency landing
        self._in_air_since = None    # monotonic time of takeoff, None on the ground
        self._estimator_ready = False

        self.cf = Crazyflie(get('uri'), get('cf_cache_dir'), log=self.get_logger().info)
        self.cf.connect()
        self.feed = MocapFeed(self.get_namespace(), self.cf, self.mocap_topic,
                              '' if self.standalone else self.ugv_mocap_topic,
                              get('mocap_send_rate'))

        if self.standalone:
            self._flight = threading.Thread(target=self._run_standalone, daemon=True)
            self._flight.start()
        else:
            commands = ReentrantCallbackGroup()
            self.create_service(Trigger, 'drone_takeoff', self._on_takeoff,
                                callback_group=commands)
            self.create_service(Trigger, 'drone_land', self._on_land, callback_group=commands)

        mode = 'standalone, no UGV' if self.standalone else 'waiting for its UGV'
        self.get_logger().info(
            f'{self.uav} ready ({mode}): team {get("team")}, {len(self.tours)} tours, '
            f'MoCap {self.mocap_topic}')
        for index, tour in enumerate(self.tours, start=1):
            self.get_logger().info(
                f'  tour {index}: {len(tour.waypoints)} waypoints, model flight time '
                f'{uav_tour_time(tour, self.model):.1f} s')

    def _get(self, name: str):
        return self.get_parameter(name).value

    def _pose_fresh(self) -> bool:
        return time.monotonic() - self.feed.pose_time < self.mocap_timeout

    # -- services ------------------------------------------------------------------

    def _on_takeoff(self, request, response):
        with self._lock:
            refusal = self._takeoff_refusal()
            if refusal:
                self.get_logger().warn(f'takeoff refused: {refusal}')
                response.success, response.message = False, refusal
                return response
            index = self._tour_index
            self._airborne.clear()
            self._finished.clear()
            self._land_requested.clear()
            self._flight = threading.Thread(
                target=self._fly_tour, args=(index, False), daemon=True)
            self._flight.start()
        # Answer once airborne: the UGV must not drive off with the UAV still on its deck.
        while not (self._airborne.is_set() or self._finished.is_set()):
            time.sleep(0.05)
        if self._airborne.is_set():
            response.success = True
            response.message = f'flying tour {index + 1}/{len(self.tours)}'
        else:
            response.success, response.message = self._result
        return response

    def _takeoff_refusal(self):
        if self._grounded:
            return f'grounded after an emergency landing ({self._grounded}); restart uav_node'
        if self._flight is not None and self._flight.is_alive():
            return 'already flying'
        if self._tour_index >= len(self.tours):
            return 'all tours are done'
        return None

    def _on_land(self, request, response):
        flight = self._flight
        if flight is None or not flight.is_alive():
            failed = self._finished.is_set() and not self._result[0]
            response.success, response.message = self._result if failed else (False, 'not flying')
            return response
        self.get_logger().info('the UGV is at the collect point and asks to land')
        self._land_requested.set()
        self._finished.wait()
        response.success, response.message = self._result
        return response

    # -- flying --------------------------------------------------------------------

    def _fly_tour(self, index: int, standalone: bool) -> None:
        """Fly one tour from takeoff to touchdown. Runs in its own thread."""
        tour = self.tours[index]
        label = f'tour {index + 1}/{len(self.tours)}'
        altitude = self.model.flight_altitude
        hovered = 0.0
        try:
            self._preflight(label, check_deck=not standalone and index == 0)
            home = self.feed.pose
            self.cf.arm(True)
            self._sleep(1.0)
            self._in_air_since = time.monotonic()
            self.get_logger().info(f'{label}: taking off to {altitude:.2f} m')
            self.cf.takeoff(altitude, self.model.takeoff_time)
            self._sleep(self.model.takeoff_time)
            self._airborne.set()

            position = (home[0], home[1], altitude)
            for number, waypoint in enumerate(tour.waypoints, start=1):
                position = self._fly_to(
                    position, waypoint, f'{label}: waypoint {number}/{len(tour.waypoints)}')
                self._sleep(self.model.hover_time)

            if standalone:
                position = self._fly_to(
                    position, (home[0], home[1], altitude), f'{label}: back to the takeoff spot')
                self._sleep(self.model.settle_time)
                land_height = home[2]
            else:
                collect = (tour.collect[0], tour.collect[1], altitude)
                position = self._fly_to(position, collect, f'{label}: collect point')
                started = time.monotonic()
                if not self._land_requested.is_set():
                    self.get_logger().info(f'{label}: hovering until the UGV reaches collect')
                while not self._land_requested.is_set():
                    self._sleep(0.05)
                hovered = time.monotonic() - started
                ugv = self._ugv_position()
                target = (ugv[0], ugv[1], altitude)
                duration = max(self.model.settle_time,
                               distance(position, target) / self.model.uav_speed)
                self.get_logger().info(f'{label}: moving above the UGV at {_fmt(ugv[:2])}')
                self.cf.go_to(*target, duration)
                self._sleep(duration)
                land_height = self.deck_height

            self.get_logger().info(f'{label}: landing at z={land_height:.2f} m')
            self.cf.land(land_height, self.model.land_time)
            self._sleep(self.model.land_time)
            self._motors_off()
            flight_time = time.monotonic() - self._in_air_since
            self._in_air_since = None
            self._tour_index = index + 1
            self._report(label, tour, flight_time, hovered)
            self._result = (True, f'landed after {flight_time:.1f} s of flight')
        except FlightAborted as abort:
            self._result = (False, str(abort))
            self._emergency_land(label, str(abort))
        except Exception as error:  # noqa: B902 - any failure in flight must end in a landing
            self.get_logger().error(f'{label}: unexpected error\n{traceback.format_exc()}')
            self._result = (False, f'unexpected error: {error!r}')
            self._emergency_land(label, repr(error))
        finally:
            self._finished.set()

    def _preflight(self, label: str, check_deck: bool) -> None:
        if not self._pose_fresh():
            raise FlightAborted(f'no MoCap pose on {self.mocap_topic}')
        battery = self.cf.battery_voltage()
        if battery is None:
            raise FlightAborted('no telemetry from the Crazyflie yet')
        if battery < self.min_takeoff_voltage:
            raise FlightAborted(
                f'battery at {battery:.2f} V, below min_takeoff_voltage '
                f'({self.min_takeoff_voltage:g} V)')
        z = self.feed.pose[2]
        if check_deck and abs(z - self.deck_height) > 0.05:
            self.get_logger().warn(
                f'{self.uav} sits at z={z:.2f} m but deck_height is {self.deck_height:.2f} m; '
                'is it on its UGV? Landings aim for deck_height')
        if not self._estimator_ready:
            self.get_logger().info(f'{label}: resetting the Kalman filter')
            self.cf.reset_estimator()
            self._sleep(self._get('estimator_settle_time'))
            self._estimator_ready = True
        passed, detail = check_estimate(
            self.cf.estimate, lambda: self.feed.pose, sleep=self._sleep,
            max_error=self._get('estimator_max_error'),
            duration=self._get('estimator_check_time'))
        if not passed:
            raise FlightAborted(f'pre-flight check failed: {detail}; power-cycle the Crazyflie')
        self.get_logger().info(
            f'{label}: pre-flight check passed, battery {battery:.2f} V, {detail}')

    def _fly_to(self, position, target, label: str):
        duration = leg_time(position, target, self.model)
        self.get_logger().info(f'{label} {_fmt(target)}, {duration:.1f} s')
        self.cf.go_to(*target, duration)
        self._sleep(duration)
        return target

    def _ugv_position(self):
        ugv, stamp = self.feed.ugv, self.feed.ugv_time
        if ugv is None or time.monotonic() - stamp > self.mocap_timeout:
            raise FlightAborted(f'no MoCap pose of the UGV on {self.ugv_mocap_topic}')
        return ugv

    def _sleep(self, seconds: float) -> None:
        """Sleep while watching the failsafes; raise FlightAborted to make the UAV land."""
        deadline = time.monotonic() + seconds
        while True:
            reason = self._abort_reason or self._failsafe()
            if reason:
                raise FlightAborted(reason)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(0.05, remaining))

    def _failsafe(self):
        if self._in_air_since is None:
            return None
        now = time.monotonic()
        if now - self.feed.pose_time > self.mocap_timeout:
            return f'MoCap pose lost for {now - self.feed.pose_time:.1f} s'
        if now - self._in_air_since > self.max_airborne_time:
            return f'airborne for more than max_airborne_time ({self.max_airborne_time:g} s)'
        return None

    def _emergency_land(self, label: str, reason: str) -> None:
        if self._in_air_since is None:
            self.get_logger().error(f'{label}: not taking off: {reason}')
            try:
                self.cf.arm(False)  # in case it was armed before the abort
            except Exception:  # noqa: B902 - best effort
                pass
            return
        self.get_logger().error(f'{label}: ABORT ({reason}); landing where it is')
        self._grounded = reason
        try:
            self.cf.land(0.0, self.emergency_land_time)
            time.sleep(self.emergency_land_time)
            self._motors_off()
        except Exception as error:  # noqa: B902 - nothing more we can do from here
            self.get_logger().error(f'emergency landing failed: {error!r}')
        finally:
            self._in_air_since = None

    def _motors_off(self) -> None:
        self.cf.stop()
        self.cf.arm(False)

    def _report(self, label: str, tour, flight_time: float, hovered: float) -> None:
        """Log the realized flight time and air slack of a tour."""
        if self.standalone:  # includes the trips from and back to the takeoff spot
            self.get_logger().info(f'{label} done: flight {flight_time:.1f} s')
            return
        slack = self.model.max_flight_time - flight_time
        verdict = 'ok' if slack >= self.model.min_air_slack else 'BELOW THE MINIMUM'
        message = (f'{label} done: flight {flight_time:.1f} s (model '
                   f'{uav_tour_time(tour, self.model):.1f} s), air slack {slack:.1f} s '
                   f'({verdict}, min {self.model.min_air_slack:g} s)')
        if hovered > 0.5:
            message += f'; hovered {hovered:.1f} s waiting for the UGV'
        self.get_logger().info(message)

    def _run_standalone(self) -> None:
        while not self._pose_fresh():
            if self._abort_reason:
                return
            self.get_logger().info(
                f'waiting for MoCap on {self.mocap_topic}', throttle_duration_sec=5.0)
            time.sleep(0.1)
        for index in range(len(self.tours)):
            self._fly_tour(index, standalone=True)
            if not self._result[0]:
                return
            if index + 1 < len(self.tours):
                pause = self._get('standalone_pause')
                self.get_logger().info(f'next tour in {pause:g} s')
                deadline = time.monotonic() + pause
                while time.monotonic() < deadline:
                    if self._abort_reason:
                        return
                    time.sleep(0.1)
        self.get_logger().info('all tours flown')

    def shutdown(self) -> None:
        """Land if flying, then close the radio link (the executor must still be spinning)."""
        self._abort_reason = 'uav_node is shutting down'
        flight = self._flight
        if flight is not None and flight.is_alive():
            if self._in_air_since is not None:
                self.get_logger().warn('shutting down: landing first')
            flight.join(timeout=self.emergency_land_time + 5.0)
        self.cf.close()


def _fmt(point) -> str:
    return '(' + ', '.join(f'{v:.2f}' for v in point) + ')'


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None) -> int:
    # Handle Ctrl+C ourselves: the node must keep streaming MoCap while the UAV lands.
    # Installed explicitly: a process started in the background may inherit SIGINT as ignored.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node, executors, status = None, [], 0
    try:
        node = UavNode()
        # The MoCap feed gets its own light executor; the blocking services a multi-threaded one.
        for executor, member in ((SingleThreadedExecutor(), node.feed),
                                 (MultiThreadedExecutor(num_threads=2), node)):
            executor.add_node(member)
            executors.append(executor)
        spinners = [threading.Thread(target=e.spin, daemon=True) for e in executors]
        for spinner in spinners:
            spinner.start()
        while all(spinner.is_alive() for spinner in spinners):
            spinners[0].join(timeout=0.2)
    except KeyboardInterrupt:
        pass
    except (MissionError, RuntimeError) as error:
        rclpy.logging.get_logger('uav_node').fatal(str(error))
        status = 1
    except Exception as error:  # noqa: B902 - e.g. cflib cannot find the Crazyradio
        rclpy.logging.get_logger('uav_node').fatal(f'{error!r}\n{traceback.format_exc()}')
        status = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # let a landing finish undisturbed
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.shutdown()  # lands first; the executors keep the MoCap stream alive meanwhile
        for executor in executors:
            executor.shutdown()
        if node is not None:
            node.feed.destroy_node()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
