"""
UGV node: drive a TurtleBot 4 through its team's tours, carrying its UAV.

For every tour the UGV
  1. drives to the release point and turns to the takeoff heading,
  2. asks its UAV to take off and waits until the UAV is airborne,
  3. drives to the collect point and turns to the landing heading,
  4. asks its UAV to land and waits until it is back on the deck,
and after the last tour it parks at the team's end point. The UGV leads the
team: the UAV only takes off and lands when its UGV asks.

Run one instance per TurtleBot 4 in the robot's namespace (e.g. /ugv1) so that
cmd_vel and scan resolve to the robot's own topics. With standalone:=true the
UGV drives its tours without a UAV (bring-up test).

Driving is a go-to-goal P controller on the MoCap pose, with the APF
avoidance layer and the emergency stop from apf.py on top.
"""

import enum
import math
import signal
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from uav_ugv_nav.mission import distance, load_mission, MissionError
from uav_ugv_nav.ugv.apf import (
    apf_velocity, ApfParams, emergency_stop, scan_returns, world_to_body, wrap_to_pi)

OTHER_ROBOT_TIMEOUT = 1.0  # ignore other UGVs whose MoCap pose is older than this [s]

# name: default. Every parameter is documented in config/params.yaml.
PARAMETERS = {
    'mission_file': '',
    'team': '',
    'mocap_topic': '',
    'uav_name': '',
    'standalone': False,
    'standalone_pause': 2.0,
    'max_linear_speed': 0.2,
    'max_angular_speed': 1.0,
    'k_linear': 0.7,
    'k_angular': 1.8,
    'goal_tolerance': 0.08,
    'mocap_timeout': 0.25,
    'start_tolerance': 0.3,
    'align_yaw': True,
    'align_yaw_deg': 0.0,
    'align_tolerance_deg': 7.0,
    'align_gain': 1.8,
    'align_max_speed': 0.8,
    'align_timeout': 4.0,
    'apf_enabled': True,
    'require_scan': True,
    'scan_timeout': 1.0,
    'lidar_yaw_offset_deg': 90.0,
    'lidar_min_range': 0.15,
    'apf_influence': 0.8,
    'apf_gain': 0.05,
    'apf_vortex': 0.2,
    'apf_sectors': 72,
    'robot_influence': 0.8,
    'robot_gain': 0.1,
    'robot_vortex': 0.2,
    'robot_mask_arc_deg': 15.0,
    'robot_mask_tolerance': 0.4,
    'stop_distance': 0.25,
    'stop_arc_deg': 60.0,
    'robot_stop_distance': 0.35,
    'stuck_speed': 0.02,
    'stuck_time': 2.0,
    'recovery_turn_speed': 0.5,
    'takeoff_timeout': 60.0,
    'land_timeout': 300.0,
}


class Phase(enum.Enum):
    TO_RELEASE = 'driving to the release point'
    TAKEOFF = 'waiting for the UAV to take off'
    TO_COLLECT = 'driving to the collect point'
    LANDING = 'waiting for the UAV to land'
    TO_END = 'driving to the end point'
    DONE = 'mission complete'
    ABORTED = 'mission aborted'


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class UgvNode(Node):

    rate = 30.0  # control loop [Hz]

    def __init__(self):
        super().__init__('ugv_node')
        self.robot = self.get_namespace().strip('/')
        if not self.robot:
            raise RuntimeError(
                'run ugv_node in the TurtleBot namespace, e.g. --ros-args -r __ns:=/ugv1')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('other_mocap_topics', Parameter.Type.STRING_ARRAY)
        get = self._get

        for name in ('mission_file', 'team', 'mocap_topic'):
            if not get(name):
                raise RuntimeError(f'parameter {name} is required')
        mission = load_mission(get('mission_file'))
        self.model = mission.model
        self.team = mission.team(get('team'))
        self.standalone = get('standalone')
        self.uav = get('uav_name')
        if not self.standalone and not self.uav:
            raise RuntimeError('parameter uav_name is required unless standalone:=true')

        self.mocap_topic = get('mocap_topic')

        # Robot state.
        self._pose = None         # (x, y, yaw) from MoCap
        self._pose_time = -math.inf
        self._scan = None
        self._scan_time = -math.inf
        self._others = {}         # MoCap topic -> (x, y, receive time)

        # Mission state.
        self.phase = None
        self.tour_index = 0
        self._phase_since = time.monotonic()
        self._arrived = False
        self._align_deadline = None
        self._stuck_for = 0.0
        self._turn_average = 0.0
        self._recovering = False
        self._recovery_dir = 1.0
        self._future = None
        self._call_started = 0.0
        self._mission_started = None
        self._left_release = None
        self._collect_arrival = None
        self._landed_at = -math.inf

        self.create_subscription(
            PoseStamped, self.mocap_topic, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        for topic in self._other_topics():
            self.create_subscription(
                PoseStamped, topic, lambda msg, t=topic: self._on_other(t, msg),
                qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        if not self.standalone:
            self.takeoff_client = self.create_client(Trigger, f'/{self.uav}/drone_takeoff')
            self.land_client = self.create_client(Trigger, f'/{self.uav}/drone_land')
        self.create_timer(1.0 / self.rate, self._step)

        mode = 'standalone, no UAV' if self.standalone else f'carrying {self.uav}'
        self.get_logger().info(
            f'{self.robot} ready ({mode}): team {self.team.name}, {len(self.team.tours)} tours, '
            f'MoCap {self.mocap_topic}, LiDAR yaw offset {get("lidar_yaw_offset_deg"):g} deg')
        for index, tour in enumerate(self.team.tours, start=1):
            self.get_logger().info(
                f'  tour {index}: release {_fmt(tour.release)} -> collect {_fmt(tour.collect)}')
        self.get_logger().info(f'  end point {_fmt(self.team.end)}')

    # -- parameters and callbacks ------------------------------------------------

    def _get(self, name: str):
        return self.get_parameter(name).value

    def _apf_params(self) -> ApfParams:
        # Read on every control step, so `ros2 param set` tunes the avoidance live.
        get = self._get
        return ApfParams(
            lidar_yaw_offset=math.radians(get('lidar_yaw_offset_deg')),
            lidar_min_range=get('lidar_min_range'),
            influence=get('apf_influence'),
            gain=get('apf_gain'),
            vortex=get('apf_vortex'),
            robot_influence=get('robot_influence'),
            robot_gain=get('robot_gain'),
            robot_vortex=get('robot_vortex'),
            mask_arc=math.radians(get('robot_mask_arc_deg')),
            mask_tolerance=get('robot_mask_tolerance'),
            stop_distance=get('stop_distance'),
            stop_arc=math.radians(get('stop_arc_deg')),
            robot_stop_distance=get('robot_stop_distance'))

    def _other_topics(self):
        try:
            topics = self.get_parameter('other_mocap_topics').value or []
        except ParameterUninitializedException:
            topics = []
        return [topic for topic in topics if topic and topic != self.mocap_topic]

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (p.x, p.y, yaw)
        self._pose_time = time.monotonic()

    def _on_other(self, topic: str, msg: PoseStamped) -> None:
        self._others[topic] = (msg.pose.position.x, msg.pose.position.y, time.monotonic())

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_time = time.monotonic()

    # -- mission state machine ---------------------------------------------------

    def _step(self) -> None:
        if self.phase in (Phase.DONE, Phase.ABORTED):
            return
        now = time.monotonic()
        if now - self._pose_time > self._get('mocap_timeout'):
            self._stop()
            self.get_logger().warn(
                f'no fresh MoCap pose on {self.mocap_topic}; holding still',
                throttle_duration_sec=2.0)
            return
        if self.phase is None:
            self._start_mission(now)
            return

        tours = self.team.tours
        tour = tours[self.tour_index] if self.tour_index < len(tours) else None
        if self.phase is Phase.TO_RELEASE and self._reached(tour.release, 'release'):
            self._set_phase(Phase.TAKEOFF)
        elif self.phase is Phase.TAKEOFF:
            self._takeoff(now)
        elif self.phase is Phase.TO_COLLECT and self._reached(tour.collect, 'collect'):
            self._collect_arrival = now
            self._set_phase(Phase.LANDING)
        elif self.phase is Phase.LANDING:
            self._landing(now)
        elif self.phase is Phase.TO_END and self._reached(self.team.end, 'end'):
            self.get_logger().info(
                f'parked at the end point; mission took {now - self._mission_started:.1f} s')
            self._set_phase(Phase.DONE)

    def _start_mission(self, now: float) -> None:
        offset = distance(self._pose[:2], self.team.start)
        if offset > self._get('start_tolerance'):
            self.get_logger().warn(
                f'{self.robot} is {offset:.2f} m from the mission start point '
                f'{_fmt(self.team.start)}; the plan assumed it starts there')
        self._mission_started = now
        self._set_phase(Phase.TO_RELEASE)

    def _set_phase(self, phase: Phase) -> None:
        self.phase = phase
        self._phase_since = time.monotonic()
        self._arrived = False
        self._align_deadline = None
        label = self._tour_label() if phase not in (Phase.TO_END, Phase.DONE) else ''
        self.get_logger().info(f'{label}{phase.value}')

    def _tour_label(self) -> str:
        return f'tour {self.tour_index + 1}/{len(self.team.tours)}: '

    def _takeoff(self, now: float) -> None:
        if self.standalone:
            if now - self._phase_since >= self._get('standalone_pause'):
                self._left_release = now
                self._set_phase(Phase.TO_COLLECT)
            return
        recharge_left = self.model.recharge_time - (now - self._landed_at)
        if recharge_left > 0.0:
            self.get_logger().info(
                f'recharging {self.uav}: {recharge_left:.0f} s left', throttle_duration_sec=5.0)
            return
        response = self._call(self.takeoff_client, self._get('takeoff_timeout'))
        if response is None:
            return
        if not response.success:
            self._abort(f'{self.uav} did not take off: {response.message}')
            return
        self.get_logger().info(f'{self.uav} is airborne ({response.message})')
        self._left_release = now
        self._set_phase(Phase.TO_COLLECT)

    def _landing(self, now: float) -> None:
        if self.standalone:
            if now - self._phase_since >= self._get('standalone_pause'):
                self._finish_tour(now)
            return
        response = self._call(self.land_client, self._get('land_timeout'))
        if response is None:
            return
        if not response.success:
            self._abort(f'{self.uav} did not land on {self.robot}: {response.message}')
            return
        self._landed_at = now
        self._finish_tour(now)

    def _finish_tour(self, now: float) -> None:
        drive = self._collect_arrival - self._left_release
        slack = self.model.max_flight_time - drive
        waited = now - self._collect_arrival
        self.get_logger().info(
            f'{self._tour_label()}done. Release->collect drive {drive:.1f} s, ground slack '
            f'{slack:.1f} s (min {self.model.min_ground_slack:g} s); waited {waited:.1f} s at '
            'collect')
        self.tour_index += 1
        more = self.tour_index < len(self.team.tours)
        self._set_phase(Phase.TO_RELEASE if more else Phase.TO_END)

    def _abort(self, reason: str) -> None:
        self._stop()
        self.get_logger().error(
            f'{self._tour_label()}{reason}. Stopping {self.robot}; restart the mission once '
            'the problem is fixed.')
        self._set_phase(Phase.ABORTED)

    def _call(self, client, timeout: float):
        """Call a Trigger service without blocking: None while waiting, then the response."""
        now = time.monotonic()
        if self._future is None:
            if not client.service_is_ready():
                self.get_logger().info(
                    f'waiting for {client.srv_name} (is the uav_node running?)',
                    throttle_duration_sec=5.0)
                return None
            self._future = client.call_async(Trigger.Request())
            self._call_started = now
            return None
        if not self._future.done():
            if now - self._call_started > timeout:
                self._future.cancel()
                self._future = None
                return Trigger.Response(
                    success=False, message=f'no answer from {client.srv_name} in {timeout:g} s')
            return None
        future, self._future = self._future, None
        try:
            response = future.result()
        except Exception as error:  # noqa: B902 - report any failure of the call
            return Trigger.Response(success=False, message=f'service call failed: {error}')
        if response is None:
            return Trigger.Response(success=False, message='service call failed')
        return response

    # -- driving -------------------------------------------------------------------

    def _reached(self, goal, label: str) -> bool:
        """Drive to goal, then turn to the takeoff/landing heading; True when both are done."""
        if not self._arrived:
            self._arrived = self._drive_to(goal, label)
            if not self._arrived:
                return False
        return self._align()

    def _drive_to(self, goal, label: str) -> bool:
        x, y, yaw = self._pose
        dx, dy = goal[0] - x, goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < self._get('goal_tolerance'):
            self._stop()
            self._stuck_for, self._recovering = 0.0, False
            return True

        apf = self._apf_params()
        robots = self._other_robots(x, y, yaw)
        scan = self._scan_body(apf)
        if scan is None:
            self._stop()
            self.get_logger().warn(
                f'no LiDAR scan on {self.resolve_topic_name("scan")} for '
                f'{self._get("scan_timeout"):g} s; holding still', throttle_duration_sec=2.0)
            return False
        returns, sparse_returns = scan
        reason = emergency_stop(returns, robots, apf)
        if reason:
            self._stop()
            self._stuck_for = 0.0
            self.get_logger().warn(f'emergency stop: {reason}', throttle_duration_sec=1.0)
            return False

        max_v, max_w = self._get('max_linear_speed'), self._get('max_angular_speed')
        yaw_error = wrap_to_pi(math.atan2(dy, dx) - yaw)
        speed = min(self._get('k_linear') * dist, max_v)
        goal_v = (speed * math.cos(yaw_error), speed * math.sin(yaw_error))
        vx, vy = goal_v
        if self._get('apf_enabled'):
            vx, vy = apf_velocity(goal_v, sparse_returns, robots, apf)
        heading = math.atan2(vy, vx)
        v = min(math.hypot(vx, vy), max_v) * max(0.0, math.cos(heading))
        w = _clamp(self._get('k_angular') * heading, max_w)
        if abs(heading) > math.radians(45.0):
            v = min(v, 0.07)  # turn mostly in place when facing away
        v, w = self._unstick(v, w, yaw_error)
        self._publish(v, w)
        self.get_logger().info(
            f'-> {label} {_fmt(goal)}: {dist:.2f} m to go | v={v:.2f} w={w:.2f} | '
            f'APF push ({vx - goal_v[0]:+.2f}, {vy - goal_v[1]:+.2f})',
            throttle_duration_sec=2.0)
        return False

    def _unstick(self, v: float, w: float, yaw_error: float):
        """Turn in place for a while if the APF has trapped the robot in a local minimum."""
        # Averaging the turn rate also catches a robot that dithers left and right in place.
        self._turn_average += (w - self._turn_average) * min(1.0, 2.0 / self.rate)
        if v < self._get('stuck_speed') and abs(self._turn_average) < 0.1:
            self._stuck_for += 1.0 / self.rate
        else:
            self._stuck_for, self._recovering = 0.0, False
        if self._stuck_for <= self._get('stuck_time'):
            return v, w
        if not self._recovering:
            self._recovering = True
            self._recovery_dir = 1.0 if yaw_error > 0.0 else -1.0
            self.get_logger().warn('stuck: turning in place to escape')
        return 0.0, self._get('recovery_turn_speed') * self._recovery_dir

    def _align(self) -> bool:
        """
        Turn in place to the takeoff/landing heading.

        With position-only MoCap the Crazyflie's Kalman filter assumes it starts
        facing world +x, so the UGV parks at a fixed heading (align_yaw_deg).
        """
        if not self._get('align_yaw'):
            return True
        now = time.monotonic()
        if self._align_deadline is None:
            self._align_deadline = now + self._get('align_timeout')
        error = wrap_to_pi(math.radians(self._get('align_yaw_deg')) - self._pose[2])
        tolerance = math.radians(self._get('align_tolerance_deg'))
        if abs(error) <= tolerance or now > self._align_deadline:
            if abs(error) > tolerance:
                self.get_logger().warn(
                    f'heading still {math.degrees(error):+.0f} deg off after '
                    f'{self._get("align_timeout"):g} s; continuing')
            self._stop()
            self._align_deadline = None
            return True
        self._publish(0.0, _clamp(self._get('align_gain') * error, self._get('align_max_speed')))
        return False

    def _scan_body(self, apf: ApfParams):
        """
        Return (all, downsampled) LiDAR returns in the body frame; None if the scan is stale.

        The emergency stop checks every ray. The APF uses about apf_sectors rays
        per revolution whatever the LiDAR resolution, which keeps its gains valid.
        """
        if time.monotonic() - self._scan_time > self._get('scan_timeout'):
            return None if self._get('require_scan') else ([], [])
        scan = self._scan
        args = (scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max, apf)
        step = max(1, len(scan.ranges) // max(1, self._get('apf_sectors')))
        return scan_returns(*args), scan_returns(*args, step=step)

    def _other_robots(self, x: float, y: float, yaw: float):
        now = time.monotonic()
        return [world_to_body(ox - x, oy - y, yaw)
                for ox, oy, stamp in self._others.values() if now - stamp < OTHER_ROBOT_TIMEOUT]

    def _publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish(0.0, 0.0)

    def stop_robot(self) -> None:
        """Send a zero velocity command (used on shutdown)."""
        try:
            self._stop()
        except Exception:  # noqa: B902 - best effort while shutting down
            pass


def _fmt(point) -> str:
    return '(' + ', '.join(f'{v:.2f}' for v in point) + ')'


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None) -> int:
    # Handle Ctrl+C ourselves so the context is still alive to send a final stop command.
    # Installed explicitly: a process started in the background may inherit SIGINT as ignored.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = None
    try:
        node = UgvNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (MissionError, RuntimeError) as error:
        rclpy.logging.get_logger('ugv_node').fatal(str(error))
        return 1
    finally:
        if node is not None:
            node.stop_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
