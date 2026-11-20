"""
Check a Crazyflie on the ground: radio link, Kalman filter and MoCap feed.

Connects, selects the Kalman filter, streams the MoCap position, resets the
filter and checks that its estimate settles on the MoCap position. The motors
stay off. Run it for every Crazyflie before its first flight of the day.

  ros2 run uav_ugv_nav cf_check --ros-args
      -p uri:=radio://0/80/2M/E7E7E7E7E7 -p mocap_topic:=/Robot_3/pose
"""

import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from uav_ugv_nav.uav.crazyflie import check_estimate, Crazyflie


class CfCheck(Node):

    def __init__(self):
        super().__init__('cf_check')
        self.declare_parameter('uri', '')
        self.declare_parameter('mocap_topic', '')
        self.declare_parameter('settle_time', 5.0)
        self.declare_parameter('check_time', 3.0)
        self.declare_parameter('max_error', 0.05)
        self.declare_parameter('cf_cache_dir', '~/.cache/uav_ugv_nav')
        self.uri = self.get_parameter('uri').value
        self.topic = self.get_parameter('mocap_topic').value
        if not self.uri or not self.topic:
            raise RuntimeError('set both uri and mocap_topic, e.g. --ros-args '
                               '-p uri:=radio://0/80/2M/E7E7E7E7E7 -p mocap_topic:=/Robot_3/pose')
        self.pose = None
        self.cf = Crazyflie(self.uri, self.get_parameter('cf_cache_dir').value,
                            log=self.get_logger().info)
        self.cf.connect()
        self.create_subscription(PoseStamped, self.topic, self._on_pose, qos_profile_sensor_data)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.pose = (p.x, p.y, p.z)
            self.cf.send_position(p.x, p.y, p.z)

    def run(self) -> bool:
        log = self.get_logger()
        deadline = time.monotonic() + 5.0
        while self.pose is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.pose is None:
            log.error(f'FAIL: no MoCap pose on {self.topic} (is the MoCap driver running?)')
            return False
        log.info(f'MoCap position: ({self.pose[0]:.3f}, {self.pose[1]:.3f}, {self.pose[2]:.3f})')
        battery = self.cf.battery_voltage()
        if battery is not None:
            log.info(f'battery: {battery:.2f} V')
        settle = self.get_parameter('settle_time').value
        log.info(f'resetting the Kalman filter and waiting {settle:g} s')
        self.cf.reset_estimator()
        time.sleep(settle)
        passed, detail = check_estimate(
            self.cf.estimate, lambda: self.pose,
            max_error=self.get_parameter('max_error').value,
            duration=self.get_parameter('check_time').value)
        if passed:
            log.info(f'PASS: {detail}')
        else:
            log.error(f'FAIL: {detail}')
        return passed


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = CfCheck()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        return 0 if node.run() else 1
    except KeyboardInterrupt:
        return 1
    except Exception as error:  # noqa: B902 - report connection problems plainly
        rclpy.logging.get_logger('cf_check').fatal(f'FAIL: {error!r}')
        return 1
    finally:
        executor.shutdown()
        if node is not None:
            node.cf.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
