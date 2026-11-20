"""
Show where the UGV's LiDAR sees the closest obstacle, in the robot's own frame.

Use it to verify lidar_yaw_offset_deg before driving: put an object about
0.5 m straight in front of the TurtleBot. The bearing should read close to
0 deg (+90 deg is the robot's left). On a TurtleBot 4 the scan frame
rplidar_link is yawed +90 deg from base_link, hence the default.

  ros2 run uav_ugv_nav scan_check --ros-args -r __ns:=/ugv1
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from uav_ugv_nav.ugv.apf import ApfParams, scan_returns, wrap_to_pi


def _side(bearing_deg: float) -> str:
    if abs(bearing_deg) <= 30.0:
        return 'ahead'
    if abs(bearing_deg) >= 150.0:
        return 'behind'
    return 'on the left' if bearing_deg > 0.0 else 'on the right'


class ScanCheck(Node):

    def __init__(self):
        super().__init__('scan_check')
        self.declare_parameter('lidar_yaw_offset_deg', 90.0)
        self.declare_parameter('lidar_min_range', 0.15)
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self._report)
        self.get_logger().info(f'listening on {self.resolve_topic_name("scan")}')

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _report(self) -> None:
        if self.scan is None:
            self.get_logger().warn('no scan received yet')
            return
        offset = math.radians(self.get_parameter('lidar_yaw_offset_deg').value)
        params = ApfParams(lidar_yaw_offset=offset,
                           lidar_min_range=self.get_parameter('lidar_min_range').value)
        scan = self.scan
        returns = scan_returns(scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_max, params)
        if not returns:
            self.get_logger().info('no valid returns in range')
            return
        bearing, rng = min(returns, key=lambda ret: ret[1])
        bearing_deg = math.degrees(bearing)
        raw_deg = math.degrees(wrap_to_pi(bearing - offset))
        self.get_logger().info(
            f'closest return {rng:.2f} m {_side(bearing_deg)}: {bearing_deg:+.0f} deg in '
            f'base_link ({raw_deg:+.0f} deg in {scan.header.frame_id or "the scan frame"})')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
