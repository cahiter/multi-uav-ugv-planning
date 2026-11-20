import math

import pytest
from uav_ugv_nav.ugv.apf import (
    apf_velocity, ApfParams, emergency_stop, hits_other_robot, scan_returns, world_to_body)

TB4 = ApfParams()  # TurtleBot 4: LiDAR frame yawed +90 deg from base_link


def scan_with_one_hit(scan_angle: float, rng: float, n: int = 720):
    """Return a 360-degree scan starting at -pi with a single return at scan_angle."""
    increment = 2.0 * math.pi / n
    ranges = [math.inf] * n
    ranges[round((scan_angle + math.pi) / increment) % n] = rng
    return ranges, -math.pi, increment


def test_tb4_scan_is_rotated_into_the_body_frame():
    # On a TurtleBot 4, an object straight ahead is seen at -90 deg in rplidar_link.
    ranges, angle_min, increment = scan_with_one_hit(-math.pi / 2.0, 0.5)
    [(bearing, rng)] = scan_returns(ranges, angle_min, increment, 12.0, TB4)
    assert bearing == pytest.approx(0.0, abs=math.radians(1.0))
    assert rng == 0.5


def test_obstacle_ahead_slows_the_robot_and_stops_it_when_close():
    vx, _ = apf_velocity((0.2, 0.0), [(0.0, 0.5)], [], TB4)
    assert vx < 0.2
    assert emergency_stop([(0.0, 0.2)], [], TB4) == 'obstacle 0.20 m ahead'


def test_obstacle_on_the_left_pushes_right_without_stopping():
    _, vy = apf_velocity((0.2, 0.0), [(math.pi / 2.0, 0.5)], [], TB4)
    assert vy < 0.0
    assert emergency_stop([(math.pi / 2.0, 0.2)], [], TB4) is None


def test_vortex_curves_towards_the_side_of_the_goal():
    # Obstacle dead ahead, goal slightly to the left: the vortex pushes further left.
    _, vy = apf_velocity((0.2, 0.05), [(0.0, 0.4)], [], TB4)
    assert vy > 0.05


def test_a_centered_obstacle_still_gets_a_sideways_push():
    # Symmetric returns of an obstacle dead ahead must not cancel each other's vortex terms,
    # otherwise the robot stalls in front of it (APF local minimum).
    returns = [(math.radians(angle), 0.5) for angle in (-10, -5, 0, 5, 10)]
    _, vy = apf_velocity((0.2, 0.0), returns, [], TB4)
    assert vy < -0.01  # veers right


def test_robots_meeting_head_on_both_veer_right():
    _, vy = apf_velocity((0.2, 0.0), [], [(0.5, 0.0)], TB4)
    assert vy < 0.0


def test_far_obstacles_are_ignored():
    assert apf_velocity((0.2, 0.0), [(0.0, 1.5)], [], TB4) == (0.2, 0.0)


def test_returns_from_another_robot_are_masked():
    other = (0.6, 0.0)  # another UGV 0.6 m ahead
    assert hits_other_robot((0.0, 0.45), [other], TB4)
    assert not hits_other_robot((math.pi / 2.0, 0.45), [other], TB4)


def test_another_robot_stops_us_only_when_ahead():
    assert emergency_stop([], [(0.3, 0.0)], TB4) is not None
    assert emergency_stop([], [(-0.3, 0.0)], TB4) is None


def test_world_to_body():
    # Facing +y, a point one meter along world +y is straight ahead.
    bx, by = world_to_body(0.0, 1.0, math.pi / 2.0)
    assert (bx, by) == pytest.approx((1.0, 0.0), abs=1e-9)


def test_invalid_and_self_returns_are_dropped():
    params = ApfParams(lidar_yaw_offset=0.0)
    returns = scan_returns([0.05, math.nan, math.inf, 20.0, 1.0], 0.0, 0.1, 12.0, params)
    assert len(returns) == 1
    assert returns[0] == pytest.approx((0.4, 1.0))
