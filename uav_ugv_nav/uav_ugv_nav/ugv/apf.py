"""
Reactive obstacle avoidance for the UGV: artificial potential fields (APF).

LiDAR returns and the other UGVs (from MoCap) push the robot away. A small
vortex term also pushes it sideways, towards the side of its goal, so it
curves around an obstacle instead of stopping in front of it. An emergency
stop is the last resort when something is very close ahead.

Every vector here is in the robot body frame (x forward, y left). LiDAR
bearings are rotated into that frame with lidar_yaw_offset: on a TurtleBot 4
the scan frame rplidar_link is yawed +90 degrees from base_link.

The functions are plain Python so they can be unit-tested without ROS.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

Vector = Tuple[float, float]
Return = Tuple[float, float]  # (bearing in the body frame [rad], range [m])


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body(dx: float, dy: float, yaw: float) -> Vector:
    """Rotate a world-frame vector into the frame of a robot with heading yaw."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)


@dataclass(frozen=True)
class ApfParams:
    """Tuning of the avoidance layer; see config/params.yaml for the meaning."""

    lidar_yaw_offset: float = math.pi / 2.0
    lidar_min_range: float = 0.15
    influence: float = 0.8
    gain: float = 0.05
    vortex: float = 0.2
    robot_influence: float = 0.8
    robot_gain: float = 0.1
    robot_vortex: float = 0.2
    mask_arc: float = math.radians(15.0)
    mask_tolerance: float = 0.4
    stop_distance: float = 0.25
    stop_arc: float = math.radians(60.0)
    robot_stop_distance: float = 0.35


def scan_returns(ranges: Sequence[float], angle_min: float, angle_increment: float,
                 range_max: float, params: ApfParams, step: int = 1) -> List[Return]:
    """Turn LaserScan ranges into (bearing, range) pairs in the body frame."""
    returns = []
    for index in range(0, len(ranges), max(1, step)):
        rng = ranges[index]
        if not math.isfinite(rng) or rng < params.lidar_min_range or rng > range_max:
            continue
        bearing = angle_min + index * angle_increment + params.lidar_yaw_offset
        returns.append((wrap_to_pi(bearing), rng))
    return returns


def hits_other_robot(ret: Return, robots: Sequence[Vector], params: ApfParams) -> bool:
    """Return True if a LiDAR return probably comes from another UGV."""
    bearing, rng = ret
    for bx, by in robots:
        dist = math.hypot(bx, by)
        if dist > params.influence + params.mask_tolerance:
            continue
        if (abs(wrap_to_pi(bearing - math.atan2(by, bx))) < params.mask_arc
                and abs(rng - dist) < params.mask_tolerance):
            return True
    return False


def _vortex_side(goal: Vector, push: Vector) -> float:
    """
    Choose which way the vortex term turns: +1 or -1.

    It turns towards the side of the push that agrees with the goal, i.e. away
    from the obstacle. When the obstacle is (almost) dead ahead there is no such
    side and the choice is fixed to +1, which veers right; two robots meeting
    head-on then both veer right and pass each other.
    """
    perp = (-push[1], push[0])
    agreement = goal[0] * perp[0] + goal[1] * perp[1]
    if abs(agreement) <= 0.1 * math.hypot(*goal) * math.hypot(*push):
        return 1.0
    return 1.0 if agreement > 0.0 else -1.0


def _repulsion(push: Vector, magnitude: float, vortex: float, side: float) -> Vector:
    """Push away from an obstacle, plus a sideways vortex term."""
    perp = (-push[1], push[0])
    return (magnitude * (push[0] + vortex * side * perp[0]),
            magnitude * (push[1] + vortex * side * perp[1]))


def apf_velocity(goal: Vector, returns: Sequence[Return], robots: Sequence[Vector],
                 params: ApfParams) -> Vector:
    """
    Add obstacle and inter-robot repulsion to a goal velocity.

    goal is the go-to-goal velocity, returns come from scan_returns() and
    robots are the other UGVs' positions relative to this robot; all in the
    body frame. All LiDAR returns share one vortex direction, chosen from their
    combined push, so that the returns of one obstacle cannot cancel each
    other's vortex terms.
    """
    pushes = []
    for ret in returns:
        bearing, rng = ret
        if rng > params.influence or hits_other_robot(ret, robots, params):
            continue
        magnitude = params.gain * (1.0 / rng - 1.0 / params.influence) ** 2
        pushes.append((magnitude, (-math.cos(bearing), -math.sin(bearing))))

    vx, vy = goal
    if pushes:
        total = (sum(m * p[0] for m, p in pushes), sum(m * p[1] for m, p in pushes))
        side = _vortex_side(goal, total)
        for magnitude, push in pushes:
            rx, ry = _repulsion(push, magnitude, params.vortex, side)
            vx, vy = vx + rx, vy + ry
    for bx, by in robots:
        dist = math.hypot(bx, by)
        if not 0.01 < dist < params.robot_influence:
            continue
        magnitude = params.robot_gain * (1.0 / dist - 1.0 / params.robot_influence) ** 2
        push = (-bx / dist, -by / dist)
        rx, ry = _repulsion(push, magnitude, params.robot_vortex, _vortex_side(goal, push))
        vx, vy = vx + rx, vy + ry
    return vx, vy


def emergency_stop(returns: Sequence[Return], robots: Sequence[Vector],
                   params: ApfParams) -> Optional[str]:
    """Return why the robot must stop right now, or None if the way ahead is clear."""
    for bx, by in robots:
        dist = math.hypot(bx, by)
        # Only robots in front: a robot that has the other one behind can still drive away.
        if dist < params.robot_stop_distance and bx > 0.0:
            return f'another UGV {dist:.2f} m ahead'
    half_arc = params.stop_arc / 2.0
    for ret in returns:
        bearing, rng = ret
        if (abs(bearing) <= half_arc and rng < params.stop_distance
                and not hits_other_robot(ret, robots, params)):
            return f'obstacle {rng:.2f} m ahead'
    return None
