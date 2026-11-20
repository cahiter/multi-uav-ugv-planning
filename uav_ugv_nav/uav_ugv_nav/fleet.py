"""
Fleet files: which robots form each team, and where they may go.

The mission says what each team does. The fleet file says which Crazyflie
and which TurtleBot 4 make up each team, how to reach them (radio URI, ROS
namespace, MoCap topic), and the safe volume of the arena. Keeping the two
apart lets the same mission run in another lab by swapping the fleet file.
"""

from dataclasses import dataclass
import re
from typing import Dict, List, Optional, Sequence, Tuple

from uav_ugv_nav.mission import (
    load_yaml, Mission, MissionError, number, reject_unknown_keys, require_keys,
    require_mapping)

_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


@dataclass(frozen=True)
class Arena:
    """Axis-aligned box in the MoCap world frame that all points must stay in."""

    x: Tuple[float, float]
    y: Tuple[float, float]
    z: Tuple[float, float]

    def contains(self, point: Sequence[float]) -> bool:
        """Return True if an (x, y) or (x, y, z) point is inside the box."""
        bounds = (self.x, self.y, self.z)[:len(point)]
        return all(low <= value <= high for value, (low, high) in zip(point, bounds))


@dataclass(frozen=True)
class UavConfig:
    """How to reach one Crazyflie."""

    name: str         # ROS namespace of its uav_node, e.g. cf1
    uri: str          # Crazyradio URI, e.g. radio://0/80/2M/E7E7E7E7E7
    mocap_topic: str  # PoseStamped topic of its MoCap rigid body


@dataclass(frozen=True)
class UgvConfig:
    """How to reach one TurtleBot 4."""

    name: str         # ROS namespace of the TurtleBot 4, e.g. ugv1
    mocap_topic: str  # PoseStamped topic of its MoCap rigid body


@dataclass(frozen=True)
class TeamConfig:
    """The robots of one team."""

    uav: UavConfig
    ugv: UgvConfig


@dataclass(frozen=True)
class Fleet:
    """All robots of the lab and the arena they share."""

    arena: Optional[Arena]
    teams: Dict[str, TeamConfig]


def load_fleet(path: str) -> Fleet:
    """Read and validate a fleet file."""
    return parse_fleet(load_yaml(path), source=path)


def parse_fleet(data, source: str = 'fleet') -> Fleet:
    """Validate the content of a fleet file that was already parsed from YAML."""
    require_mapping(data, source)
    reject_unknown_keys(data, {'arena', 'teams'}, source)
    arena = _parse_arena(data['arena'], f'{source}: arena') if data.get('arena') else None
    teams_data = data.get('teams')
    require_mapping(teams_data, f'{source}: teams')
    if not teams_data:
        raise MissionError(f'{source}: teams: the fleet needs at least one team')
    teams = {}
    for name, team in teams_data.items():
        where = f'{source}: teams.{name}'
        require_mapping(team, where)
        reject_unknown_keys(team, {'uav', 'ugv'}, where)
        require_keys(team, ('uav', 'ugv'), where)
        teams[str(name)] = TeamConfig(
            uav=UavConfig(**_robot(team['uav'], ('name', 'uri', 'mocap_topic'), f'{where}.uav')),
            ugv=UgvConfig(**_robot(team['ugv'], ('name', 'mocap_topic'), f'{where}.ugv')))
    _check_unique(teams, source)
    return Fleet(arena=arena, teams=teams)


def _parse_arena(data, where: str) -> Arena:
    require_mapping(data, where)
    reject_unknown_keys(data, {'x', 'y', 'z'}, where)
    require_keys(data, ('x', 'y', 'z'), where)
    bounds = {}
    for axis in ('x', 'y', 'z'):
        value = data[axis]
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise MissionError(f'{where}.{axis}: expected [min, max], got {value!r}')
        low, high = (number(v, f'{where}.{axis}') for v in value)
        if low >= high:
            raise MissionError(f'{where}.{axis}: min must be smaller than max')
        bounds[axis] = (low, high)
    return Arena(**bounds)


def _robot(data, keys: Sequence[str], where: str) -> Dict[str, str]:
    require_mapping(data, where)
    reject_unknown_keys(data, set(keys), where)
    require_keys(data, keys, where)
    values = {key: str(data[key]) for key in keys}
    if not _NAME.match(values['name']):
        raise MissionError(
            f'{where}.name: {values["name"]!r} is not a valid ROS name '
            '(letters, digits and underscores, starting with a letter)')
    if not values['mocap_topic'].startswith('/'):
        raise MissionError(f'{where}.mocap_topic: use an absolute topic name (starting with /)')
    return values


def _check_unique(teams: Dict[str, TeamConfig], source: str) -> None:
    seen = {}
    for team_name, team in teams.items():
        for label, value in (('robot name', team.uav.name), ('robot name', team.ugv.name),
                             ('MoCap topic', team.uav.mocap_topic),
                             ('MoCap topic', team.ugv.mocap_topic), ('radio URI', team.uav.uri)):
            key = (label, value)
            if key in seen:
                raise MissionError(
                    f'{source}: {label} {value!r} is used by both {seen[key]} and {team_name}')
            seen[key] = team_name


def fleet_problems(mission: Mission, fleet: Fleet) -> List[str]:
    """List reasons why the mission cannot run on this fleet; empty if it can."""
    problems = [
        f'mission team {name!r} has no robots in the fleet file'
        for name in mission.teams if name not in fleet.teams]
    if fleet.arena is not None:
        problems += arena_problems(mission, fleet.arena)
    return problems


def arena_problems(mission: Mission, arena: Arena) -> List[str]:
    """List every mission point that lies outside the arena."""
    problems = []
    if not arena.z[0] <= mission.model.flight_altitude <= arena.z[1]:
        problems.append(
            f'flight_altitude {mission.model.flight_altitude} m is outside the arena z range '
            f'{list(arena.z)}')
    for team in mission.teams.values():
        points = [('start', team.start), ('end', team.end)]
        for index, tour in enumerate(team.tours, start=1):
            points.append((f'tour {index} release', tour.release))
            points += [(f'tour {index} waypoint {k}', wp)
                       for k, wp in enumerate(tour.waypoints, start=1)]
            points.append((f'tour {index} collect', tour.collect))
        for label, point in points:
            if not arena.contains(point):
                coords = ', '.join(f'{v:g}' for v in point)
                problems.append(f'{team.name} {label} ({coords}) is outside the arena')
    return problems
