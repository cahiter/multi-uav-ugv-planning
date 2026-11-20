"""
Mission files: the plan that the UAV-UGV teams execute.

A mission gives every team a start point, an end point and a list of tours.
In each tour the UGV drops its UAV at a release point, the UAV visits a list
of waypoints, and the UGV picks it up again at a collect point. Any planner
can drive this framework by writing a mission file.

The module also holds the timing model that turns a tour into durations. The
UAV node flies with exactly these durations, so the estimates printed by
check_mission are what the robots do, minus hardware overheads.

Only PyYAML is needed here, so missions can be checked without ROS.
"""

from dataclasses import dataclass, fields
import math
from typing import Dict, List, Sequence, Tuple

import yaml

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


class MissionError(ValueError):
    """A mission or fleet file is malformed."""


@dataclass(frozen=True)
class TimingModel:
    """
    Timing model and flight-time budget of a mission.

    Times are in seconds, distances in meters, speeds in m/s.
    """

    max_flight_time: float = 90.0   # flight-time budget per tour
    min_air_slack: float = 20.0     # budget each tour's flight must leave unused
    min_ground_slack: float = 20.0  # same for the UGV's release->collect drive
    flight_altitude: float = 0.8    # altitude of waypoints given as [x, y]
    uav_speed: float = 0.3          # UAV speed between waypoints
    min_leg_time: float = 2.0       # shortest duration of a single UAV move
    hover_time: float = 3.0         # time spent hovering at each waypoint
    takeoff_time: float = 3.0       # climb from the deck to flight altitude
    settle_time: float = 2.0        # align above the UGV before descending
    land_time: float = 4.0          # descent onto the deck
    ugv_speed: float = 0.12         # average UGV speed, turns included
    recharge_time: float = 0.0      # minimum time on the deck between tours


@dataclass(frozen=True)
class Tour:
    """One release -> waypoints -> collect cycle of a team."""

    release: Point2
    waypoints: Tuple[Point3, ...]
    collect: Point2


@dataclass(frozen=True)
class Team:
    """The plan of one UAV-UGV team."""

    name: str
    start: Point2
    end: Point2
    tours: Tuple[Tour, ...]


@dataclass(frozen=True)
class Mission:
    """A timing model and one plan per team."""

    model: TimingModel
    teams: Dict[str, Team]

    def team(self, name: str) -> Team:
        """Return the plan of the team called name."""
        try:
            return self.teams[name]
        except KeyError:
            known = ', '.join(self.teams) or 'none'
            raise MissionError(f'team {name!r} is not in the mission (teams: {known})') from None


# ---------------------------------------------------------------------------
# Loading and validation
# ---------------------------------------------------------------------------

def load_yaml(path: str):
    """Read a YAML file, turning I/O and syntax errors into MissionError."""
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except OSError as error:
        raise MissionError(f'cannot read {path}: {error.strerror}') from None
    except yaml.YAMLError as error:
        raise MissionError(f'{path} is not valid YAML: {error}') from None


def load_mission(path: str) -> Mission:
    """Read and validate a mission file."""
    return parse_mission(load_yaml(path), source=path)


def parse_mission(data, source: str = 'mission') -> Mission:
    """Validate the content of a mission file that was already parsed from YAML."""
    require_mapping(data, source)
    reject_unknown_keys(data, {'model', 'teams'}, source)
    model = _parse_model(data.get('model') or {}, f'{source}: model')
    teams_data = data.get('teams')
    require_mapping(teams_data, f'{source}: teams')
    if not teams_data:
        raise MissionError(f'{source}: teams: the mission needs at least one team')
    teams = {}
    for name, team_data in teams_data.items():
        teams[str(name)] = _parse_team(str(name), team_data, model, f'{source}: teams.{name}')
    return Mission(model=model, teams=teams)


def _parse_model(data, where: str) -> TimingModel:
    require_mapping(data, where)
    reject_unknown_keys(data, {f.name for f in fields(TimingModel)}, where)
    values = {}
    for key, value in data.items():
        values[key] = number(value, f'{where}.{key}')
        if values[key] < 0.0:
            raise MissionError(f'{where}.{key}: must not be negative')
    model = TimingModel(**values)
    for key in ('max_flight_time', 'flight_altitude', 'uav_speed', 'ugv_speed'):
        if getattr(model, key) <= 0.0:
            raise MissionError(f'{where}.{key}: must be greater than zero')
    return model


def _parse_team(name: str, data, model: TimingModel, where: str) -> Team:
    require_mapping(data, where)
    reject_unknown_keys(data, {'start', 'end', 'tours'}, where)
    require_keys(data, ('start', 'end', 'tours'), where)
    tours_data = data['tours']
    if not isinstance(tours_data, list) or not tours_data:
        raise MissionError(f'{where}.tours: expected a non-empty list of tours')
    tours = tuple(
        _parse_tour(tour, model, f'{where}.tours[{index}]')
        for index, tour in enumerate(tours_data))
    return Team(
        name=name,
        start=point2(data['start'], f'{where}.start'),
        end=point2(data['end'], f'{where}.end'),
        tours=tours)


def _parse_tour(data, model: TimingModel, where: str) -> Tour:
    require_mapping(data, where)
    reject_unknown_keys(data, {'release', 'waypoints', 'collect'}, where)
    require_keys(data, ('release', 'waypoints', 'collect'), where)
    waypoints = data['waypoints']
    if not isinstance(waypoints, list) or not waypoints:
        raise MissionError(
            f'{where}.waypoints: expected a non-empty list of [x, y] or [x, y, z] points')
    return Tour(
        release=point2(data['release'], f'{where}.release'),
        waypoints=tuple(
            point3(point, model.flight_altitude, f'{where}.waypoints[{index}]')
            for index, point in enumerate(waypoints)),
        collect=point2(data['collect'], f'{where}.collect'))


def require_mapping(data, where: str) -> None:
    """Raise MissionError unless data is a YAML mapping."""
    if not isinstance(data, dict):
        raise MissionError(f'{where}: expected a mapping, got {type(data).__name__}')


def require_keys(data: dict, keys: Sequence[str], where: str) -> None:
    """Raise MissionError if any of keys is missing from data."""
    missing = [key for key in keys if key not in data]
    if missing:
        raise MissionError(f'{where}: missing {", ".join(missing)}')


def reject_unknown_keys(data: dict, allowed, where: str) -> None:
    """Raise MissionError on keys that are not allowed (catches typos)."""
    unknown = sorted(str(key) for key in data if key not in allowed)
    if unknown:
        raise MissionError(
            f'{where}: unknown key(s) {", ".join(unknown)}; '
            f'expected {", ".join(sorted(allowed))}')


def number(value, where: str) -> float:
    """Return value as a float, or raise MissionError if it is not a finite number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise MissionError(f'{where}: expected a number, got {value!r}')
    return float(value)


def point2(value, where: str) -> Point2:
    """Parse an [x, y] point."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise MissionError(f'{where}: expected [x, y], got {value!r}')
    return (number(value[0], where), number(value[1], where))


def point3(value, default_z: float, where: str) -> Point3:
    """Parse an [x, y] or [x, y, z] point; a missing z becomes default_z."""
    if not isinstance(value, (list, tuple)) or len(value) not in (2, 3):
        raise MissionError(f'{where}: expected [x, y] or [x, y, z], got {value!r}')
    coords = [number(v, where) for v in value]
    return (coords[0], coords[1], coords[2] if len(coords) == 3 else default_z)


# ---------------------------------------------------------------------------
# Timing model
# ---------------------------------------------------------------------------

def distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Return the Euclidean distance between two points of the same dimension."""
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


def leg_time(a: Sequence[float], b: Sequence[float], model: TimingModel) -> float:
    """Return how long the UAV takes to fly from a to b."""
    return max(model.min_leg_time, distance(a, b) / model.uav_speed)


def uav_tour_time(tour: Tour, model: TimingModel) -> float:
    """
    Estimate the UAV flight time of a tour, from takeoff to touchdown.

    Take off above the release point, fly to each waypoint and hover there,
    fly to the collect point, settle above the UGV and land.
    """
    altitude = model.flight_altitude
    path = [(*tour.release, altitude), *tour.waypoints, (*tour.collect, altitude)]
    total = model.takeoff_time + model.settle_time + model.land_time
    total += len(tour.waypoints) * model.hover_time
    total += sum(leg_time(a, b, model) for a, b in zip(path, path[1:]))
    return total


def ugv_time(a: Point2, b: Point2, model: TimingModel) -> float:
    """Estimate how long the UGV takes to drive from a to b."""
    return distance(a, b) / model.ugv_speed


def tour_slack(tour: Tour, model: TimingModel) -> Tuple[float, float]:
    """
    Return the (air, ground) slack of a tour under the timing model.

    Air slack is the part of the flight-time budget that the UAV's flight
    leaves unused; ground slack is what the UGV's release->collect drive leaves
    unused. A tour is acceptable when they reach min_air_slack and
    min_ground_slack: that margin absorbs delays during execution.
    """
    return (model.max_flight_time - uav_tour_time(tour, model),
            model.max_flight_time - ugv_time(tour.release, tour.collect, model))


def team_mission_time(team: Team, model: TimingModel) -> float:
    """
    Estimate the time a team needs for its whole plan.

    The drive to the first release point, then for every tour the slower of
    the UAV's flight and the UGV's drive, between tours the longer of the
    drive to the next release point and the recharge time, and finally the
    drive to the end point.
    """
    tours = team.tours
    total = ugv_time(team.start, tours[0].release, model)
    total += ugv_time(tours[-1].collect, team.end, model)
    for tour in tours:
        total += max(uav_tour_time(tour, model), ugv_time(tour.release, tour.collect, model))
    for current, following in zip(tours, tours[1:]):
        total += max(ugv_time(current.collect, following.release, model), model.recharge_time)
    return total


def constraint_violations(mission: Mission) -> List[str]:
    """List the tours that leave less slack than the mission requires; empty if none."""
    model = mission.model
    problems = []
    for team in mission.teams.values():
        for index, tour in enumerate(team.tours, start=1):
            air, ground = tour_slack(tour, model)
            if air < model.min_air_slack:
                problems.append(
                    f'{team.name} tour {index}: air slack {air:.1f} s < min_air_slack '
                    f'{model.min_air_slack:.1f} s (estimated flight '
                    f'{uav_tour_time(tour, model):.1f} s, budget {model.max_flight_time:.1f} s)')
            if ground < model.min_ground_slack:
                problems.append(
                    f'{team.name} tour {index}: ground slack {ground:.1f} s < min_ground_slack '
                    f'{model.min_ground_slack:.1f} s (estimated drive '
                    f'{ugv_time(tour.release, tour.collect, model):.1f} s, '
                    f'budget {model.max_flight_time:.1f} s)')
    return problems
