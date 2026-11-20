"""
Check a mission file before running it.

Prints, for every tour, the estimated UAV flight time and UGV driving time and
the resulting air and ground slack, and the estimated mission time of each
team. Exits with status 1 if a tour leaves less slack than the mission
requires or, with --fleet, if a point lies outside the arena or a team has no
robots.

  ros2 run uav_ugv_nav check_mission missions/demo_two_teams.yaml --fleet config/fleet.yaml
"""

import argparse
import sys

from uav_ugv_nav.fleet import fleet_problems, load_fleet
from uav_ugv_nav.mission import (
    constraint_violations, load_mission, Mission, MissionError, team_mission_time, tour_slack,
    ugv_time, uav_tour_time)


def report(mission: Mission, source: str) -> str:
    """Return a human-readable summary of the mission and its timing."""
    model = mission.model
    lines = [
        f'Mission {source}',
        f'  budget: {model.max_flight_time:g} s of flight per tour, '
        f'air slack >= {model.min_air_slack:g} s, ground slack >= {model.min_ground_slack:g} s',
        f'  speeds: UAV {model.uav_speed:g} m/s, UGV {model.ugv_speed:g} m/s, '
        f'altitude {model.flight_altitude:g} m',
    ]
    makespan, slowest = 0.0, None
    for team in mission.teams.values():
        n_points = sum(len(tour.waypoints) for tour in team.tours)
        lines += [
            '',
            f'{team.name}: {len(team.tours)} tours, {n_points} waypoints',
            '  tour  waypoints  UAV flight  air slack  UGV drive  ground slack',
        ]
        for index, tour in enumerate(team.tours, start=1):
            air, ground = tour_slack(tour, model)
            ok = air >= model.min_air_slack and ground >= model.min_ground_slack
            lines.append(
                f'  {index:>4}  {len(tour.waypoints):>9}  {uav_tour_time(tour, model):>8.1f} s'
                f'  {air:>7.1f} s  {ugv_time(tour.release, tour.collect, model):>7.1f} s'
                f'  {ground:>10.1f} s  {"ok" if ok else "VIOLATED"}')
        total = team_mission_time(team, model)
        lines.append(f'  estimated mission time: {total:.1f} s')
        if total > makespan:
            makespan, slowest = total, team.name
    lines += ['', f'Estimated makespan: {makespan:.1f} s ({slowest})']
    return '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Check a uav_ugv_nav mission file against its timing model and the arena.')
    parser.add_argument('mission', help='mission YAML file')
    parser.add_argument('--fleet', help='fleet YAML file: also check robots and arena bounds')
    args = parser.parse_args(argv)
    try:
        mission = load_mission(args.mission)
        fleet = load_fleet(args.fleet) if args.fleet else None
    except MissionError as error:
        print(f'error: {error}', file=sys.stderr)
        return 2

    print(report(mission, args.mission))
    problems = constraint_violations(mission)
    if fleet is not None:
        problems += fleet_problems(mission, fleet)
    print()
    if problems:
        print(f'{len(problems)} problem(s):')
        print('\n'.join(f'  - {problem}' for problem in problems))
        return 1
    inside = ', and every point is inside the arena' if fleet is not None else ''
    print(f'OK: every tour leaves the required slack{inside}.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
