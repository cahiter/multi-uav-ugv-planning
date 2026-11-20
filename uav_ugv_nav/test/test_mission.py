import copy
import os
import re

import pytest
from uav_ugv_nav.mission import (
    constraint_violations, load_mission, MissionError, parse_mission, team_mission_time,
    TimingModel, Tour, tour_slack, uav_tour_time, ugv_time)

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(PACKAGE, 'missions', 'demo_two_teams.yaml')

MINIMAL = {
    'model': {},
    'teams': {
        't1': {
            'start': [0, 0],
            'end': [1, 0],
            'tours': [{'release': [0, 0], 'waypoints': [[0, 1], [1, 1, 1.2]], 'collect': [1, 0]}],
        },
    },
}


def minimal(**model):
    data = copy.deepcopy(MINIMAL)
    data['model'].update(model)
    return data


def test_demo_mission_loads_and_fits_its_budget():
    mission = load_mission(DEMO)
    assert set(mission.teams) == {'team1', 'team2'}
    assert [len(team.tours) for team in mission.teams.values()] == [3, 2]
    assert constraint_violations(mission) == []


def test_waypoint_altitude_defaults_to_flight_altitude():
    mission = parse_mission(minimal(flight_altitude=0.8))
    assert mission.team('t1').tours[0].waypoints == ((0.0, 1.0, 0.8), (1.0, 1.0, 1.2))


def _set(*path_and_value):
    *path, key, value = path_and_value

    def mutate(data):
        target = data
        for step in path:
            target = target[step]
        target[key] = value
    return mutate


def _drop(*path):
    *path, key = path

    def mutate(data):
        target = data
        for step in path:
            target = target[step]
        del target[key]
    return mutate


@pytest.mark.parametrize('mutate, message', [
    (_set('teams', 't1', 'tours', 0, 'waypionts', []), 'unknown key(s) waypionts'),
    (_drop('teams', 't1', 'tours', 0, 'collect'), 'missing collect'),
    (_set('teams', 't1', 'tours', 0, 'waypoints', []), 'non-empty list'),
    (_set('teams', 't1', 'tours', []), 'non-empty list of tours'),
    (_set('teams', 't1', 'start', [0]), 'expected [x, y]'),
    (_set('teams', 't1', 'start', ['a', 0]), 'expected a number'),
    (_set('model', 'uav_speed', 0), 'greater than zero'),
    (_set('model', 'hover_time', -1), 'must not be negative'),
    (_set('model', 'hover_tme', 1), 'unknown key(s) hover_tme'),
    (_set('teams', {}), 'at least one team'),
])
def test_malformed_missions_are_rejected(mutate, message):
    data = minimal()
    mutate(data)
    with pytest.raises(MissionError, match=re.escape(message)):
        parse_mission(data)


def test_unknown_team_is_rejected():
    with pytest.raises(MissionError, match='not in the mission'):
        parse_mission(minimal()).team('t9')


def test_tour_time_matches_hand_computation():
    model = TimingModel(uav_speed=0.5, min_leg_time=1.0, hover_time=2.0, takeoff_time=3.0,
                        settle_time=1.0, land_time=4.0, flight_altitude=1.0)
    tour = Tour(release=(0.0, 0.0), waypoints=((0.0, 3.0, 1.0), (0.2, 3.0, 1.0)),
                collect=(4.0, 3.0))
    # Legs: 3 m (6 s), 0.2 m (min_leg_time 1 s), 3.8 m (7.6 s); two hovers of 2 s.
    expected = 3.0 + 1.0 + 4.0 + 2 * 2.0 + 6.0 + 1.0 + 7.6
    assert uav_tour_time(tour, model) == pytest.approx(expected)


def test_slack_and_violations():
    mission = parse_mission(minimal(max_flight_time=30.0, min_air_slack=10.0, uav_speed=0.1))
    air, ground = tour_slack(mission.team('t1').tours[0], mission.model)
    assert air < 10.0 <= ground
    problems = constraint_violations(mission)
    assert len(problems) == 1 and 'air slack' in problems[0]


def test_team_mission_time():
    data = minimal(ugv_speed=1.0, recharge_time=5.0)
    first = data['teams']['t1']['tours'][0]
    data['teams']['t1']['tours'].append(dict(first, release=[3, 0], collect=[4, 0]))
    data['teams']['t1']['end'] = [6, 0]
    mission = parse_mission(data)
    model, team = mission.model, mission.team('t1')
    tour1, tour2 = team.tours
    expected = (
        ugv_time((0, 0), (0, 0), model) + ugv_time((4, 0), (6, 0), model)
        + max(uav_tour_time(tour1, model), 1.0) + max(uav_tour_time(tour2, model), 1.0)
        + max(2.0, 5.0))  # collect 1 -> release 2 is 2 m at 1 m/s, recharge takes 5 s
    assert team_mission_time(team, model) == pytest.approx(expected)
