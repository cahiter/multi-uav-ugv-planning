import copy
import os

import pytest
from uav_ugv_nav.fleet import fleet_problems, load_fleet, parse_fleet
from uav_ugv_nav.mission import load_mission, MissionError

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO_FLEET = os.path.join(PACKAGE, 'config', 'fleet.yaml')
DEMO_MISSION = os.path.join(PACKAGE, 'missions', 'demo_two_teams.yaml')

FLEET = {
    'arena': {'x': [-5, 5], 'y': [-5, 5], 'z': [0, 2]},
    'teams': {
        'team1': {
            'uav': {'name': 'cf1', 'uri': 'radio://0/80/2M/E7E7E7E7E7',
                    'mocap_topic': '/Robot_3/pose'},
            'ugv': {'name': 'ugv1', 'mocap_topic': '/Robot_1/pose'},
        },
        'team2': {
            'uav': {'name': 'cf2', 'uri': 'radio://1/100/2M/E7E7E7E7E8',
                    'mocap_topic': '/Robot_4/pose'},
            'ugv': {'name': 'ugv2', 'mocap_topic': '/Robot_2/pose'},
        },
    },
}


def fleet(**changes):
    data = copy.deepcopy(FLEET)
    data.update(changes)
    return data


def test_demo_fleet_runs_the_demo_mission():
    assert fleet_problems(load_mission(DEMO_MISSION), load_fleet(DEMO_FLEET)) == []


def test_points_outside_the_arena_are_reported():
    tiny = parse_fleet(fleet(arena={'x': [-1, 0], 'y': [0, 1], 'z': [0, 2]}))
    problems = fleet_problems(load_mission(DEMO_MISSION), tiny)
    assert problems and all('outside the arena' in problem for problem in problems)


def test_flight_altitude_above_the_arena_is_reported():
    low = parse_fleet(fleet(arena={'x': [-5, 5], 'y': [-5, 5], 'z': [0, 0.5]}))
    problems = fleet_problems(load_mission(DEMO_MISSION), low)
    assert any('flight_altitude' in problem for problem in problems)


def test_mission_team_without_robots_is_reported():
    data = fleet()
    del data['teams']['team2']
    problems = fleet_problems(load_mission(DEMO_MISSION), parse_fleet(data))
    assert problems == ["mission team 'team2' has no robots in the fleet file"]


def test_shared_mocap_topic_is_rejected():
    data = fleet()
    data['teams']['team2']['ugv']['mocap_topic'] = '/Robot_1/pose'
    with pytest.raises(MissionError, match='used by both team1 and team2'):
        parse_fleet(data)


def test_robot_names_must_be_valid_ros_names():
    data = fleet()
    data['teams']['team1']['uav']['name'] = 'cf-1'
    with pytest.raises(MissionError, match='not a valid ROS name'):
        parse_fleet(data)
