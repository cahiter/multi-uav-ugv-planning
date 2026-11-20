"""
Launch the UAV and UGV nodes of the teams in a mission.

  ros2 launch uav_ugv_nav mission.launch.py                                 # everything
  ros2 launch uav_ugv_nav mission.launch.py teams:=team1                    # one team
  ros2 launch uav_ugv_nav mission.launch.py robots:=ugv standalone:=true    # UGVs alone

Before any node starts, the mission is checked against the fleet: the launch
fails if a mission team has no robots or a point lies outside the arena, and
it warns about tours that break the timing budget (see check_mission).
Each node runs in the namespace of its robot: /cf1/uav_node, /ugv1/ugv_node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_ugv_nav.fleet import fleet_problems, load_fleet
from uav_ugv_nav.mission import constraint_violations, load_mission


def generate_launch_description():
    share = get_package_share_directory('uav_ugv_nav')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value=os.path.join(share, 'missions', 'demo_two_teams.yaml'),
            description='mission file: what each team does'),
        DeclareLaunchArgument(
            'fleet', default_value=os.path.join(share, 'config', 'fleet.yaml'),
            description='fleet file: the robots of each team and the arena'),
        DeclareLaunchArgument(
            'params', default_value=os.path.join(share, 'config', 'params.yaml'),
            description='tuning of the nodes'),
        DeclareLaunchArgument(
            'teams', default_value='all',
            description='comma-separated names of the teams to start, or all'),
        DeclareLaunchArgument(
            'robots', default_value='all', choices=['all', 'uav', 'ugv'],
            description='start only the UAV nodes or only the UGV nodes'),
        DeclareLaunchArgument(
            'standalone', default_value='false', choices=['true', 'false'],
            description='UAVs fly and UGVs drive their tours without their partner'),
        # After Ctrl+C, give flying UAVs time to land before they are terminated.
        DeclareLaunchArgument('sigterm_timeout', default_value='15'),
        OpaqueFunction(function=_launch_teams),
    ])


def _launch_teams(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mission_file = os.path.abspath(os.path.expanduser(arg('mission')))
    fleet_file = os.path.abspath(os.path.expanduser(arg('fleet')))
    params_file = os.path.abspath(os.path.expanduser(arg('params')))
    mission, fleet = load_mission(mission_file), load_fleet(fleet_file)
    problems = fleet_problems(mission, fleet)
    if problems:
        raise RuntimeError(
            f'cannot run {mission_file} with {fleet_file}:\n  - ' + '\n  - '.join(problems))

    if arg('teams') == 'all':
        selected = list(mission.teams)
    else:
        selected = [name.strip() for name in arg('teams').split(',') if name.strip()]
    unknown = [name for name in selected if name not in mission.teams]
    if unknown:
        raise RuntimeError(
            f'unknown team(s) {", ".join(unknown)}; the mission has {", ".join(mission.teams)}')

    robots = arg('robots')
    standalone = arg('standalone') == 'true'
    actions = [LogInfo(msg=f'WARNING (timing budget): {problem}')
               for problem in constraint_violations(mission)]
    for name in selected:
        team = fleet.teams[name]
        common = {'mission_file': mission_file, 'team': name, 'standalone': standalone}
        if robots in ('all', 'uav'):
            uav_params = dict(common, uri=team.uav.uri, mocap_topic=team.uav.mocap_topic,
                              ugv_mocap_topic=team.ugv.mocap_topic)
            actions.append(Node(
                package='uav_ugv_nav', executable='uav_node', name='uav_node',
                namespace=team.uav.name, output='screen', emulate_tty=True,
                parameters=[params_file, uav_params]))
        if robots in ('all', 'ugv'):
            ugv_params = dict(common, mocap_topic=team.ugv.mocap_topic, uav_name=team.uav.name)
            # Every other UGV of the fleet is an obstacle, even if its team is not running.
            others = [t.ugv.mocap_topic for n, t in fleet.teams.items() if n != name]
            if others:
                ugv_params['other_mocap_topics'] = others
            actions.append(Node(
                package='uav_ugv_nav', executable='ugv_node', name='ugv_node',
                namespace=team.ugv.name, output='screen', emulate_tty=True,
                parameters=[params_file, ugv_params]))
    return actions
