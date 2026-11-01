"""AI 夜間巡回デモ一式を起動する.

    ros2 launch ward_patrol ward_patrol.launch.py scenario:=A
    ros2 launch ward_patrol ward_patrol.launch.py scenario:=random layout_seed:=7

scenario: A / B / C / random (ward_patrol/ward_map.py の SCENARIOS を参照)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ward_patrol.ward_map import scenario_people


def spawn_people(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    seed = LaunchConfiguration('layout_seed').perform(context)
    people = scenario_people(scenario, int(seed) if seed else None)
    actions = []
    for name, _kind, _room, (x, y, z, r, p, yaw) in people:
        actions.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{name}',
            arguments=['-entity', name, '-database', 'ward_person',
                       '-x', str(x), '-y', str(y), '-z', str(z),
                       '-R', str(r), '-P', str(p), '-Y', str(yaw)],
            output='log'))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('ward_patrol')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(pkg, 'worlds', 'hospital_ward.world')
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='A'),
        DeclareLaunchArgument('layout_seed', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='/workspace/output'),

        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg, 'models')),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(tb3, 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # ロボット (patrol_robot) は world に含めてある (GUI カメラで追従するため)
        OpaqueFunction(function=spawn_people),

        Node(package='ward_patrol', executable='fall_detector', output='screen',
             parameters=[use_sim_time]),
        Node(package='ward_patrol', executable='ward_monitor', output='screen',
             parameters=[use_sim_time], condition=IfCondition(LaunchConfiguration('monitor'))),
        # 人物の配置と AI モデルの読み込みが終わってから巡回を始める
        TimerAction(period=8.0, actions=[
            Node(package='ward_patrol', executable='patrol_manager', output='screen',
                 parameters=[use_sim_time,
                             {'output_dir': LaunchConfiguration('output_dir')}]),
        ]),
    ])
