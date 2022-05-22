import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    urdf_path_cfg = LaunchConfiguration('urdf_path')
    urdf_path = urdf_path_cfg.perform(context)

    print('\033[92m' + "robot_state_publisher: Use urdf dir: " + urdf_path + '\033[0m')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_cfg,
            'robot_description': robot_desc
        }]
    )

    return [
        robot_state_publisher_node,
    ]

def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='robot.urdf',
        description='urdf_path'
    )

    ld.add_action(urdf_path_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
