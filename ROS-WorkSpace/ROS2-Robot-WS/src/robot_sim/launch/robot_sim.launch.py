import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory('robot_sim'), 'param', 'params.yaml'))

    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(get_package_share_directory('robot_sim'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot_sim.urdf'

    urdf = os.path.join(get_package_share_directory('robot_sim'), 'urdf', urdf_file_name)

    return LaunchDescription([
        LogInfo(msg=['Execute Robot Sim']),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='parameter 路径'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_dir, '/rviz2.launch.py'])),

        Node(
            package='robot_sim',
            executable='robot_sim',
            parameters=[param_dir],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
