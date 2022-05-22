import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_name = 'robot_sim'
    urdf_name = "robot.urdf" 
    package_path = FindPackageShare(package = package_name).find(package_name) 
    urdf_path = os.path.join(package_path, 'urdf', urdf_name)
    param_path = os.path.join(package_path, 'param', 'params.yaml')
    launch_path = os.path.join(package_path, 'launch')

    with open(urdf_path, 'r') as urdf:
        robot_desc = urdf.read()

    info = LogInfo(
        msg=['Execute Robot Sim']
    )

    robot_sim_node = Node(
        package=package_name,
        executable='robot_sim',
        parameters=[param_path],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'rviz2.launch.py')
        )
    )

    ld.add_action(info)
    ld.add_action(robot_sim_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_launch)

    return ld
