import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_astra_camera')

    rviz_config_file = LaunchConfiguration('rviz_config')

    return LaunchDescription([

        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                package_dir, 'launch', 'rviz', 'astra.rviz'),
            description='Full path to the RVIZ config file to use'),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]),
    ])
