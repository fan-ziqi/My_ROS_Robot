import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('astra_camera'),
        'config',
        'astra.yaml'
    )

    return LaunchDescription([
        Node(
            package="astra_camera",
            executable="astra_camera_node",
            name="astra_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
