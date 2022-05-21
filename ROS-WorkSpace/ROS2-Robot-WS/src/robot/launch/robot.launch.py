# 导入launch和node包
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

# 定义launch描述函数
def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time_cfg = LaunchConfiguration('use_sim_time', default='false')

    package_name = 'robot'
    urdf_name = "robot.urdf" 

    package_path = FindPackageShare(package = package_name).find(package_name) 
    launch_path = os.path.join(package_path, 'launch')
    urdf_path = os.path.join(package_path, 'urdf', urdf_name)
    params_path = os.path.join(package_path, 'config', 'params.yaml')

    # 调用节点
    start_robot = Node(
        package=package_name,
        executable='robot',
        parameters=[params_path,  {'publish_odom_transform': True}, {'odom_frame': "/odom"}],
        output='screen'
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_cfg,
            'urdf_path': urdf_path
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'rviz2.launch.py')
        )
    )

    # base_footprint_to_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['-0.035', '0', '0.95', '1.57', '0', '0', 'base_footprint', 'base_imu_link'],
    #     output='screen'
    # )

    ld.add_action(start_robot)
    ld.add_action(robot_state_publisher_launch)
    # ld.add_action(base_footprint_to_imu)
    ld.add_action(rviz_launch)

    return ld