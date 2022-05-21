import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    package_dir = get_package_share_directory('ros2_astra_camera')
    parameters_path = os.path.join(
        package_dir, 'launch', 'params', 'astra_pro.yaml')
    urdf = os.path.join(package_dir, 'launch', 'urdf', 'astra_pro.urdf')

    enable_color_cloud = LaunchConfiguration('enable_color_cloud')

    declare_enable_color_cloud_cmd = DeclareLaunchArgument(
        'enable_color_cloud', default_value='False', description='Whether to enable color cloud')

    start_depth_image_to_color_cloud_container_cmd = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(enable_color_cloud),
        composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    parameters=[{"queue_size": 5,
                                 "exact_sync": False}],
                    remappings=[('rgb/camera_info', 'camera/projector/camera_info'),
                                ('rgb/image_rect_color', 'camera/color/image_raw'),
                                ('depth_registered/image_rect',
                                 'camera/depth/image'),
                                ('points', 'camera/depth_registered/points')],
                ),
        ],
        output='screen',
    )

    start_depth_image_to_cloud_container_cmd = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(PythonExpression(['not ', enable_color_cloud])),
        composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    parameters=[{"queue_size": 5,
                                 "exact_sync": False}],
                    remappings=[('camera_info', 'camera/depth/camera_info'),
                                ('image_rect', 'camera/depth/image'),
                                ('points', 'camera/depth/points')],
                ),
        ],
        emulate_tty=True,
        output='screen',
    )

    start_astra_camera_node_cmd = Node(
        package="ros2_astra_camera",
        executable="astra_camera_node",
        name="astra_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameters_path]
    )

    start_uvc_camera_node_cmd = Node(
        package="ros2_astra_camera",
        executable="uvc_camera_node",
        name="uvc_camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameters_path]
    )

    start_robot_state_publisher_node_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        arguments=[urdf]
    )

    ld = LaunchDescription()
    ld.add_action(declare_enable_color_cloud_cmd)
    ld.add_action(start_depth_image_to_color_cloud_container_cmd)
    ld.add_action(start_depth_image_to_cloud_container_cmd)
    ld.add_action(start_astra_camera_node_cmd)
    ld.add_action(start_uvc_camera_node_cmd)
    ld.add_action(start_robot_state_publisher_node_cmd)

    return ld
