import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time_cfg = LaunchConfiguration('use_sim_time', default='true')

    robot_name_in_model = 'robot_gazebo'
    package_name = 'robot_gazebo'
    urdf_name = "robot.urdf" 
       
    package_path = FindPackageShare(package = package_name).find(package_name) 
    urdf_path = os.path.join(package_path, 'urdf', urdf_name)
    launch_path = os.path.join(package_path, 'launch')

    print('\033[92m' + "spawn_entity: Use urdf dir: " + urdf_path + '\033[0m')

    gazebo_world_path = os.path.join(package_path, 'worlds/migong.world')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(package_path, 'meshes')},
        output='screen'
    )

    # Launch the robot
    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-file', urdf_path ],
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

    ld.add_action(use_sim_time_arg)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(spawn_entity_node)

    return ld
