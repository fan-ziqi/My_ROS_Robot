# 导入launch和node包
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# 定义launch描述函数
def generate_launch_description():
    params_dir = os.path.join(get_package_share_directory('robot'), 'config', 'params.yaml')
    return LaunchDescription([
        
        # 调用节点
        Node(
            package='robot',
            executable='robot',
            parameters=[params_dir,  {'publish_odom_transform': True}, {'odom_frame': "/odom"}],
            output='screen'
        )
    ])