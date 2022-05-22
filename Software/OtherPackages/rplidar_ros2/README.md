# rplidar_ros2

## 简介

思岚rplidar ROS2功能包

## 编译

```bash
colcon build --packages-select rplidar_ros2 --symlink-install --merge-instal
```

## 运行节点

方法1: ros2 run

```bash
ros2 run rplidar_ros2 rplidarNode
```

方法2: ros2 launch

```bash
ros2 launch rplidar_ros2 rplidar.launch.py
```

## RVIZ

打开rviz2

```bash
rviz2
```

`fix frame`手动填写`laser`

左下角`add`->`By topic`->`LaserScan`

