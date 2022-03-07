# Linux串口库

测试环境：`Ubuntu20.04` + `ROS2-Galactic`

将tx与rx短接，即可实现自收自发测试

## 编译

```bash
colcon build --symlink-install --merge-install --packages-select serial_test
```

## 运行

```bash
ros2 run serial_test serial_test
```

