cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_client_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  world_canvas_msgs
)

catkin_package()
catkin_python_setup()
