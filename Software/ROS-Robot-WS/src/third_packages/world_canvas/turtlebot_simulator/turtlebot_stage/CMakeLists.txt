cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_stage)

find_package(catkin REQUIRED)

catkin_package()

catkin_add_env_hooks(25.turtlebot-stage SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
