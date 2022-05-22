cmake_minimum_required(VERSION 2.8.3)
project(astra_launch)

find_package(catkin REQUIRED)

catkin_package()

# install the launch files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

