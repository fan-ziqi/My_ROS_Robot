cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_server)

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/world_canvas_server
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

FILE(GLOB_RECURSE script_files scripts/*.py)

install(PROGRAMS ${script_files}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
