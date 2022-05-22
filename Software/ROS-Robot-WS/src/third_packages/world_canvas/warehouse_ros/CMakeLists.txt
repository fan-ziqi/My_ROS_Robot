cmake_minimum_required(VERSION 2.8.3)
project(warehouse_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostime
  std_msgs
  geometry_msgs
  pluginlib
  tf
)
find_package(Boost COMPONENTS system filesystem thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_ros
  CATKIN_DEPENDS roscpp rostime std_msgs geometry_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(warehouse_srcs
  src/database_loader.cpp
  src/transform_collection.cpp)

add_library(warehouse_ros SHARED ${warehouse_srcs})

target_link_libraries(warehouse_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(warehouse_test_dbloader src/test_dbloader.cpp)
target_link_libraries(warehouse_test_dbloader warehouse_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS warehouse_ros
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
