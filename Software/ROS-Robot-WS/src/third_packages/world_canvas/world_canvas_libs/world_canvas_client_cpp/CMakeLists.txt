cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_client_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  unique_id
  uuid_msgs
  visualization_msgs
  world_canvas_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES world_canvas_client
  CATKIN_DEPENDS roscpp unique_id uuid_msgs visualization_msgs world_canvas_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(world_canvas_client src/annotation_collection.cpp
                                src/world_collection.cpp
                                src/filter_criteria.cpp)
add_dependencies(world_canvas_client uuid_msgs_gencpp world_canvas_msgs_gencpp)
target_link_libraries(world_canvas_client ${catkin_LIBRARIES})

install(TARGETS world_canvas_client LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
