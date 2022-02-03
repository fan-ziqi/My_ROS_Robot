cmake_minimum_required(VERSION 3.5)
project(rplidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

add_definitions(-w)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
ament_target_dependencies(rplidarNode rclcpp sensor_msgs std_srvs)

install(TARGETS
  rplidarNode
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(rplidarNode
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/src>
    $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
