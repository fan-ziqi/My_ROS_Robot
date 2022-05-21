cmake_minimum_required(VERSION 3.8)
project(ros2_astra_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated -Wno-error -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(astra_msgs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(libuvc REQUIRED)

set(dependencies astra_msgs rclcpp rclcpp_components sensor_msgs
                 image_transport camera_info_manager)

option(FILTER "OB Filter library" ON)
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
if((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
  # for schroot enviroment
elseif((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "x86")
elseif(${MACHINES} MATCHES "x86")

elseif(${MACHINES} MATCHES "x86")
  set(HOST_PLATFORM "x86")
elseif(${MACHINES} MATCHES "i686")
  set(HOST_PLATFORM "x86")
elseif(${MACHINES} MATCHES "i386")
  set(HOST_PLATFORM "x86")
elseif(${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm")
elseif((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm")
endif()

set(ORBBEC_OPENNI2_REDIST
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/openni2_redist/${HOST_PLATFORM}")
link_directories(${ORBBEC_OPENNI2_REDIST})

include_directories(include lib lib/openni2
                    ${CMAKE_CURRENT_BINARY_DIR}/openni2_redist/${HOST_PLATFORM})

add_library(
  astra_wrapper
  src/astra_wrapper/astra_convert.cpp
  src/astra_wrapper/astra_device.cpp
  src/astra_wrapper/astra_device_info.cpp
  src/astra_wrapper/astra_timer_filter.cpp
  src/astra_wrapper/astra_frame_listener.cpp
  src/astra_wrapper/astra_device_manager.cpp
  src/astra_wrapper/astra_exception.cpp
  src/astra_wrapper/astra_video_mode.cpp
  src/astra_wrapper/astra_device_type.cpp)
ament_target_dependencies(astra_wrapper ${dependencies})
target_link_libraries(astra_wrapper -lOpenNI2 -L${ORBBEC_OPENNI2_REDIST})

add_library(astra_driver SHARED src/astra_driver.cpp)
target_compile_definitions(astra_driver PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(astra_driver ${dependencies})
target_link_libraries(astra_driver astra_wrapper)
rclcpp_components_register_nodes(astra_driver "ros2_astra_camera::AstraDriver")

add_library(camera_driver SHARED src/camera_driver.cpp)
target_compile_definitions(camera_driver PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(camera_driver ${dependencies})
target_link_libraries(camera_driver astra_wrapper ${libuvc_LIBRARIES})
rclcpp_components_register_nodes(camera_driver
                                 "ros2_astra_camera::CameraDriver")

add_executable(astra_camera_node src/astra_camera_node.cpp)
target_link_libraries(astra_camera_node astra_driver)
ament_target_dependencies(astra_camera_node ${dependencies})

add_executable(uvc_camera_node src/camera_driver_node.cpp)
target_link_libraries(uvc_camera_node camera_driver)
ament_target_dependencies(uvc_camera_node ${dependencies})

install(TARGETS astra_camera_node uvc_camera_node DESTINATION lib/${PROJECT_NAME})

install(FILES lib/openni2_redist/${HOST_PLATFORM}/libOpenNI2.so
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY lib/openni2_redist/${HOST_PLATFORM}/OpenNI2
        DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS astra_wrapper camera_driver astra_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(astra_wrapper)
ament_package()
