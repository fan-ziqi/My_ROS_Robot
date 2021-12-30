cmake_minimum_required(VERSION 2.8.3)
project(astra_camera)

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(libuvc REQUIRED)

#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m  OUTPUT_VARIABLE  MACHINES )
execute_process(COMMAND getconf LONG_BIT  OUTPUT_VARIABLE  MACHINES_BIT )
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
IF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "x64")
# for schroot enviroment
ELSEIF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "x86" )
ELSEIF ( ${MACHINES} MATCHES "x86" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i686" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i386" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "arm" )
      set(HOST_PLATFORM  "arm")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "arm64")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "arm")
ENDIF ()

message(STATUS "ORRBEC : ${HOST_PLATFORM}")
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

generate_dynamic_reconfigure_options(cfg/Astra.cfg cfg/UVCCamera.cfg)

add_service_files(
  FILES
  GetSerial.srv
  GetDeviceType.srv
  GetIRGain.srv
  GetCameraInfo.srv
  GetUVCExposure.srv
  GetIRExposure.srv
  GetUVCGain.srv
  GetUVCWhiteBalance.srv
  SetUVCWhiteBalance.srv
  SetUVCGain.srv
  SetIRExposure.srv
  SetIRGain.srv
  SetIRFlood.srv
  SetLaser.srv
  SetLDP.srv
  SetUVCExposure.srv
  ResetIRGain.srv
  ResetIRExposure.srv
  SwitchIRCamera.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astra_wrapper
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_runtime
  #DEPENDS libastra
)

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})

set(ORBBEC_OPENNI2_REDIST  "${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/${HOST_PLATFORM}")
link_directories(${ORBBEC_OPENNI2_REDIST}
                 ${catkin_LINK_DIRS}
                 )

#MESSAGE(STATUS "ORRBEC : ${ORBBEC_OPENNI2_REDIST}")

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2
                    ${libuvc_INCLUDE_DIRS}
                    )


add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
   src/astra_device_type.cpp
)
target_link_libraries(astra_wrapper ${catkin_LIBRARIES} -lOpenNI2  -L${ORBBEC_OPENNI2_REDIST}
  ${Boost_LIBRARIES} )
add_dependencies(astra_wrapper ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_driver_lib
                src/astra_driver.cpp
                src/astra_device_type.cpp
)
target_link_libraries(astra_driver_lib astra_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_driver_lib ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(astra_camera_nodelet
   ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(astra_camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(astra_list_devices
   src/list_devices.cpp
)
target_link_libraries(astra_list_devices astra_wrapper)

add_executable(astra_test_wrapper test/test_wrapper.cpp )
target_link_libraries(astra_test_wrapper astra_wrapper ${Boost_LIBRARIES})
if (UNIX AND NOT APPLE)
  add_executable(astra_usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "astra_usb_reset")
endif()

add_executable(camera_node src/libuvc_camera/main.cpp
                           src/libuvc_camera/camera_driver.cpp
                           src/astra_device_type.cpp)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_library(libuvc_camera_nodelet src/libuvc_camera/nodelet.cpp
                                  src/libuvc_camera/camera_driver.cpp
                                  src/astra_device_type.cpp)
target_link_libraries(libuvc_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS astra_wrapper astra_camera_nodelet astra_camera_node astra_list_devices astra_driver_lib ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES include/openni2_redist/x64/libOpenNI2.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

install(DIRECTORY include/openni2_redist/x64/OpenNI2
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

# add xml file
install(FILES astra_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES 56-orbbec-usb.rules
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

install(FILES 56-orbbec-usb.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS camera_node libuvc_camera_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)