cmake_minimum_required(VERSION 3.8)
project(astra_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies rosidl_default_generators sensor_msgs)

set(srv_files
    "srv/GetCameraInfo.srv"
    "srv/GetDeviceType.srv"
    "srv/GetIRExposure.srv"
    "srv/GetIRGain.srv"
    "srv/GetSerial.srv"
    "srv/GetUVCExposure.srv"
    "srv/GetUVCGain.srv"
    "srv/GetUVCWhiteBalance.srv"
    "srv/GetVersion.srv"
    "srv/ResetIRExposure.srv"
    "srv/ResetIRGain.srv"
    "srv/SetAeEnable.srv"
    "srv/SetAutoExposure.srv"
    "srv/SetAutoWhiteBalance.srv"
    "srv/SetDistortioncal.srv"
    "srv/SetFan.srv"
    "srv/SetIRExposure.srv"
    "srv/SetIRFlood.srv"
    "srv/SetIRGain.srv"
    "srv/SetLaser.srv"
    "srv/SetLDP.srv"
    "srv/SetMirror.srv"
    "srv/SetUVCExposure.srv"
    "srv/SetUVCGain.srv"
    "srv/SetUVCWhiteBalance.srv"
    "srv/SwitchIRCamera.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files} DEPENDENCIES
                           ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
