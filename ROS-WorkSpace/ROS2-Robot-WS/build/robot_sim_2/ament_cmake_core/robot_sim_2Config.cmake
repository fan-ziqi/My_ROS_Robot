# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robot_sim_2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robot_sim_2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robot_sim_2_FOUND FALSE)
  elseif(NOT robot_sim_2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robot_sim_2_FOUND FALSE)
  endif()
  return()
endif()
set(_robot_sim_2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robot_sim_2_FIND_QUIETLY)
  message(STATUS "Found robot_sim_2: 2.0.0 (${robot_sim_2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robot_sim_2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robot_sim_2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robot_sim_2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${robot_sim_2_DIR}/${_extra}")
endforeach()
