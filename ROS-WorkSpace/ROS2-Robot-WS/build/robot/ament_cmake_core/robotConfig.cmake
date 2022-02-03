# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robot_FOUND FALSE)
  elseif(NOT robot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robot_FOUND FALSE)
  endif()
  return()
endif()
set(_robot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robot_FIND_QUIETLY)
  message(STATUS "Found robot: 2.0.0 (${robot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robot_DIR}/${_extra}")
endforeach()
