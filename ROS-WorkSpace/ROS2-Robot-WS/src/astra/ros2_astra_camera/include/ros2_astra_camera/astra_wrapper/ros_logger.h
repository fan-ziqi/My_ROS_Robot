#ifndef ROS_LOGGER_H_
#define ROS_LOGGER_H_

#include <rcutils/logging_macros.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#define ROS_INFO(...) RCUTILS_LOG_INFO_NAMED("astra_wrapper", __VA_ARGS__)
#define ROS_DEBUG(...) RCUTILS_LOG_DEBUG_NAMED("astra_wrapper", __VA_ARGS__)
#define ROS_WARN(...) RCUTILS_LOG_WARN_NAMED("astra_wrapper", __VA_ARGS__)
#define ROS_ERROR(...) RCUTILS_LOG_ERROR_NAMED("astra_wrapper", __VA_ARGS__)
#define ROS_WARN_ONCE(...) \
  RCUTILS_LOG_WARN_ONCE_NAMED("astra_wrapper", __VA_ARGS__)

#define ROS_INFO_STREAM(stream_arg)                                  \
  do {                                                               \
    std::stringstream ss;                                            \
    ss << stream_arg;                                                \
    RCUTILS_LOG_INFO_NAMED("astra_wrapper", "%s", ss.str().c_str()); \
  } while (0)

#define ROS_DEBUG_STREAM(stream_arg)                                  \
  do {                                                                \
    std::stringstream ss;                                             \
    ss << stream_arg;                                                 \
    RCUTILS_LOG_DEBUG_NAMED("astra_wrapper", "%s", ss.str().c_str()); \
  } while (0)

#define ROS_ERROR_STREAM(stream_arg)                                  \
  do {                                                                \
    std::stringstream ss;                                             \
    ss << stream_arg;                                                 \
    RCUTILS_LOG_ERROR_NAMED("astra_wrapper", "%s", ss.str().c_str()); \
  } while (0)

#define ROS_WARN_STREAM(stream_arg)                                  \
  do {                                                               \
    std::stringstream ss;                                            \
    ss << stream_arg;                                                \
    RCUTILS_LOG_WARN_NAMED("astra_wrapper", "%s", ss.str().c_str()); \
  } while (0)

#endif  // ROS_LOGGER_H_
