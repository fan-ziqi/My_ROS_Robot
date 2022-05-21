#include <rclcpp/rclcpp.hpp>

#include "ros2_astra_camera/camera_driver.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<ros2_astra_camera::CameraDriver>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
