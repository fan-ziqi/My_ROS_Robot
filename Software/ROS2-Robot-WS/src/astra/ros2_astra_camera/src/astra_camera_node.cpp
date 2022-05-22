#include "ros2_astra_camera/astra_driver.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::Node::SharedPtr node =
      std::make_shared<ros2_astra_camera::AstraDriver>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
