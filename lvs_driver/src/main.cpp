#include "lvs_driver/lvs_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lvs_driver::LvsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}