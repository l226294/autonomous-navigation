#include "control_transfer/control_transfer.hpp"
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlTransfer>("control_transfer");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}