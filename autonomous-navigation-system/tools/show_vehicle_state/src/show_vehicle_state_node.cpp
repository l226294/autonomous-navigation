#include "show_vehicle_state/show_vehicle_state.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShowVehicleState>("show_ego_vehicle_state");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}