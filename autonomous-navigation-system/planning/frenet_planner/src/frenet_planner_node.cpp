#include "frenet_planner/frenet_planner.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<planning::FrenetPlanner>("frenet_planner");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
