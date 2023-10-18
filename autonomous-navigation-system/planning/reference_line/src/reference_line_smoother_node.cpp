#include "reference_line/reference_line_smoother.hpp"

using namespace planning;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReferenceLineSmoother>("reference_line_smoother");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}