cmake_minimum_required(VERSION 3.14)
project(map_based_prediction)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(motion_utils REQUIRED)
include_directories(/home/luo/ros2_autoware_ws/install/motion_utils/include/motion_utils)
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(map_based_prediction_node SHARED
  src/map_based_prediction_node.cpp
  src/path_generator.cpp
  src/debug.cpp
)

rclcpp_components_register_node(map_based_prediction_node
  PLUGIN "map_based_prediction::MapBasedPredictionNode"
  EXECUTABLE map_based_prediction
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
