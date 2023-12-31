# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(odom_to_state_conversion_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(ODOM_TO_STATE_CONVERSION_NODES_LIB_SRC
  src/odom_to_state_conversion_nodes.cpp
)

set(ODOM_TO_STATE_CONVERSION_NODES_LIB_HEADERS
  include/odom_to_state_conversion_nodes/odom_to_state_conversion_nodes.hpp
  include/odom_to_state_conversion_nodes/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${ODOM_TO_STATE_CONVERSION_NODES_LIB_SRC}
  ${ODOM_TO_STATE_CONVERSION_NODES_LIB_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "odom_to_state_conversion_nodes::OdomToStateConversionNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_odom_to_state_conversion_nodes.cpp)
  set(TEST_ODOM_TO_STATE_CONVERSION_NODES_EXE test_odom_to_state_conversion_nodes)
  ament_add_gtest(${TEST_ODOM_TO_STATE_CONVERSION_NODES_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_ODOM_TO_STATE_CONVERSION_NODES_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
