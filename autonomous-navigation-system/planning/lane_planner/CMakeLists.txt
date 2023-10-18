# Copyright 2020 The Autoware Foundation
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
# Set compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

project(lane_planner)

# dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(time_utils REQUIRED)
ament_auto_find_build_dependencies()

set(LANE_PLANNER_LIB_SRC
  src/lane_planner.cpp
)

set(LANE_PLANNER_LIB_HEADERS
  include/lane_planner/lane_planner.hpp
  include/lane_planner/visibility_control.hpp
)

# generate library
ament_auto_add_executable(${PROJECT_NAME}
  ${LANE_PLANNER_LIB_SRC}
  ${LANE_PLANNER_LIB_HEADERS}
)



# ament package generation and installing
ament_auto_package()
