cmake_minimum_required(VERSION 3.5)
project(earth_to_map_tf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)

find_package(ament_cmake_auto REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(GEOGRAPHICLIB REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(earth_to_map_tf
  include/earth_to_map_tf/earth_to_map_tf.hpp
  src/earth_to_map_tf.cpp
  src/earth_to_map_tf_node.cpp
  )
include_directories(/usr/include/yaml-cpp)
target_link_libraries(earth_to_map_tf /usr/lib/x86_64-linux-gnu/libyaml-cpp.so)
target_link_libraries(earth_to_map_tf ${GeographicLib_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
