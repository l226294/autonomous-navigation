// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lane_planner class.

#ifndef LANE_PLANNER__LANE_PLANNER_HPP_
#define LANE_PLANNER__LANE_PLANNER_HPP_

#include <lane_planner/visibility_control.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

#include <iostream>
#include <vector>

namespace autoware
{
/// \brief TODO(ryohsuke.mitsudome): Document namespaces!
namespace lane_planner
{

using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using geometry_msgs::msg::PoseArray;



using lanelet::LaneletMapConstPtr;

// utility functions
float distance2d(const TrajectoryPoint & p1, const TrajectoryPoint & p2);

// calculate curvature by circle fitting to three points
float calculate_curvature(
  const TrajectoryPoint & p1, const TrajectoryPoint & p2,
  const TrajectoryPoint & p3);



/// \brief A class for recording trajectories and replaying them as plans
class LanePlanner : public rclcpp::Node 
{
public:
  explicit LanePlanner(std::string name);

  Trajectory plan_trajectory(
    LaneletRoute::ConstSharedPtr had_map_route,
    const LaneletMapConstPtr & map);

private:
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  lanelet::LaneletMapPtr lanelet_map_;
  // trajectory planning sub functions
  TrajectoryPoints generate_base_trajectory(
    LaneletRoute::ConstSharedPtr had_map_route,
    const LaneletMapConstPtr & map);
  void set_angle(TrajectoryPoints * trajectory_points);
  void set_steering_angle(TrajectoryPoints * trajectory_points);
  void set_time_from_start(TrajectoryPoints * trajectory_points);
  void modify_velocity(Trajectory * trajectory);
  void onMap(autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg);
  void onRoute(LaneletRoute::ConstSharedPtr route_msg);
  PoseArray trajectory2PoseArray(const Trajectory & trajectory);

  Trajectory create_trajectory_message(
    const std_msgs::msg::Header & header,
    const TrajectoryPoints & trajectory_points);
};  // class LanePlanner

}  // namespace lane_planner
}  // namespace autoware

#endif  // LANE_PLANNER__LANE_PLANNER_HPP_
