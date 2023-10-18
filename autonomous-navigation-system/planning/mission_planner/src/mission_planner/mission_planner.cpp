// Copyright 2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mission_planner.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include<iostream>

#include <array>
#include <random>

namespace
{

using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;

LaneletPrimitive convert(const LaneletPrimitive & p)
{
  
  LaneletPrimitive primitive;
  primitive.id = p.id;
  primitive.primitive_type = p.primitive_type;
  return primitive;
}

LaneletSegment convert(const LaneletSegment & s)
{
  
  LaneletSegment segment;
  segment.preferred_primitive.id = s.preferred_primitive.id;
  segment.primitives.push_back(convert(s.preferred_primitive));
  for (const auto & p : s.primitives) {
    segment.primitives.push_back(convert(p));
  }
  return segment;
}

std::array<uint8_t, 16> generate_random_id()
{
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> engine(std::random_device{}());
  std::array<uint8_t, 16> id;
  std::generate(id.begin(), id.end(), std::ref(engine));
  return id;
}

}  // namespace

namespace mission_planner
{

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_("mission_planner", "mission_planner::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(this->get_logger(), "初始化mission_planner");

  map_frame_ = static_cast<std::string>(declare_parameter("map_frame").get<std::string>());

  planner_ = plugin_loader_.createSharedInstance("mission_planner::lanelet2::DefaultPlanner");
  planner_->initialize(this);

  odometry_ = nullptr;
  sub_odometry_ = create_subscription<Odometry>(
    "/carla/ego_vehicle/odometry", rclcpp::QoS(1),
    std::bind(&MissionPlanner::on_odometry, this, std::placeholders::_1));
  
  goal_pose_sub_ptr =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS(10),
    std::bind(&MissionPlanner::on_set_route_points, this, std::placeholders::_1));
  
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_marker_ = create_publisher<MarkerArray>("debug/route_marker", durable_qos);
  pub_route_ = create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/mission_planner/output/route", durable_qos);


}

void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;
}

void MissionPlanner::change_route(const LaneletRoute & route)
{
  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route));
}



// NOTE: The route services should be mutually exclusive by callback group.
void MissionPlanner::on_set_route_points(
  const geometry_msgs::msg::PoseStamped::SharedPtr req)
{

  if (!planner_->ready() || !odometry_){
    return;
  }
  // Use temporary pose stamped for transform.
  PoseStamped pose;
  pose.header = req->header;

  // Convert route points.
  PlannerPlugin::RoutePoints points;
  points.push_back(odometry_->pose.pose);
  pose.pose = req->pose;
  points.push_back(pose.pose);

  // Plan route.
  std::cout<<"Plan route"<<std::endl;
  LaneletRoute route = planner_->plan(points);
  if (route.segments.empty()) {
    std::cout<<"Plan route failed!"<<std::endl;
  }
  route.header.stamp = req->header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid.uuid = generate_random_id();

  // Update route.
  change_route(route);
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::MissionPlanner)
