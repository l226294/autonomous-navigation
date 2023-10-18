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

#include "lane_planner/lane_planner.hpp"
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <had_map_utils/had_map_utils.hpp>
#include "tf2/utils.h"
#include <limits>
#include <algorithm>
#include <math.h>

#include <time_utils/time_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
using std::placeholders::_1;
namespace autoware
{
namespace lane_planner
{

//求两点之间的距离
float distance2d(const TrajectoryPoint & p1, const TrajectoryPoint & p2)
{
  return sqrtf(static_cast<float>(std::pow(p1.pose.position.x - p2.pose.position.x,2) + std::pow(p1.pose.position.y - p2.pose.position.y,2)));
}

// calculate curvature by circle fitting to three points 根据三点共圆求曲率
float calculate_curvature(
  const TrajectoryPoint & p1, const TrajectoryPoint & p2,
  const TrajectoryPoint & p3)
{
  const float epsilon = std::numeric_limits<float>::epsilon();
  float den = std::max(
    distance2d(p1, p2) * distance2d(
      p2,
      p3) * distance2d(
      p3, p1), epsilon);
  //求曲率，两条相邻边叉乘除以三条边的乘积，再乘以2，就得曲率
  const float curvature =
    2.0F *
    static_cast<float>(
    (p2.pose.position.x - p1.pose.position.x) * (p3.pose.position.y - p1.pose.position.y) -
    (p2.pose.position.y - p1.pose.position.y) * (p3.pose.position.x - p1.pose.position.x)) /
    den;
  return curvature;
}
//lanelet::ConstLanelets是一个向量，lanelet::InvalId等于0。
size_t get_closest_lanelet(const lanelet::ConstLanelets & lanelets, const TrajectoryPoint & point)
{
  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto & llt = lanelets.at(i);
    const auto & point2d =
      lanelet::Point2d(lanelet::InvalId, point.pose.position.x, point.pose.position.y)
      .basicPoint2d();
    // TODO(mitsudome-r): change this implementation to remove dependency to boost
    const double distance = lanelet::geometry::distanceToCenterline2d(llt, point2d);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}


LanePlanner::LanePlanner(std::string name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "开启.");
  auto qos_transient_local = rclcpp::QoS{1}.transient_local();
  vector_map_subscriber_ = create_subscription<HADMapBin>(
    "/map/vector_map", qos_transient_local, std::bind(&LanePlanner::onMap, this, _1));
  
  route_subscriber_ = create_subscription<LaneletRoute>(
    "/mission_planner/output/route", qos_transient_local, std::bind(&LanePlanner::onRoute, this, _1));
  trajectory_pub_ = create_publisher<Trajectory>("/planning/trajectory", qos_transient_local);
  debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos_transient_local);
  
}
PoseArray LanePlanner::trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

void LanePlanner::onRoute(LaneletRoute::ConstSharedPtr route_msg){
  Trajectory trajectory_ = plan_trajectory(route_msg, lanelet_map_);
  trajectory_pub_->publish(trajectory_);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));


}

void LanePlanner::onMap(autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  // route_handler->setMap(*map_msg);

  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_);
  std::cout<<"map loaded"<<std::endl;
    
}



autoware_auto_planning_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(
  const lanelet::ConstPoint3d & pt,
  const float velocity)
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
  trajectory_point.pose.position.x = pt.x();
  trajectory_point.pose.position.y = pt.y();
  trajectory_point.longitudinal_velocity_mps = velocity;
  return trajectory_point;
}

lanelet::Point3d convertToLaneletPoint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return lanelet::Point3d(lanelet::InvalId, pt.pose.position.x, pt.pose.position.y, 0.0);
}

Trajectory LanePlanner::plan_trajectory(
  LaneletRoute::ConstSharedPtr had_map_route,
  const lanelet::LaneletMapConstPtr & map)
{
  // generate trajectory. Only x, y, and velocity is filled in at this point
  auto trajectory_points = generate_base_trajectory(had_map_route, map);

  // calculate missing fields in trajectory
  set_angle(&trajectory_points);

  set_steering_angle(&trajectory_points);

  set_time_from_start(&trajectory_points);

  auto trajectory = create_trajectory_message(had_map_route->header, trajectory_points);

  modify_velocity(&trajectory);

  return trajectory;
}

void LanePlanner::modify_velocity(Trajectory * trajectory)
{
  if (trajectory->points.empty()) {
    return;
  }

  // always set zero velocity at the end of trajectory for safety.
  auto & last_pt = trajectory->points.back();
  last_pt.longitudinal_velocity_mps = 0.0F;
  last_pt.lateral_velocity_mps = 0.0F;
  last_pt.acceleration_mps2 = 0.0F;
  // last_pt.heading_rate_rps = 0.0F;
  // m_trajectory_smoother.Filter(*trajectory);
}

TrajectoryPoints LanePlanner::generate_base_trajectory(
  LaneletRoute::ConstSharedPtr had_map_route,
  const LaneletMapConstPtr & map)
{
  using lanelet::utils::to2D;

  lanelet::ConstLanelets lanelets;
  for (const auto & segment : had_map_route->segments) {
    const auto & primitive = segment.primitives.front();
    try {
      const auto lane = map->laneletLayer.get(primitive.id);
      lanelets.push_back(lane);
    } catch (const lanelet::NoSuchPrimitiveError & ex) {
      // stop adding lanelets if lane cannot be found. e.g. goal is outside of queried submap
      break;
    }
  }

  // return empty trajectory if there are no lanes
  if (lanelets.empty()) {
    return TrajectoryPoints();
  }

  TrajectoryPoint trajectory_start_point;
  trajectory_start_point.pose = had_map_route->start_pose;

  TrajectoryPoint trajectory_goal_point;
  trajectory_goal_point.pose = had_map_route->goal_pose;

  const auto start_index = get_closest_lanelet(lanelets, trajectory_start_point);

  TrajectoryPoints trajectory_points;

  // using Germany Location since it is default location for Lanelet2
  // TODO(mitsudome-r): create define default location for Autoware
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr =
    lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,
    lanelet::Participants::Vehicle);

  // set position and velocity
  // trajectory_points.push_back(trajectory_start_point);
  for (size_t i = start_index; i < lanelets.size(); i++) {
    const auto & lanelet = lanelets.at(i);
    const auto & centerline = autoware::common::had_map_utils::generateFineCenterline(
      lanelet,
      1.0);//轨迹分辨率
    const auto speed_limit = static_cast<float>(10.0);//修改限速
      //static_cast<float>(traffic_rules_ptr->speedLimit(lanelet).speedLimit.value());

    double start_length = 0;
    if (i == start_index) {
      const auto start_point = convertToLaneletPoint(trajectory_start_point);
      start_length =
        lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(start_point)).length;//起始车道的中心线的起点到该车道中心线中距离给定起点最近的点的距离
    }

    double end_length = std::numeric_limits<float>::max();
    if (i == lanelets.size() - 1) {
      const auto goal_point = convertToLaneletPoint(trajectory_goal_point);
      end_length = lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(goal_point)).length;
    }

    double accumulated_length = 0;
    // skip first point to avoid inserting overlaps
    for (size_t j = 1; j < centerline.size(); j++) {
      const auto llt_prev_pt = centerline[j - 1];
      const auto llt_pt = centerline[j];
      accumulated_length += lanelet::geometry::distance2d(to2D(llt_prev_pt), to2D(llt_pt));
      if (accumulated_length < start_length) {continue;}
      if (accumulated_length > end_length) {break;}
      trajectory_points.push_back(convertToTrajectoryPoint(llt_pt, 1.5));//speed_limit
    }
  }
  return trajectory_points;
}

void LanePlanner::set_angle(TrajectoryPoints * trajectory_points)
{
  for (size_t i = 0; i < trajectory_points->size(); i++) {
    double angle = 0;
    auto & pt = trajectory_points->at(i);
    if (i + 1 < trajectory_points->size()) {
      size_t speeed_decrease_index = std::max(trajectory_points->size()/2, trajectory_points->size()-5);
      if (i > speeed_decrease_index){
          pt.longitudinal_velocity_mps = pt.longitudinal_velocity_mps - static_cast<float>((i - speeed_decrease_index)) * pt.longitudinal_velocity_mps/static_cast<float>((trajectory_points->size()-(speeed_decrease_index + 1)));
      }

      const auto & next_pt = trajectory_points->at(i + 1);
      angle = std::atan2(
        next_pt.pose.position.y - pt.pose.position.y,
        next_pt.pose.position.x - pt.pose.position.x);
    } else if (i != 0) {
      const auto & prev_pt = trajectory_points->at(i - 1);
      angle = std::atan2(
        pt.pose.position.y - prev_pt.pose.position.y,
        pt.pose.position.x - prev_pt.pose.position.x);
        pt.longitudinal_velocity_mps = 0.0;
    }
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, angle);
    
    pt.pose.orientation = tf2::toMsg(quat);
  }
}

void LanePlanner::set_steering_angle(TrajectoryPoints * trajectory_points)
{
  if (trajectory_points->empty()) {
    return;
  }

  // set steering angle
  const float wheel_base = 3.0;
  for (size_t i = 1; i < trajectory_points->size() - 1; i++) {
    auto & pt = trajectory_points->at(i);
    const auto & prev_pt = trajectory_points->at(i - 1);
    const auto & next_pt = trajectory_points->at(i + 1);
    const auto curvature = calculate_curvature(prev_pt, pt, next_pt);
    pt.front_wheel_angle_rad = std::atan(wheel_base * curvature);
  }
}

void LanePlanner::set_time_from_start(TrajectoryPoints * trajectory_points)
{
  if (trajectory_points->empty()) {
    return;
  }

  // set time_from_start
  double accumulated_time = 0;
  trajectory_points->at(0).time_from_start.sec = 0;
  trajectory_points->at(0).time_from_start.nanosec = 0;
  for (size_t i = 1; i < trajectory_points->size(); i++) {
    auto & pt = trajectory_points->at(i);
    const auto & prev_pt = trajectory_points->at(i - 1);
    const double distance_x = pt.pose.position.x - prev_pt.pose.position.x;
    const double distance_y = pt.pose.position.y - prev_pt.pose.position.y;
    const double distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
    const double velocity = prev_pt.longitudinal_velocity_mps;
    accumulated_time += distance / std::max(velocity, 0.5);
    std::chrono::nanoseconds duration(static_cast<int64_t>(accumulated_time * 1e9));
    pt.time_from_start = time_utils::to_message(duration);
  }
}

Trajectory LanePlanner::create_trajectory_message(
  const std_msgs::msg::Header & header,
  const TrajectoryPoints & trajectory_points)
{
  Trajectory trajectory;
  trajectory.header = header;
  size_t trajectory_length =
    std::min(trajectory_points.size(), static_cast<size_t>(Trajectory::CAPACITY));
  // size_t trajectory_length = trajectory_points.size();
  trajectory.points.resize(trajectory_length);
  for (size_t i = 0; i < trajectory_length; i++) {
    trajectory.points.at(i) = trajectory_points.at(i);
  }
  return trajectory;
}

}  // namespace lane_planner

}  // namespace autoware

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<autoware::lane_planner::LanePlanner>("global_planner_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
