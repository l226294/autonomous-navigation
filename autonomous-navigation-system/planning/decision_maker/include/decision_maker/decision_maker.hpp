#ifndef _DECISION_MAKER_HPP_
#define _DECISION_MAKER_HPP_

#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "decision_maker/context.hpp"
#include "decision_maker/state.hpp"
#include <decision_maker_msgs/msg/scene.hpp>

#include "frenet_frame.hpp"
#include "utils.hpp"

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include "tf2/utils.h"

// namespace planning{
using namespace planning;

using std::placeholders::_1;

using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_perception_msgs::msg::PredictedObject;

class DecisionMakerNode : public rclcpp::Node
{
public:
  DecisionMakerNode(Context* context);
private:
  // ros timer
  rclcpp::TimerBase::SharedPtr timer_;

  // ros subscribers
  rclcpp::Subscription<Trajectory>::SharedPtr global_route_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr lanelet_bin_map_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr predicted_objects_sub_;

  // ros publishers
  rclcpp::Publisher<decision_maker_msgs::msg::Scene>::SharedPtr scene_pub_;

  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::ConstLanelets road_lanelets_;

  Context* context;
  VehicleState current_state_;
  Frenet frenet_;
  std::vector<ObstacleState> obstacles_in_frenet_;
  ObstacleState target_obstacle_;
  double left_bound_ = 0.0;
  double right_bound_ = 0.0;
  double lane_width_ = 3.5;
  double distance_to_goal_thresh_ = 1.0;
  PredictedObjects m_dynamic_objects_;
  lanelet::routing::RoutingGraphConstPtr vehicle_graph_;
  lanelet::Lanelet m_current_lanelet;
  bool is_map_ready = false;
  bool is_global_route_ready = false;
  std::vector<Point2D> global_route_;
  double max_lateral_offset_ = 5.0;

public:
  // function
  bool isApproachingGoal();
  double getCurrentLateralOffset();
  void ObstacleToFrenet();
  std::string determineCurrentScene();

private:
  // function
  std::vector<Point2D> convertFromTrajectory(const Trajectory &msg);

  // callback functions
  void onTimer();
  void onOdometry(nav_msgs::msg::Odometry::SharedPtr current_pose_msg);
  void globalRouteCallback(const Trajectory::ConstSharedPtr route_msg);
  void onLaneletMapBin(autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg);
  void onPredictedObjects(PredictedObjects::ConstSharedPtr objects_msg);
};

// }


#endif // _DECISION_MAKER_HPP_
  