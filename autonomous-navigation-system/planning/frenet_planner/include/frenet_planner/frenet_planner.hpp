#ifndef _FRENET_PLANNER_HPP_
#define _FRENET_PLANNER_HPP_

#include <limits>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <queue>

#include "polynomials.hpp"
#include "frenet_frame.hpp"
#include "utils.hpp"
#include "path_checker/path_checker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <decision_maker_msgs/msg/scene.hpp>

namespace planning {

using std::placeholders::_1;

using geometry_msgs::msg::PoseArray;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedObject;

class FrenetPlanner : public rclcpp::Node
{
public:

  FrenetPlanner(std::string name);

private:
  // ros timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // ros subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr global_route_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr detected_objects_sub_;
  rclcpp::Subscription<decision_maker_msgs::msg::Scene>::SharedPtr scene_sub_;

  // ros publishers
  rclcpp::Publisher<Trajectory>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr generated_paths_marker_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;

  VehicleState current_state_; // x, y, yaw, speed
  std::vector<Point2D> global_route_;
  PredictedObjects detected_objects_;
  std::vector<std::vector<Point2D>> obstacles_;
  std::shared_ptr<decision_maker_msgs::msg::Scene> current_scene_{};

  Setting settings_;
  
  bool global_route_ready_ = false;
  bool current_state_ready_ = false;
  FrenetPath best_traj_;
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> candidate_trajs_;

  // instances
  PathChecker paths_checker_;
  Frenet frenet_;
 
  // function
  void initialize();

  void plan();

  std::vector<FrenetPath> generateFrenetPaths(const FrenetState& curr_state,
                            double left_bound, double right_bound, double target_speed);
  
  std::vector<FrenetPath> generateFrenetPaths(const FrenetState& curr_state, 
                            double s_goal, double target_speed);

  void calculateGlobalPaths(std::vector<FrenetPath>& frenet_traj_list);

  bool selectBestPath(std::vector<std::vector<Point2D>> &obstacles);

  std::vector<Point2D> convertObstacleToPoints(const PredictedObject &msg);

  std::vector<Point2D> convertFromTrajectory(const Trajectory &msg);

  PoseArray trajectory2PoseArray(const Trajectory & trajectory);

  double LatOffsetCost(const FrenetPath& lat_trajectory) const;

  void modifyStartState(FrenetState& start_state);

  // callback functions
  void currentStateCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  void detectedObjectsCallback(const PredictedObjects::ConstSharedPtr objects_msg);

  void globalRouteCallback(const Trajectory::ConstSharedPtr route_msg);

  void timer_callback();

  void sceneCallback(decision_maker_msgs::msg::Scene::SharedPtr msg);

  // publish functions
  void publishTrajectory(const FrenetPath& selected_path);

  void publishStopTrajectory(const FrenetState& start_state);

  void publishEmergencyTrajectory();

  void publishGenratedPaths(const std::vector<FrenetPath>& paths);

  

};

}

#endif // _FRENET_PLANNER_HPP_
