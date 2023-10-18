#ifndef _SHOW_VEHICLE_STATE_HPP_
#define _SHOW_VEHICLE_STATE_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <decision_maker_msgs/msg/scene.hpp>

using std::placeholders::_1;

class ShowVehicleState : public rclcpp::Node
{
public:
  ShowVehicleState(std::string name);
private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odm_sub_;
  rclcpp::Subscription<decision_maker_msgs::msg::Scene>::SharedPtr scene_sub_;
  
  geometry_msgs::msg::Point m_position;
  bool pose_init_ = false;

  void onOdom(nav_msgs::msg::Odometry::SharedPtr msg);
  void onScene(decision_maker_msgs::msg::Scene::SharedPtr msg);
  visualization_msgs::msg::Marker::SharedPtr get_velocity_text_marker_ptr(
      const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
      const std_msgs::msg::ColorRGBA & color_rgba);
};
#endif // _SHOW_VEHICLE_STATE_HPP_

