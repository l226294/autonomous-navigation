#include "show_vehicle_state/show_vehicle_state.hpp"

ShowVehicleState::ShowVehicleState(std::string name) : Node(name)
{
  ego_vehicle_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/ego_vehicle_marker", rclcpp::QoS(1));
  scene_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/scene_marker", rclcpp::QoS(1));
  odm_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/carla/ego_vehicle/odometry", rclcpp::QoS(1), 
    std::bind(&ShowVehicleState::onOdom, this, std::placeholders::_1));
  scene_sub_ = create_subscription<decision_maker_msgs::msg::Scene>(
    "/decision_maker/scene", rclcpp::QoS{1},
    std::bind(&ShowVehicleState::onScene, this, _1));
}

void ShowVehicleState::onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray markers;
  m_position = msg->pose.pose.position;
  pose_init_ = true;

  // Get marker for velocity text
  geometry_msgs::msg::Point vel_vis_position;
  vel_vis_position.x = msg->pose.pose.position.x + 1.0;
  vel_vis_position.y = msg->pose.pose.position.y + 1.0;
  vel_vis_position.z = 0.0;

  std_msgs::msg::ColorRGBA color_rgba;
  color_rgba.g = 1.0;
  color_rgba.a = 1.0;
  auto velocity_text_marker = get_velocity_text_marker_ptr(
    msg->twist.twist, vel_vis_position,
    color_rgba);
  if (velocity_text_marker) {
    velocity_text_marker->id = 0;
    markers.markers.push_back(*velocity_text_marker);
    ego_vehicle_marker_pub_->publish(markers);
  }    
}

void ShowVehicleState::onScene(decision_maker_msgs::msg::Scene::SharedPtr msg)
{
  if(!pose_init_){
      return;
  }
  visualization_msgs::msg::MarkerArray markers;
  geometry_msgs::msg::Point scene_vis_position;
  scene_vis_position.x = m_position.x-3.0;
  scene_vis_position.y = m_position.y-3.0;
  auto marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
  marker_ptr->header.frame_id = "map";
  marker_ptr->header.stamp = this->now();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("scene");
  marker_ptr->scale.x = 2.0;
  marker_ptr->scale.z = 2.0;
  marker_ptr->text = msg->scene;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = scene_vis_position;      
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  std_msgs::msg::ColorRGBA color_rgba;
  color_rgba.g = 1.0;
  color_rgba.a = 1.0;
  marker_ptr->color = color_rgba;
  marker_ptr->id = 1;
  markers.markers.push_back(*marker_ptr);
  scene_marker_pub_->publish(markers);
}

visualization_msgs::msg::Marker::SharedPtr ShowVehicleState::get_velocity_text_marker_ptr(
  const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<visualization_msgs::msg::Marker>();
  marker_ptr->header.frame_id = "map";
  marker_ptr->header.stamp = this->now();

  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("velocity");
  marker_ptr->scale.x = 2.0;
  marker_ptr->scale.z = 2.0;

  double vel = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
  marker_ptr->text = std::to_string(static_cast<int>(vel * 3.6)) + std::string("[km/h]");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = vis_pos;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}