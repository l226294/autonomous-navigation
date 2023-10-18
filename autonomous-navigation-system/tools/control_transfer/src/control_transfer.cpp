#include "control_transfer/control_transfer.hpp"

ControlTransfer::ControlTransfer(std::string name) : Node(name)
{
  control_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 1);
  control_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/vehicle/ackermann_vehicle_command", 1, 
    std::bind(&ControlTransfer::control_callback, this, std::placeholders::_1));
}

void ControlTransfer::control_callback(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr control_msg)
{
  ackermann_msgs::msg::AckermannDrive ackermann_msg;
  ackermann_msg.steering_angle = control_msg->lateral.steering_tire_angle;
  ackermann_msg.steering_angle_velocity = control_msg->lateral.steering_tire_rotation_rate;
  ackermann_msg.speed = control_msg->longitudinal.speed;
  ackermann_msg.acceleration = control_msg->longitudinal.acceleration;
  ackermann_msg.jerk = control_msg->longitudinal.jerk;
  control_pub_->publish(ackermann_msg);  
}