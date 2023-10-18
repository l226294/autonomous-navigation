#ifndef _CONTROL_TRANSFER_HPP_
#define _CONTROL_TRANSFER_HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

using std::placeholders::_1;

class ControlTransfer : public rclcpp::Node
{
public:
  ControlTransfer(std::string name);
private:
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr control_pub_;
  void control_callback(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr control_msg);
};

#endif // _CONTROL_TRANSFER_HPP_