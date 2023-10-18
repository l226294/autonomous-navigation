#ifndef _EARTH_TO_MAP_HPP_
#define _EARTH_TO_MAP_HPP_
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>
#include <GeographicLib/Geocentric.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

using std::placeholders::_1;

/// struct to hold geodetic pose of map origin
/// decribed as a latitude, longitude, and elevation
/// along with an orientation: roll, pitch and yaw
struct geodetic_pose_t
{
  double latitude;
  double longitude;
  double elevation;
  double roll;
  double pitch;
  double yaw;
};

/// struct to hold geocentric pose of map origin
/// decribed as a x, y, z
/// along with an orientation: roll, pitch and yaw
struct geocentric_pose_t
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class EarthToMapTfNode : public rclcpp::Node
{
public:
  EarthToMapTfNode(std::string name);
private:
  // ros timer
  rclcpp::TimerBase::SharedPtr m_transform_pub_timer{nullptr};

  // ros publisher
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr earth_to_map_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr map_to_gnssbl_pub_;  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ros subscriber
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;

  bool heading_init_ = false;
  geometry_msgs::msg::Quaternion heading_;
  const std::string m_yaml_file_name;
  geodetic_pose_t nav_sat_fix_origin_;
  bool origin_ready_ = false;

  // functions
  void run();
  geocentric_pose_t load_map_origin(const std::string & yaml_file_name);
  void read_from_yaml(const std::string & yaml_file_name, geodetic_pose_t * geo_pose);
  void publish_earth_to_map_transform(geocentric_pose_t pose);
  // callback functions
  void NavSatFix2LocalCartesianWGS84(const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg);
  void HeadingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
};

#endif // _EARTH_TO_MAP_HPP_