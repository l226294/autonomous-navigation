#include "earth_to_map_tf/earth_to_map_tf.hpp"

EarthToMapTfNode::EarthToMapTfNode(std::string name) : Node(name), 
  m_yaml_file_name(declare_parameter("map_yaml_file").get<std::string>())
{
  earth_to_map_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());
  map_to_gnssbl_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf",
    rclcpp::QoS(rclcpp::KeepLast(5U)));
  
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_pose", 1);
  heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "/heading", rclcpp::QoS(1),
    std::bind(&EarthToMapTfNode::HeadingCallback, this, std::placeholders::_1));
  nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix", rclcpp::QoS{1}, std::bind(&EarthToMapTfNode::NavSatFix2LocalCartesianWGS84, this, std::placeholders::_1));
  run();
}

void EarthToMapTfNode::run()
{
  geocentric_pose_t pose = load_map_origin(m_yaml_file_name);
  publish_earth_to_map_transform(pose);
}

geocentric_pose_t EarthToMapTfNode::load_map_origin(
  const std::string & yaml_file_name)
{
  geodetic_pose_t geodetic_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!yaml_file_name.empty()) {
    read_from_yaml(yaml_file_name, &geodetic_pose);
  } else {
    throw std::runtime_error("YAML file name empty\n");
  }
  double x(0.0), y(0.0), z(0.0);

  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Forward(
    geodetic_pose.latitude,
    geodetic_pose.longitude,
    geodetic_pose.elevation,
    x, y, z);

  return {x, y, z, geodetic_pose.roll, geodetic_pose.pitch, geodetic_pose.yaw};
}

void EarthToMapTfNode::read_from_yaml(
  const std::string & yaml_file_name,
  geodetic_pose_t * geo_pose)
{
  try {
    YAML::Node map_info = YAML::LoadFile(yaml_file_name);
    if (map_info["map_config"]) {
      if (map_info["map_config"]["latitude"] &&
        map_info["map_config"]["longitude"] &&
        map_info["map_config"]["elevation"])
      {
        geo_pose->latitude = map_info["map_config"]["latitude"].as<double>();
        geo_pose->longitude = map_info["map_config"]["longitude"].as<double>();
        geo_pose->elevation = map_info["map_config"]["elevation"].as<double>();
        nav_sat_fix_origin_.latitude = geo_pose->latitude;
        nav_sat_fix_origin_.longitude = geo_pose->longitude;
        nav_sat_fix_origin_.elevation = geo_pose->elevation;
        origin_ready_ = true;
      } else {
        throw std::runtime_error("Yaml file: map origin not found\n");
      }
      if (map_info["map_config"]["roll"]) {
        geo_pose->roll = map_info["map_config"]["roll"].as<double>();
      }
      if (map_info["map_config"]["pitch"]) {
        geo_pose->pitch = map_info["map_config"]["pitch"].as<double>();
      }
      if (map_info["map_config"]["yaw"]) {
        geo_pose->yaw = map_info["map_config"]["yaw"].as<double>();
      }
    } else {
      throw std::runtime_error("Yaml file: map config not found\n");
    }
  } catch (const YAML::BadFile & ex) {
    throw std::runtime_error("Yaml file not found\n");
  } catch (const YAML::ParserException & ex) {
    throw std::runtime_error("Yaml syntax error\n");
  }
}

void EarthToMapTfNode::publish_earth_to_map_transform(geocentric_pose_t pose)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.translation.z = pose.z;

  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);
  tf.header.stamp = this->now();
  tf.transform.rotation = tf2::toMsg(q);
  tf.header.frame_id = "earth";
  tf.child_frame_id = "map";

  tf2_msgs::msg::TFMessage static_tf_msg;
  static_tf_msg.transforms.push_back(tf);

  m_transform_pub_timer = create_wall_timer(
    std::chrono::milliseconds(100),
    [this, static_tf_msg]() {
      earth_to_map_pub_->publish(static_tf_msg);
    });

  earth_to_map_pub_->publish(static_tf_msg);

}

// callback functions

void EarthToMapTfNode::NavSatFix2LocalCartesianWGS84(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg)
{
  if(!origin_ready_){
    return;
  }
  double x(0.0), y(0.0), z(0.0);
  try {
    GeographicLib::LocalCartesian localCartesian_origin(
      nav_sat_fix_origin_.latitude, nav_sat_fix_origin_.longitude, nav_sat_fix_origin_.elevation);
    localCartesian_origin.Forward(
      nav_sat_fix_msg->latitude, nav_sat_fix_msg->longitude, 0.0,
      x, y, z);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    if(heading_init_){
      pose.pose.orientation = heading_;
      double yaw = tf2::getYaw(heading_); // rad
      pose_pub_->publish(pose);
      geometry_msgs::msg::TransformStamped tf;

      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.translation.z = z;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      tf.header.stamp = this->now();
      tf.transform.rotation = tf2::toMsg(q);
      tf.header.frame_id = "map";
      tf.child_frame_id = "gnss_base_link";

      tf2_msgs::msg::TFMessage tf_msg;
      tf_msg.transforms.push_back(tf);
      map_to_gnssbl_pub_->publish(tf_msg); 
      
    }
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to convert NavSatFix to LocalCartesian" << err.what());
    
  }
}

void EarthToMapTfNode::HeadingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) 
{
  heading_ = msg->quaternion;
  heading_init_ = true;
}
