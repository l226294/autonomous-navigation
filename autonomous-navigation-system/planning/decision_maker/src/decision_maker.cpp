#include "decision_maker/decision_maker.hpp"

// namespace planning{

DecisionMakerNode::DecisionMakerNode(Context* context) :  Node("decision_maker"),context(context)
{
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  current_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/odometry", rclcpp::QoS(10),
      std::bind(&DecisionMakerNode::onOdometry, this, _1));
  global_route_sub_ = create_subscription<Trajectory>(
      "/planning/trajectory", durable_qos,
      std::bind(&DecisionMakerNode::globalRouteCallback, this, _1));
  lanelet_bin_map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "/map/vector_map", durable_qos,
      std::bind(&DecisionMakerNode::onLaneletMapBin, this, _1));
  predicted_objects_sub_ = create_subscription<PredictedObjects>(
      "/perception/object_recognition/objects", 1, std::bind(&DecisionMakerNode::onPredictedObjects, this, _1));
  
  scene_pub_ = create_publisher<decision_maker_msgs::msg::Scene>("~/scene", rclcpp::QoS{1});
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DecisionMakerNode::onTimer, this));

  context->SetDecisionMaker(this);

  // start state machine
  context->Start("Ready");

};


void DecisionMakerNode::ObstacleToFrenet(){
  FrenetState start_state;
  if(!frenet_.ToFrenet(current_state_, start_state)){
    return;
  };
  obstacles_in_frenet_.clear();
  for(const auto object : m_dynamic_objects_.objects){
    VehicleState obstacle_state;
    obstacle_state.x = 
      object.kinematics.initial_pose_with_covariance.pose.position.x;
    obstacle_state.y = 
      object.kinematics.initial_pose_with_covariance.pose.position.y;
    obstacle_state.v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    obstacle_state.yaw = tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);

    FrenetState obstacle_in_frenet;
    if(!frenet_.ToFrenet(obstacle_state, obstacle_in_frenet)){
      continue;
    }
    if(std::abs(obstacle_in_frenet.d) > max_lateral_offset_){
      continue;
    }
    ObstacleState object_state;
    object_state.x = obstacle_state.x;
    object_state.y = obstacle_state.y;
    object_state.yaw = obstacle_state.yaw;
    object_state.v = obstacle_state.v;
    object_state.s = obstacle_in_frenet.s;
    object_state.d = obstacle_in_frenet.d;
    object_state.distance_to_ego = obstacle_in_frenet.s - start_state.s;    
    obstacles_in_frenet_.push_back(object_state);
  }
}


std::string DecisionMakerNode::determineCurrentScene(){
  if(isApproachingGoal()){
    return "Complete";
  }
  double lon_distance_thresh = 50.0;
  double follow_lon_distance_thresh = 30.0;
  double follow_lat_distance_thresh = 1.0;
  double too_close_distance_thresh = 10.0;
  double min_lon_distance = 100.0;
  bool is_obstacle_forward = false;
  for(const auto& obstacle : obstacles_in_frenet_)
  {
    double distance = obstacle.distance_to_ego;
    if(distance > 0 && distance < lon_distance_thresh && std::abs(obstacle.d) < follow_lat_distance_thresh)
    {
      is_obstacle_forward = true;
      if(distance < min_lon_distance)
      {
        min_lon_distance = distance;
        target_obstacle_ = obstacle;
      }
      if(distance < too_close_distance_thresh)
      {
        return "Error";
      }
    }  
  }
  if(is_obstacle_forward)
  {
    if(target_obstacle_.v > current_state_.v)
    {
      return "LaneKeep";
    }else{
      bool left_changeable = true;
      bool right_changeable = true;
      const auto & left_lane = vehicle_graph_->adjacentLeft(m_current_lanelet);
      if(left_lane)
      {
        for(const auto& obstacle : obstacles_in_frenet_)
        {
          double distance = obstacle.distance_to_ego;
          if(obstacle.d > follow_lat_distance_thresh && obstacle.d < 5.0)
          {
            if(obstacle.v < current_state_.v)
            {
              if(distance > 5.0)
              {
                left_changeable = false;
                break;
              }
            }else{
              if(distance < 30.0)
              {
                left_changeable = false;
                break;
              }
            }
          }
        }
      }else{
        left_changeable = false;
      }

      if(left_changeable)
      {
        left_bound_ = lane_width_;
        right_bound_ = 0.0;
        return "OverTake";
      }else{
        const auto & right_lane = vehicle_graph_->adjacentRight(m_current_lanelet);
        if(right_lane)
        {
          for(const auto& obstacle : obstacles_in_frenet_)
          {
            double distance = obstacle.distance_to_ego;
            if(std::abs(obstacle.d) > follow_lat_distance_thresh && std::abs(obstacle.d) < 5.0)
            {
              if(obstacle.v < current_state_.v)
              {
                if(distance > 5.0)
                {
                  right_changeable = false;
                  break;
                }
              }else{
                if(distance < 30.0)
                {
                  right_changeable = false;
                  break;
                }
              } 
            }
          }
        }else{
          right_changeable = false;
        }
        if(right_changeable)
        {
          left_bound_ = 0.0;
          right_bound_ = -lane_width_;
          return "OverTake";
        }
      }
      if(!left_changeable && !right_changeable)
      {
        return "CarFollow";
      }
    }
  }else{
    return "LaneKeep";
  }
}

bool DecisionMakerNode::isApproachingGoal(){
  FrenetState start_state;
  if(frenet_.ToFrenet(current_state_, start_state)){
    double distance_to_goal = frenet_.path_s_m.back() - start_state.s;
    if(std::abs(distance_to_goal) < distance_to_goal_thresh_){
      return true;
    }
  }
  return false;
}

double DecisionMakerNode::getCurrentLateralOffset()
{
  FrenetState start_state;
  if(frenet_.ToFrenet(current_state_, start_state)){
    return start_state.d;
  }else{
    return 10000;
  }
}

std::vector<Point2D>
DecisionMakerNode::convertFromTrajectory(const Trajectory &trajectory) 
{
  std::vector<Point2D> ref_route(0);
  for (int i = 0; i < trajectory.points.size(); i++) {
    double x = trajectory.points[i].pose.position.x;
    double y = trajectory.points[i].pose.position.y;
    ref_route.push_back(Point2D(x, y));    
  }
  return ref_route;
}


// callback function

void DecisionMakerNode::onTimer()
{
  if(!is_map_ready || !is_global_route_ready || !context){
    return;
  }
  context->Update();
  std::string current_state = context->GetCurStateName();
  RCLCPP_INFO(get_logger(), "current_state: %s", current_state.c_str());
  decision_maker_msgs::msg::Scene scene;
  scene.header.stamp = this->now();
  scene.header.frame_id = "map";
  scene.scene = current_state;
  if(current_state == "CarFollow"){
    scene.target_speed = target_obstacle_.v;
    scene.distance_to_follow_vehicle = target_obstacle_.distance_to_ego;
  }else{
    scene.target_speed = current_state_.v;
  }
  
  if(current_state == "OverTake"){
    scene.left_bound = left_bound_;
    scene.right_bound = right_bound_;
  }else{
    scene.left_bound = 0.0;
    scene.right_bound = 0.0;
  }
  scene_pub_->publish(scene);
  
}

void DecisionMakerNode::onOdometry(nav_msgs::msg::Odometry::SharedPtr current_pose_msg)
{
  current_state_.x = current_pose_msg->pose.pose.position.x;
  current_state_.y = current_pose_msg->pose.pose.position.y;
  current_state_.yaw = tf2::getYaw(current_pose_msg->pose.pose.orientation);
  current_state_.v = current_pose_msg->twist.twist.linear.x;

  distance_to_goal_thresh_ = current_state_.v*current_state_.v/(2*3.5);
  
  if(is_map_ready){
    lanelet::Lanelet current_lanelet;
    lanelet::utils::query::getClosestLanelet(road_lanelets_, current_pose_msg->pose.pose, &current_lanelet);
    m_current_lanelet = current_lanelet;
  }
  
}

void DecisionMakerNode::onLaneletMapBin(autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle_graph_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules);
  
  std::cout<<"map loaded"<<std::endl;
  is_map_ready = true;
    
}

void DecisionMakerNode::onPredictedObjects(PredictedObjects::ConstSharedPtr objects_msg)
{
  m_dynamic_objects_ = * objects_msg;
}

void DecisionMakerNode::globalRouteCallback(const Trajectory::ConstSharedPtr route_msg)
{
  global_route_ = convertFromTrajectory(*route_msg);
  frenet_.SetPath(global_route_);
  std::cout<<"global route loaded"<<std::endl;
  is_global_route_ready = true;
}

// }


