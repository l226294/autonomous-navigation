#include "frenet_planner/frenet_planner.hpp"

namespace planning {

FrenetPlanner::FrenetPlanner(std::string name) : Node(name) 
{
  initialize();
}

void FrenetPlanner::initialize() {
  //Param
  settings_.global_frame = static_cast<std::string>(declare_parameter("global_frame", "map"));

  //Enable or disable decision maker 
  settings_.decision_maker = static_cast<bool>(declare_parameter("decision_maker", true));
  
  //Constraints
  settings_.inner_circle_offsets = static_cast<std::vector<double>>(declare_parameter("inner_circle_offsets").get<std::vector<double>>());
  settings_.inner_circle_radius = static_cast<std::vector<double>>(declare_parameter("inner_circle_radius").get<std::vector<double>>());
  settings_.outer_circle_offsets = static_cast<std::vector<double>>(declare_parameter("outer_circle_offsets").get<std::vector<double>>());
  settings_.outer_circle_radius = static_cast<std::vector<double>>(declare_parameter("outer_circle_radius").get<std::vector<double>>());
  settings_.max_speed = static_cast<double>(declare_parameter("max_speed", 14.0));
  settings_.max_accel = static_cast<double>(declare_parameter("max_accel", 2.0));
  settings_.max_decel = static_cast<double>(declare_parameter("max_decel", -2.0));
  settings_.max_curvature = static_cast<double>(declare_parameter("max_curvature", 1.0));

  settings_.tick_t = static_cast<double>(declare_parameter("tick_t", 0.2));

  settings_.vehicle_width = static_cast<double>(declare_parameter("vehicle_width", 2.0));
  settings_.lane_width = static_cast<double>(declare_parameter("lane_width", 3.5));
  
  //Param for generate FrenetPaths
  settings_.max_road_width = static_cast<double>(declare_parameter("max_road_width", 7.0));
  settings_.num_width = static_cast<int>(declare_parameter("num_width", 7));

  settings_.max_t = static_cast<double>(declare_parameter("max_t", 5.0));
  settings_.min_t = static_cast<double>(declare_parameter("min_t", 3.0));
  settings_.num_t = static_cast<int>(declare_parameter("num_t", 5));

  settings_.highest_speed = static_cast<double>(declare_parameter("highest_speed", 9.0));
  settings_.lowest_speed = static_cast<double>(declare_parameter("lowest_speed", 7.0));
  settings_.num_speed = static_cast<int>(declare_parameter("num_speed", 3));

  settings_.max_jerk_s = static_cast<double>(declare_parameter("max_jerk_s", 1.0));
  settings_.max_jerk_d = static_cast<double>(declare_parameter("max_jerk_d", 1.0));
 
  // cost weights
  settings_.k_time = static_cast<double>(declare_parameter("k_time", 1.0));
  settings_.k_lat_d = static_cast<double>(declare_parameter("k_lat_d", 1.0));
  settings_.k_lat_jerk = static_cast<double>(declare_parameter("k_lat_jerk", 1.0));
  settings_.k_lon_speed = static_cast<double>(declare_parameter("k_lon_speed", 1.0));
  settings_.k_lon_jerk = static_cast<double>(declare_parameter("k_lon_jerk", 1.0));
  settings_.k_lat = static_cast<double>(declare_parameter("k_lat", 1.0));
  settings_.k_lon = static_cast<double>(declare_parameter("k_lon", 1.0));
  
  // ros timer
  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&FrenetPlanner::timer_callback, this));

  // ros subscribers
  current_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/odometry", rclcpp::QoS(1),
      std::bind(&FrenetPlanner::currentStateCallback, this, _1));
  detected_objects_sub_ = create_subscription<PredictedObjects>(
      "/detected_objects", rclcpp::QoS{1}, std::bind(&FrenetPlanner::detectedObjectsCallback, this, _1));
  global_route_sub_ = create_subscription<Trajectory>(
      "/global_route", rclcpp::QoS{1}.transient_local(),
      std::bind(&FrenetPlanner::globalRouteCallback, this, _1));
  scene_sub_ = create_subscription<decision_maker_msgs::msg::Scene>(
      "/decision_maker/scene", rclcpp::QoS{1},
      std::bind(&FrenetPlanner::sceneCallback, this, _1));

  // ros publishers
  selected_path_pub_ = create_publisher<Trajectory>("/selected_path", rclcpp::QoS(1));
  generated_paths_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/generated_paths_marker", rclcpp::QoS(1));
  debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", rclcpp::QoS(1));
  
  // initialize instances
  paths_checker_ = PathChecker(settings_);

  if(!settings_.decision_maker){
    current_scene_ = std::make_unique<decision_maker_msgs::msg::Scene>();
    current_scene_->scene = "OverTake";
    current_scene_->left_bound = settings_.lane_width;
    current_scene_->right_bound = -settings_.lane_width;
  }

  RCLCPP_INFO(this->get_logger(),"Initialization complete.");

}

void FrenetPlanner::plan()
{
  
  // Clear the canidate trajectories
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> empty;
  std::swap(candidate_trajs_, empty);

  auto start_time = std::chrono::high_resolution_clock::now();  
  
  FrenetState start_state; 
  
  if(current_state_.v < settings_.lowest_speed){
    current_state_.v = settings_.lowest_speed;
  }else if(current_state_.v > settings_.highest_speed){
    current_state_.v = settings_.highest_speed;
  }

  if(!frenet_.ToFrenet(current_state_, start_state)){
    return;
  };

  if(current_scene_->scene == "Error"){
    RCLCPP_INFO(this->get_logger(),"emergency braking");
    publishEmergencyTrajectory();
    return;
  }else if(current_scene_->scene == "Complete"){
    RCLCPP_INFO(this->get_logger(),"mission complete");
    publishStopTrajectory(start_state);
    return;
  }

  modifyStartState(start_state);

  if(!settings_.decision_maker){
    if(std::abs(frenet_.path_s_m.back() - start_state.s) 
      < current_state_.v * current_state_.v / (2 * std::abs(settings_.max_decel))){
      publishStopTrajectory(start_state);
      return;
    }
  }
 
  // Sample a list of FrenetPaths
  double cruise_speed = (settings_.lowest_speed + settings_.highest_speed) / 2;
  
  std::vector<FrenetPath> all_trajs;
  if(current_scene_->scene != "CarFollow"){
    all_trajs = generateFrenetPaths(start_state, current_scene_->left_bound, current_scene_->right_bound, cruise_speed);
  }else{
    double s_goal = std::min(current_scene_->distance_to_follow_vehicle, current_scene_->target_speed * settings_.min_t);
    s_goal = start_state.s + s_goal;
    all_trajs = generateFrenetPaths(start_state, s_goal, current_scene_->target_speed);
  }

  // Convert to global paths
  calculateGlobalPaths(all_trajs);

  selectBestPath(obstacles_);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  
  RCLCPP_INFO(this->get_logger(),"Elapsed time: %.4f milliseconds.", duration.count() / 1000.0);
  
  // Publish the  Trajectory
  publishTrajectory(best_traj_);

  // Publish generated paths for visualization
  publishGenratedPaths(all_trajs);
  
}


void FrenetPlanner::modifyStartState(FrenetState& start_state)
{
  if(current_scene_->scene != "OverTake"){
    if(start_state.d > 0.5){
      start_state.d_d = -0.5;
    }else{
      start_state.d_d = -start_state.d;
    }
  }else if(current_scene_->scene == "OverTake"){
    if(current_scene_->left_bound > 1.0){
      if(start_state.d < 0){
        start_state.d_d = -start_state.d;
      }else if(start_state.d > 3.5){
        start_state.d_d = -(start_state.d - 3.5);
      }
    }else if(current_scene_->right_bound < -1.0){
      if(start_state.d < -3.5){
        start_state.d_d = -(start_state.d + 3.5);
      }else if(start_state.d > 0){
        start_state.d_d = -start_state.d;
      }
    }
    
  }
}

// Sample a list of FrenetPaths
std::vector<FrenetPath> 
FrenetPlanner::generateFrenetPaths(const FrenetState& curr_state,
        double left_bound, double right_bound, double target_speed)
{
  // list of frenet paths generated
  std::vector<FrenetPath> frenet_trajs;

  int path_index = 0;
 
  double delta_width = 0;
  // generate different goals with a lateral offset
  if(settings_.num_width == 1 || std::abs(right_bound - left_bound)<0.1){
    right_bound = 0;
    left_bound = 0;
    delta_width = settings_.lane_width;
  }else{
    delta_width = (left_bound - right_bound)/(settings_.num_width - 1);
  }
  
  for (double d = right_bound; d < left_bound + delta_width / 2 ; d += delta_width)  // left being positive
  {
    // generate d_t polynomials
    const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
    for (double T = settings_.min_t; T < settings_.max_t + delta_t / 2; T += delta_t)
    {
      // calculate time cost (encourage longer planning horizon)
      const double time_cost = settings_.k_time*(1 - (T - settings_.min_t)/(settings_.max_t - settings_.min_t));

      FrenetPath frenet_traj = FrenetPath();

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.emplace_back(curr_state.d);
      start_d.emplace_back(curr_state.d_d);
      start_d.emplace_back(curr_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.emplace_back(d);
      end_d.emplace_back(0.0);
      end_d.emplace_back(0.0);

      // generate lateral quintic polynomial
      QuinticPolynomial lateral_quintic_poly = QuinticPolynomial(start_d, end_d, T);
      
      // calculate lat jerk cost
      double lat_jerk_cost = 0.0;
      double lat_jerk_abs = 0.0;

      // store the this lateral trajectory into frenet_traj
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_traj.t.emplace_back(t);
        frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
        frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
        lat_jerk_cost += std::pow(frenet_traj.d_ddd.back(), 2);
        lat_jerk_abs += std::abs(frenet_traj.d_ddd.back());
      }

      // generate longitudinal quartic polynomial
      const double delta_speed = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed - 1 );
      for (double sample_speed = settings_.lowest_speed; sample_speed < settings_.highest_speed + delta_speed / 2; sample_speed += delta_speed)
      {

        // copy the longitudinal path over
        FrenetPath target_frenet_traj = frenet_traj;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.emplace_back(curr_state.s);
        start_s.emplace_back(curr_state.s_d);
        start_s.emplace_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.emplace_back(sample_speed);
        end_s.emplace_back(0.0);

        // generate longitudinal quartic polynomial
        QuarticPolynomial longitudinal_quartic_poly = QuarticPolynomial(start_s, end_s, T);

        // calculate lat jerk cost
        double lon_jerk_cost = 0.0;
        double lon_jerk_abs = 0.0;

        // store the this longitudinal trajectory into target_frenet_traj
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          double s = longitudinal_quartic_poly.calculatePoint(t);
          if(s > frenet_.path_s_m.back()){
            continue;
          }
          target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
          lon_jerk_cost += std::pow(target_frenet_traj.s_ddd.back(), 2);
          lon_jerk_abs += std::abs(target_frenet_traj.s_ddd.back());
        }

        // calculate speed cost
        const double speed_cost = settings_.k_lon_speed * std::pow((target_speed - target_frenet_traj.s_d.back())/settings_.highest_speed, 2);        
        
        if(lat_jerk_abs < 0.00001){
          lat_jerk_abs = 1;
        }
        if(lon_jerk_abs < 0.00001){
          lon_jerk_abs = 1;
        }
        // calculate lateral offset cost
        const double lat_d_cost = LatOffsetCost(target_frenet_traj);

        // calculate jerk costs
        lat_jerk_cost = settings_.k_lat_jerk * ((lat_jerk_cost / std::pow(settings_.max_jerk_d, 2)) / (lat_jerk_abs / settings_.max_jerk_d));
        lon_jerk_cost = settings_.k_lon_jerk * ((lon_jerk_cost / std::pow(settings_.max_jerk_s, 2)) / (lon_jerk_abs / settings_.max_jerk_s));
        // total cost 
        target_frenet_traj.final_cost = settings_.k_lat * (lat_d_cost + lat_jerk_cost)
                                    + settings_.k_lon * (speed_cost + time_cost + lon_jerk_cost);
        target_frenet_traj.traj_index = path_index;
        
        // RCLCPP_INFO(this->get_logger(),"path index: %d , final_cost: %.4f, lat_d_cost: %.4f, speed_cost: %.4f, time_cost: %.4f, "
        //   "lat_jerk_cost: %.4f, lon_jerk_cost: %.4f", path_index, target_frenet_traj.final_cost, lat_d_cost, speed_cost, time_cost
        //   ,lat_jerk_cost, lon_jerk_cost);
        frenet_trajs.emplace_back(target_frenet_traj);
        path_index++;

      }
    }
  }

  return frenet_trajs;
}

// Sample a list of FrenetPaths for carfollow 
std::vector<FrenetPath> 
FrenetPlanner::generateFrenetPaths(const FrenetState& curr_state,
        double s_goal, double target_speed)
{
  // list of frenet paths generated
  std::vector<FrenetPath> frenet_trajs;

  int path_index = 0;

  const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);

  for (double T = settings_.min_t; T < settings_.max_t + delta_t / 2; T += delta_t)
  {

    // calculate time cost (encourage longer planning horizon)
    const double time_cost = settings_.k_time*(1 - (T - settings_.min_t)/(settings_.max_t - settings_.min_t));

    FrenetPath frenet_traj = FrenetPath();

    // start lateral state [d, d_d, d_dd]
    std::vector<double> start_d;
    start_d.emplace_back(curr_state.d);
    start_d.emplace_back(curr_state.d_d);
    start_d.emplace_back(curr_state.d_dd);

    // end lateral state [d, d_d, d_dd]
    std::vector<double> end_d;
    end_d.emplace_back(0.0);
    end_d.emplace_back(0.0);
    end_d.emplace_back(0.0);

    QuinticPolynomial lateral_quintic_poly = QuinticPolynomial(start_d, end_d, T);
    
    // calculate lat jerk cost
    double lat_jerk_cost = 0.0;
    double lat_jerk_abs = 0.0;

    // store the this lateral trajectory into frenet_traj
    for (double t = 0.0; t <= T; t += settings_.tick_t)
    {
      frenet_traj.t.emplace_back(t);
      frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
      frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
      frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
      frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
      lat_jerk_cost += std::pow(frenet_traj.d_ddd.back(), 2);
      lat_jerk_abs += std::abs(frenet_traj.d_ddd.back());
    }

    int m_num_speed = 1;
    for (double sample_speed = target_speed - m_num_speed + 1 ; m_num_speed > 0; m_num_speed--)
    {

      // copy the longitudinal path over
      FrenetPath target_frenet_traj = frenet_traj;

      // start longitudinal state [s, s_d, s_dd]
      std::vector<double> start_s;
      start_s.emplace_back(curr_state.s);
      start_s.emplace_back(curr_state.s_d);
      start_s.emplace_back(0.0);

      // end longitudinal state [s_d, s_dd]
      std::vector<double> end_s;
      end_s.emplace_back(s_goal);
      end_s.emplace_back(sample_speed);
      end_s.emplace_back(0.0);

      QuinticPolynomial longitudinal_quartic_poly = QuinticPolynomial(start_s, end_s, T);

      // calculate lat jerk cost
      double lon_jerk_cost = 0.0;
      double lon_jerk_abs = 0.0;

      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        double s = longitudinal_quartic_poly.calculatePoint(t);
        if(s > frenet_.path_s_m.back()){
          continue;
        }
        target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
        target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
        target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
        target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        lon_jerk_cost += std::pow(target_frenet_traj.s_ddd.back(), 2);
        lon_jerk_abs += std::abs(target_frenet_traj.s_ddd.back());
      }

      // calculate speed cost
      const double speed_cost = settings_.k_lon_speed * std::pow((target_speed - target_frenet_traj.s_d.back())/settings_.highest_speed, 2);        
      
      if(lat_jerk_abs < 0.00001){
        lat_jerk_abs = 1;
      }
      if(lon_jerk_abs < 0.00001){
        lon_jerk_abs = 1;
      }

      // calculate lateral offset cost
      const double lat_d_cost = LatOffsetCost(target_frenet_traj);

      // calculate jerk costs
      lat_jerk_cost = settings_.k_lat_jerk * ((lat_jerk_cost / std::pow(settings_.max_jerk_d, 2)) / (lat_jerk_abs / settings_.max_jerk_d));
      lon_jerk_cost = settings_.k_lon_jerk * ((lon_jerk_cost / std::pow(settings_.max_jerk_s, 2)) / (lon_jerk_abs / settings_.max_jerk_s));
      // total cost 
      target_frenet_traj.final_cost = settings_.k_lat * (lat_d_cost + lat_jerk_cost)
                                  + settings_.k_lon * (speed_cost + time_cost + lon_jerk_cost);
      target_frenet_traj.traj_index = path_index;
      
      // RCLCPP_INFO(this->get_logger(),"path index: %d , final_cost: %.4f, lat_d_cost: %.4f, speed_cost: %.4f, time_cost: %.4f, "
      //   "lat_jerk_cost: %.4f, lon_jerk_cost: %.4f", path_index, target_frenet_traj.final_cost, lat_d_cost, speed_cost, time_cost
      //   ,lat_jerk_cost, lon_jerk_cost);

      frenet_trajs.emplace_back(target_frenet_traj);
      path_index++;

    }
  }

  return frenet_trajs;
}

// Evaluator LatOffsetCost of trajectory 
double FrenetPlanner::LatOffsetCost(
    const FrenetPath& trajectory) const {

  double lat_offset_start = trajectory.d[0];
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (int i = 0; i < trajectory.s.size(); i++) {
 
    double lat_offset = trajectory.d[i];
    double cost = lat_offset / settings_.lane_width;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * 10.0;
      cost_abs_sum += std::fabs(cost) * 10.0;
    } else {
      cost_sqr_sum += cost * cost * 1.0;
      cost_abs_sum += std::fabs(cost) * 1.0;
    }
    
  }
  return cost_sqr_sum / (cost_abs_sum + 1e-5);
}


// Convert to global paths
void FrenetPlanner::calculateGlobalPaths(std::vector<FrenetPath>& frenet_traj_list)
{
  for (int i = 0; i < frenet_traj_list.size(); i++)
  {
    for (int j = 0; j < frenet_traj_list[i].s.size(); j++)
    {

      FrenetState p_sd;

      p_sd.s = frenet_traj_list[i].s[j];
      p_sd.d = frenet_traj_list[i].d[j];
      Point2D sample_point;
      frenet_.ToCartesian(p_sd, sample_point);
      const double frenet_x = sample_point.x;
      const double frenet_y = sample_point.y;
      if (!isLegal(frenet_x) || !isLegal(frenet_y))
      {
        RCLCPP_INFO(this->get_logger(),"not legal");
        break;
      }
      else
      {
        frenet_traj_list[i].x.emplace_back(frenet_x);
        frenet_traj_list[i].y.emplace_back(frenet_y);
      }
    }
    // calculate yaw and ds
    for (int j = 0; j < frenet_traj_list[i].x.size() - 1; j++)
    {
      const double dx = frenet_traj_list[i].x[j+1] - frenet_traj_list[i].x[j];
      const double dy = frenet_traj_list[i].y[j+1] - frenet_traj_list[i].y[j];
      frenet_traj_list[i].yaw.emplace_back(atan2(dy, dx));
      frenet_traj_list[i].ds.emplace_back(sqrt(dx * dx + dy * dy));
    }

    frenet_traj_list[i].yaw.emplace_back(frenet_traj_list[i].yaw.back());
    frenet_traj_list[i].ds.emplace_back(frenet_traj_list[i].ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_traj_list[i].yaw.size() - 1; j++)
    {
      double yaw_diff = unifyAngleRange(frenet_traj_list[i].yaw[j+1] - frenet_traj_list[i].yaw[j]);
      frenet_traj_list[i].c.emplace_back(yaw_diff / frenet_traj_list[i].ds[j]);
    }
    candidate_trajs_.push(frenet_traj_list[i]);
  }

}

// Selects the best path in the path set
bool FrenetPlanner::selectBestPath(std::vector<std::vector<Point2D>> &obstacles) {
  bool best_traj_found = false;
  while(!best_traj_found && !candidate_trajs_.empty())
  {
    best_traj_ = candidate_trajs_.top();
    candidate_trajs_.pop();
    if (!paths_checker_.checkConstraints(best_traj_, obstacles))
    {
      continue;
    }
    best_traj_found = true;
    
  }
  if(!best_traj_found){
    return false;
  }
  return true;
}



std::vector<Point2D> FrenetPlanner::convertObstacleToPoints(
    const PredictedObject &msg) 
{
  // RCLCPP_INFO(this->get_logger(),"Processing obstacles.");

  std::vector<Point2D> obstacle_pts(8, Point2D());

  double x = 
    msg.kinematics.initial_pose_with_covariance.pose.position.x;
  double y = 
    msg.kinematics.initial_pose_with_covariance.pose.position.y;
  double z = 
    msg.kinematics.initial_pose_with_covariance.pose.position.z;
  double yaw = 
    tf2::getYaw(msg.kinematics.initial_pose_with_covariance.pose.orientation);

  double xrad = msg.shape.dimensions.x / 2.0;
  double yrad = msg.shape.dimensions.y / 2.0;
  double zrad = 0.8;

  Eigen::MatrixXd cpos(8, 2), rotyaw(2, 2), cpos_shift(8, 2);

  cpos << -xrad, -yrad, -xrad, 0, -xrad, yrad, 0, yrad, xrad, yrad, xrad, 0,
      xrad, -yrad, 0, -yrad;

  rotyaw << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);

  cpos_shift << x, y, x, y, x, y, x, y, x, y, x, y, x, y, x, y;

  cpos = cpos * rotyaw + cpos_shift;

  for (int i = 0; i < cpos.rows(); i++) {
    obstacle_pts[i] = {cpos(i, 0), cpos(i, 1)};
  }

  return obstacle_pts;
}

std::vector<Point2D>
FrenetPlanner::convertFromTrajectory(const Trajectory & trajectory) 
{
  std::vector<Point2D> ref_route(0);
  for (int i = 0; i < trajectory.points.size(); i++) {
    double x = trajectory.points[i].pose.position.x;
    double y = trajectory.points[i].pose.position.y;
    ref_route.push_back(Point2D(x, y));    
  }
  return ref_route;
}

PoseArray FrenetPlanner::trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

// Callback Functions

void FrenetPlanner::timer_callback(){
  if(!current_state_ready_ || !global_route_ready_ || !current_scene_){
    return;
  }
  plan();
}

void FrenetPlanner::currentStateCallback(
    nav_msgs::msg::Odometry::SharedPtr msg) {
  current_state_.x = msg->pose.pose.position.x;
  current_state_.y = msg->pose.pose.position.y;
  current_state_.yaw = tf2::getYaw(msg->pose.pose.orientation);
  current_state_.v = msg->twist.twist.linear.x;
  current_state_ready_ = true;

}

void FrenetPlanner::detectedObjectsCallback(
    const PredictedObjects::ConstSharedPtr objects_msg) {
  detected_objects_ = *objects_msg;
  obstacles_.clear();
  for(const auto object : detected_objects_.objects){
    obstacles_.push_back(convertObstacleToPoints(object));
  }
}

void FrenetPlanner::globalRouteCallback(const Trajectory::ConstSharedPtr msg) {
  global_route_ = convertFromTrajectory(*msg);
  frenet_.SetPath(global_route_);
  global_route_ready_ = true;
}



void FrenetPlanner::sceneCallback(decision_maker_msgs::msg::Scene::SharedPtr msg){
  if(settings_.decision_maker){
    current_scene_ = msg;
  }
}

// Publish msgs

void FrenetPlanner::publishEmergencyTrajectory()
{
  Trajectory trajectory;
  trajectory.header.stamp = this->now();
  trajectory.header.frame_id = settings_.global_frame;
  TrajectoryPoint point;
  point.pose.position.x = current_state_.x;
  point.pose.position.y = current_state_.y;
  point.pose.position.z = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  trajectory.points.push_back(point);
  selected_path_pub_->publish(trajectory);
}

void FrenetPlanner::publishStopTrajectory(const FrenetState& start_state)
{
  int start_index;
  for(int i = frenet_.path_s_m.size() -1; i >=0; i--){
    if(frenet_.path_s_m[i] < start_state.s){
      start_index = i + 1;
      break;
    }
  }

  Trajectory trajectory;
  trajectory.header.frame_id = settings_.global_frame;
  trajectory.header.stamp = this->now();
  double angle = current_state_.yaw;

  for (int i = start_index; i < global_route_.size(); i++) {
    TrajectoryPoint point;
    point.pose.position.x = global_route_[i].x;
    point.pose.position.y = global_route_[i].y;
    point.pose.position.z = 0.0;
    if(i < global_route_.size()-1){
      angle = std::atan2(
        global_route_[i+1].y - global_route_[i].y,
        global_route_[i+1].x - global_route_[i].x);
    }     
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, angle);
    point.pose.orientation = tf2::toMsg(rotation);
    point.longitudinal_velocity_mps = 0.0;
    trajectory.points.push_back(point);
  }
  RCLCPP_INFO(this->get_logger(),"publish Stop Trajectory");
  selected_path_pub_->publish(trajectory);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory));
  global_route_ready_ = false;

}


// Publish the final path as an output to other velocity planners
void FrenetPlanner::publishTrajectory(const FrenetPath& selected_path) {

  Trajectory trajectory;
  trajectory.header.frame_id = settings_.global_frame;
  trajectory.header.stamp = this->now();
  
  for (int i = 0; i < selected_path.x.size(); i++) {
    TrajectoryPoint point;
    point.pose.position.x = selected_path.x[i];
    point.pose.position.y = selected_path.y[i];
    point.pose.position.z = 0.0;

    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, selected_path.yaw[i]);
    point.pose.orientation = tf2::toMsg(rotation);

    if(current_scene_->scene == "CarFollow"){
      point.longitudinal_velocity_mps = current_scene_->target_speed;
    }else{
      point.longitudinal_velocity_mps = selected_path.s_d[i];
    }
    trajectory.points.push_back(point);
  }
  selected_path_pub_->publish(trajectory);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory));
}


// Publish generated paths for visualization
void FrenetPlanner::publishGenratedPaths(
    const std::vector<FrenetPath>& paths) {

  visualization_msgs::msg::MarkerArray generate_paths_marker;
  for (int i = 0; i < paths.size(); i++) {

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = settings_.global_frame;
    line_strip.header.stamp = this->now();
    line_strip.ns = "generated_paths";
    line_strip.id = i;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    // line width
    line_strip.scale.x = 0.1;

    if (i == best_traj_.traj_index) {
      // line_strip.ns = "selected_path";
      // line strip is pink
      line_strip.color.r = 1.0;
      line_strip.color.g = 0.078;
      line_strip.color.b = 0.576;
      line_strip.color.a = 1.0;
    } else {
      // line strip is blue
      line_strip.color.b = 1.0;
      if (paths[i].collision_passed) {
        line_strip.color.a = 1.0;
      } else {
        line_strip.color.a = 0.1;//0.25
      }
    }

    for (int j = 0; j < paths[i].x.size(); j++) {
      
      geometry_msgs::msg::Point p;
      p.x = paths[i].x[j];
      p.y = paths[i].y[j];   
      p.z = 0.0;
      line_strip.points.push_back(p);
    }

    generate_paths_marker.markers.push_back(line_strip);
  }
  generated_paths_marker_pub_->publish(generate_paths_marker);
}

}
