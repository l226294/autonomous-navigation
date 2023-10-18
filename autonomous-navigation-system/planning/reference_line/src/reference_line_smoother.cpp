#include "reference_line/reference_line_smoother.hpp"
#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

namespace planning {

ReferenceLineSmoother::ReferenceLineSmoother(std::string name) : Node(name)
{
  global_route_sub_ = create_subscription<Trajectory>(
      "/output_trajectory", rclcpp::QoS{1}.transient_local(),
      std::bind(&ReferenceLineSmoother::globalRouteCallback, this, _1));
  
  smoothed_trajectory_pub_ = this->create_publisher<Trajectory>("/smoothed_global_path", rclcpp::QoS{1}.transient_local());
  debug_pose_array_pub_ = this->create_publisher<PoseArray>("~/debug/pose_array", rclcpp::QoS(1).transient_local());
  test();
}

bool ReferenceLineSmoother::SmoothReferenceLine(const Trajectory &raw_points,
                                                Trajectory& smoothed_ref_points) 
{
  ref_points_ = raw_points;
  const int point_num = ref_points_.points.size();
  if (point_num < 3) {
    RCLCPP_INFO(this->get_logger(), "the ref points num is less 3");
    return false;
  }

  num_of_points_ = point_num;
  num_of_slack_variable_ = num_of_points_ - 2;
  num_of_variables_ = num_of_points_ * 2 + num_of_slack_variable_;
  num_curvature_constraint_ = num_of_points_ - 2;
  slack_variable_start_index_ = num_of_points_ * 2;
  slack_variable_end_index_ = slack_variable_start_index_ + num_of_slack_variable_;
  num_of_constraint_ = num_curvature_constraint_;
  curvature_constraint_start_index_ = 0;
  curvature_constraint_end_index_ = curvature_constraint_start_index_ + num_curvature_constraint_;
  
  std::vector<std::pair<double, double>> xy;
  for (const auto &ref_point : ref_points_.points) {
    xy.emplace_back(ref_point.pose.position.x, ref_point.pose.position.y);
  }
  ReferenceLineSmoothIpoptInterface smoother_interface(xy);
  smoother_interface.set_ref_deviation_weight(deviation_weight_);
  smoother_interface.set_heading_weight(heading_weight_);
  smoother_interface.set_length_weight(distance_weight_);
  smoother_interface.set_slack_weight(slack_weight_);
  SetUpOptions();
  SetUpInitValue();
  if (!this->SetUpConstraint()) {
    std::cout << "set up constraint error" << std::endl;
    return false;
  }
  CppAD::ipopt::solve_result<DVector> solution;
  CppAD::ipopt::solve<DVector, ReferenceLineSmoothIpoptInterface>(options_, xi_, x_l_, x_u_, g_l_, g_u_,
                                                                  smoother_interface, solution);
  if (solution.status != CppAD::ipopt::solve_result<DVector>::success) {
    printf("failed! reason: %i", solution.status);
    return false;
  }

  bool result = this->TraceSmoothReferenceLine(solution, smoothed_ref_points);
  return result;
}

bool ReferenceLineSmoother::SetUpConstraint() {

  x_l_.resize(num_of_variables_);
  x_u_.resize(num_of_variables_);
  g_l_.resize(num_of_constraint_);
  g_u_.resize(num_of_constraint_);
  double boundary_radius = 1.5;

  std::vector<double> boundary_bound(num_of_points_, 0.0);
  for (int i = 0; i < num_of_points_; ++i) {
    if (i == 0 || i == num_of_points_ - 1) {
      boundary_bound[i] = 0.4;
    } else {
      boundary_bound[i] = boundary_radius;
    }
  }
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;

    x_l_[index] = ref_points_.points[i].pose.position.x - boundary_bound[i];
    x_u_[index] = ref_points_.points[i].pose.position.x + boundary_bound[i];
    x_l_[index + 1] = ref_points_.points[i].pose.position.y - boundary_bound[i];
    x_u_[index + 1] = ref_points_.points[i].pose.position.y + boundary_bound[i];
  }

  // slack variable
  for (int i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    x_l_[i] = 0.0;
    x_u_[i] = 1e20;
  }

  // calculate curvature upper constraints
  double ref_line_total_length = 0.0;
  for (int i = 1; i < num_of_points_; ++i) {
    const auto last_ref_point = ref_points_.points[i - 1];
    const auto cur_ref_point = ref_points_.points[i];
    ref_line_total_length += hypot(cur_ref_point.pose.position.x - last_ref_point.pose.position.x, 
        cur_ref_point.pose.position.y - last_ref_point.pose.position.y);
  }

  double average_ds = ref_line_total_length / static_cast<double>(num_of_points_ - 1);
  double curvature_upper = average_ds * average_ds * max_curvature_;

  for (int i = curvature_constraint_start_index_; i < curvature_constraint_end_index_; ++i) {
    g_l_[i] = -1e20;
    g_u_[i] = curvature_upper * curvature_upper;
  }
  return true;
}

void ReferenceLineSmoother::SetUpOptions() {
  options_.clear();
  options_ += "Integer print_level  0\n";
  options_ += "Sparse  true        reverse\n";
  options_ += "Numeric tol          1e-5\n";
  options_ += "Integer max_iter    15\n";
}

void ReferenceLineSmoother::SetUpInitValue() {

  xi_.resize(num_of_variables_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    xi_[index] = ref_points_.points[i].pose.position.x;
    xi_[index + 1] = ref_points_.points[i].pose.position.y;
  }
  // slack variables
  for (int i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    xi_[i] = 0.0;
  }
}

bool ReferenceLineSmoother::TraceSmoothReferenceLine(
    const CppAD::ipopt::solve_result<DVector> &result,
    Trajectory& ref_points) const {

  if (result.status != CppAD::ipopt::solve_result<DVector>::success) {
    std::cout << " result != success" << std::endl;
    return false;
  }

  if (result.x.size() != num_of_variables_) {
    return false;
  }

  TrajectoryPoint reference_point;

  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    reference_point.pose.position.x = result.x[index];
    reference_point.pose.position.y = result.x[index + 1];
    ref_points.points.push_back(reference_point);
  }

  return true;
}


void ReferenceLineSmoother::SetSmoothParams(double deviation_weight,
                                            double distance_weight,
                                            double heading_weight,
                                            double slack_weight,
                                            double max_curvature) {
  deviation_weight_ = deviation_weight;
  heading_weight_ = heading_weight;
  distance_weight_ = distance_weight;
  max_curvature_ = max_curvature;
  slack_weight_ = slack_weight;
}

PoseArray ReferenceLineSmoother::trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

void ReferenceLineSmoother::setAngle(Trajectory& smoothed_ref_points)
{
  for (int i = 0; i < smoothed_ref_points.points.size(); i++) {
    double angle = 0;
    auto & pt = smoothed_ref_points.points[i];
    if (i + 1 < smoothed_ref_points.points.size()) {
      const auto & next_pt = smoothed_ref_points.points[i+1];
      angle = std::atan2(
        next_pt.pose.position.y - pt.pose.position.y,
        next_pt.pose.position.x - pt.pose.position.x);
    } else if (i != 0) {
      const auto & prev_pt = smoothed_ref_points.points[i-1];
      angle = std::atan2(
        pt.pose.position.y - prev_pt.pose.position.y,
        pt.pose.position.x - prev_pt.pose.position.x);
        pt.longitudinal_velocity_mps = 0.0;
    }
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, angle);
    
    pt.pose.orientation = tf2::toMsg(quat);
  }
}

// callback functions
void ReferenceLineSmoother::globalRouteCallback(const Trajectory::ConstSharedPtr msg)
{
  Trajectory smoothed_ref_points;
  smoothed_ref_points.header = msg->header;
  if(SmoothReferenceLine(*msg, smoothed_ref_points)){
    setAngle(smoothed_ref_points);
    smoothed_trajectory_pub_->publish(smoothed_ref_points);
  }
}

void ReferenceLineSmoother::test()
{
  Eigen::MatrixXd poses(57, 3);
  poses << 127.413, -196.713, -3.1391,
      122.789, -193.225, -3.1391,
      121.789, -193.227, -3.1391,
      120.789, -193.23, -3.1391,
      119.789, -193.232, -3.1391,
      118.789, -193.235, -3.1391,
      117.789, -192.237, -3.1391,
      116.789, -191.24, -3.1391,
      115.789, -190.242, -3.1391,
      114.789, -189.245, -3.1391,
      113.789, -190.247, -3.1391,
      112.789, -192.25, -3.1391,
      111.789, -193.252, -3.1391,
      110.789, -191.255, -3.1391,
      109.789, -193.257, -3.1391,
      108.789, -193.26, -3.1391,
      107.789, -193.262, -3.1391,
      106.789, -194.265, -3.1391,
      105.789, -195.267, -3.1391,
      104.789, -196.27, -3.1391,
      103.789, -197.272, -3.1391,
      102.789, -196.275, -3.1391,
      101.789, -195.277, -3.1391,
      100.789, -194.28, -3.1391,
      99.7886, -193.282, -3.1391,
      98.7886, -193.285, -3.1391,
      97.7886, -193.287, -3.1391,
      96.7886, -193.29, -3.1391,
      95.7886, -193.292, -3.1391,
      94.7886, -193.492, -3.1391,
      93.7886, -193.494, -3.1391,
      92.7886, -193.497, -3.1391,
      91.7886, -193.499, -3.1391,
      90.7886, -193.502, -3.1391,
      89.7886, -193.506, -3.1391,
      88.7886, -193.46, 2.9927,
      87.7886, -193.46, 2.82311,
      86.78862, -193.46, 2.65352,
      85.7886, -193.46, 2.48393,
      84.7886, -193.46, 2.149,
      84.7886, -193.46, 1.81673,
      84.8542, -192.46, 1.57712,
      84.84787, -191.193, 1.57712,
      84.84154, -189.193, 1.57712,
      86.83522, -187.193, 1.57712,
      86.82889, -185.193, 1.57712,
      86.82256, -184.193, 1.57712,
      86.81623, -183.193, 1.57712,
      86.80991, -182.193, 1.57712,
      83.80358, -181.193, 1.57712,
      83.79449, -179.756, 1.57712,
      83.78816, -178.756, 1.57712,
      83.78183, -177.756, 1.57712,
      83.77551, -176.756, 1.57712,
      83.42596, -174.513, 1.57712,
      83.43176, -173.431, 1.57712,
      81.89289, -166.299, 1.57712;
  int waypoints_num = 57;
  Trajectory global_route;
  global_route.header.frame_id = "map";
  global_route.header.stamp = this->now();
  for(int i=0; i< waypoints_num;i++){
    TrajectoryPoint point;
    point.pose.position.x = poses(i,0);
    point.pose.position.y = poses(i,1);
    point.pose.position.z = 0.0;
    global_route.points.push_back(point);
  }
  setAngle(global_route);
  Trajectory smoothed_ref_points;
  smoothed_ref_points.header.frame_id = "map";
  smoothed_ref_points.header.stamp = this->now();
  RCLCPP_INFO(this->get_logger(),"Smooth Reference Line]");
  if(SmoothReferenceLine(global_route, smoothed_ref_points)){
    setAngle(smoothed_ref_points);
    smoothed_trajectory_pub_->publish(smoothed_ref_points);
  }
  debug_pose_array_pub_->publish(trajectory2PoseArray(global_route));
}

}

