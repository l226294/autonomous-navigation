#ifndef _REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_
#define _REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_

#include <cppad/ipopt/solve.hpp>

#include <glog/logging.h>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include "tf2/utils.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>



namespace planning {

using std::placeholders::_1;
using geometry_msgs::msg::PoseArray;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;


class ReferenceLineSmoother : public rclcpp::Node
{
public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADVector;
  typedef CPPAD_TESTVECTOR(double) DVector;
  ReferenceLineSmoother() = default;
  ~ReferenceLineSmoother() = default;
  ReferenceLineSmoother(std::string name);

  bool SmoothReferenceLine(const Trajectory &raw_points,
                              Trajectory& smoothed_ref_points);

  void SetSmoothParams(double deviation_weight,
                       double distance_weight,
                       double heading_weight,
                       double slack_weight,
                       double max_curvature);
private:
  bool SetUpConstraint();
  void SetUpOptions();
  void SetUpInitValue();
  bool TraceSmoothReferenceLine(
      const CppAD::ipopt::solve_result<DVector> &result,
    Trajectory& ref_points) const;
  
  void setAngle(Trajectory& smoothed_ref_points);
  void globalRouteCallback(const Trajectory::ConstSharedPtr msg);
  PoseArray trajectory2PoseArray(const Trajectory & trajectory);
  void test();
  
private:
  Trajectory ref_points_;
  std::string options_;
  DVector x_l_;
  DVector x_u_;
  DVector g_l_;
  DVector g_u_;
  DVector xi_;
  double deviation_weight_ = 7.5;
  double heading_weight_ = 60.0;
  double distance_weight_ = 1.0;
  double max_curvature_ = 5.0;
  double slack_weight_ = 5.0;
  int num_of_points_;
  int slack_variable_start_index_;
  int num_of_slack_variable_;
  int slack_variable_end_index_;
  int num_curvature_constraint_;
  int curvature_constraint_start_index_;
  int curvature_constraint_end_index_;
  int num_of_variables_;
  int num_of_slack_constr_;
  int num_of_constraint_;
  int slack_constraint_start_index_;
  int slack_constraint_end_index_;

  rclcpp::Subscription<Trajectory>::SharedPtr global_route_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr smoothed_trajectory_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;

};

}

#endif // _REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_HPP_