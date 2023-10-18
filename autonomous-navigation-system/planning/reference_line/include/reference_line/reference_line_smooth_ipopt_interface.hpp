#ifndef _REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_
#define _REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_

#include <cppad/ipopt/solve.hpp>

#include <string>

namespace planning {
class ReferenceLineSmoothIpoptInterface {
 public:

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  explicit ReferenceLineSmoothIpoptInterface(const std::vector<std::pair<double, double>> &ref_points);
  ~ReferenceLineSmoothIpoptInterface() = default;
  void operator()(ADvector &fg, const ADvector &x);
  void set_ref_deviation_weight(double weight) { this->ref_deviation_weight_ = weight; }
  void set_curvature_weight(double weight) { this->curvature_weight_ = weight; }
  void set_length_weight(double weight) { this->length_weight_ = weight; }
  void set_heading_weight(double weight) { this->heading_weight_ = weight; }
  void set_slack_weight(double weight) { this->slack_weight_ = weight; }

 private:
  std::vector<std::pair<double, double>> ref_points_;
  double ref_deviation_weight_;
  double curvature_weight_;
  double length_weight_;
  double heading_weight_;
  double max_curvature_;
  double slack_weight_;
  int num_of_points_;
  int num_of_slack_variable_;
  int num_of_variables_;
  int num_curvature_constraint_;
  int slack_variable_start_index_;
  int slack_variable_end_index_;

  int num_of_slack_constr_;
  int num_of_constraint_;
  int curvature_constraint_start_index_;
  int curvature_constraint_end_index_;
  int slack_constraint_start_index_;
  int slack_constraint_end_index_;

};

}
#endif // _REFERENCE_LINE_REFERENCE_LINE_SMOOTH_IPOPT_INTERFACE_HPP_
