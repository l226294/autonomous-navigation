#include "reference_line/reference_line_smooth_ipopt_interface.hpp"

namespace planning {

void ReferenceLineSmoothIpoptInterface::operator()(
    ReferenceLineSmoothIpoptInterface::ADvector &fg,
    const ReferenceLineSmoothIpoptInterface::ADvector &x) 
{
  fg[0] = 0.0;
  // deviation from raw reference line
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    fg[0] += ref_deviation_weight_ * ((x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first)
        + (x[index + 1] - ref_points_[i].second) * (x[index + 1] - ref_points_[i].second));
  }
  // the theta error cost;
  for (int i = 0; i < num_of_points_ - 2; ++i) {
    int findex = i * 2;
    int mindex = findex + 2;
    int lindex = mindex + 2;
    fg[0] += heading_weight_ * (CppAD::pow((x[findex] + x[lindex] - 2.0 * x[mindex]), 2) +
        CppAD::pow((x[findex + 1] + x[lindex + 1] - 2.0 * x[mindex + 1]), 2));
  }
  // the total length cost
  for (int i = 0; i < num_of_points_ - 1; ++i) {
    int findex = i * 2;
    int nindex = findex + 2;
    fg[0] += length_weight_ * (CppAD::pow(x[findex] - x[nindex], 2) +
        CppAD::pow(x[findex + 1] - x[nindex + 1], 2));
  }

  for (int i = slack_variable_start_index_; i < slack_variable_end_index_; ++i) {
    fg[0] += slack_weight_ * x[i];
  }

  for (int i = 0; i + 2 < num_of_points_; ++i) {
    int findex = i * 2;
    int mindex = findex + 2;
    int lindex = mindex + 2;

    fg[curvature_constraint_start_index_ + 1 + i] = (((x[findex] + x[lindex]) - 2.0 * x[mindex]) *
        ((x[findex] + x[lindex]) - 2.0 * x[mindex]) +
        ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]) *
            ((x[findex + 1] + x[lindex + 1]) - 2.0 * x[mindex + 1]));
  }
}

ReferenceLineSmoothIpoptInterface::ReferenceLineSmoothIpoptInterface(
  const std::vector<std::pair<double,double>> &ref_points) 
{
  ref_points_ = ref_points;
  num_of_points_ = ref_points_.size();
  num_of_slack_variable_ = num_of_points_ - 2;
  num_of_variables_ = num_of_points_ * 2 + num_of_slack_variable_;
  num_curvature_constraint_ = num_of_points_ - 2;
  slack_variable_start_index_ = num_of_points_ * 2;
  slack_variable_end_index_ = slack_variable_start_index_ + num_of_slack_variable_;
  num_of_constraint_ = num_curvature_constraint_;
  curvature_constraint_start_index_ = 0;
  curvature_constraint_end_index_ = curvature_constraint_start_index_ + num_curvature_constraint_;

}

}
