#ifndef _FRENET_FRAME_HPP_
#define _FRENET_FRAME_HPP_

#include <math.h>
#include <stdio.h>
#include <tgmath.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "utils.hpp"
#include "Eigen/Dense"

namespace planning {

struct FrenetState
{
  FrenetState() : s(0.0), d(0.0) {}
  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double d;
  double d_d;
  double d_dd;
  double d_ddd;
};

class FrenetPath
{
 public:

  FrenetPath() {};

  virtual ~FrenetPath() {};

  friend bool operator < (const FrenetPath& lhs, const FrenetPath& rhs)
  {
    return lhs.final_cost < rhs.final_cost;
  }
  friend bool operator > (const FrenetPath& lhs, const FrenetPath& rhs)
  {
    return lhs.final_cost > rhs.final_cost;
  }

  int traj_index;
  // checks
  bool constraint_passed;
  bool collision_passed;
  // costs
  double final_cost;  // final cost for a generated trajectory
  // time list
  std::vector<double> t;
  // longitudinal
  std::vector<double> s;
  std::vector<double> s_d;
  std::vector<double> s_dd;
  std::vector<double> s_ddd;
  // lateral
  std::vector<double> d;
  std::vector<double> d_d;
  std::vector<double> d_dd;
  std::vector<double> d_ddd;
  // state
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> ds;
  std::vector<double> c;

};


class Frenet {
 public:
  Frenet();
  Frenet(const std::vector<Point2D> &path, bool spec_origin = false,
         Point2D origin = Point2D());

  bool ToFrenet(const VehicleState& curr_state, FrenetState &p_sd) const;
  
  bool ToCartesian(const FrenetState p_sd, Point2D &p_xy) const;
  bool ToCartesian(const FrenetState p_sd, Point2D &p_xy,
                   double &road_dir) const;
  void SetPath(const std::vector<Point2D> &path, bool spec_origin = false,
               Point2D origin = Point2D());

  std::vector<double> path_s_m;
 private:
  // Waypoints of the center line of the road in the global map coordinates
  std::vector<Point2D> path_m;
  
  std::vector<double> path_theta_m;
  void CreateSPath(bool spec_origin = false, Point2D origin = Point2D());
};

}

#endif // _FRENET_FRAME_HPP_
