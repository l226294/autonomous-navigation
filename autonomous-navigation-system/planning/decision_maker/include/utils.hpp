#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <array>
#include <vector>
#include <limits>
#include <cmath>

namespace planning {

constexpr double pi()
{ 
  return M_PI; 
}


double unifyAngleRange(const double angle);

// Check if a value is legal (not nan or inf)
bool isLegal(const double x);


struct VehicleState
{
 public:
  VehicleState() {}
  VehicleState(const double x, const double y, const double yaw, const double speed)
    : x(x), y(y), yaw(yaw), v(speed) {}

  double x;
  double y;
  double yaw;  
  double v;
};
struct ObstacleState
{
  double x;
  double y;
  double yaw;
  double s;
  double d;
  double v;
  double distance_to_ego;
};

struct Point2D {
  Point2D() : x(0), y(0) {}

  Point2D(double x, double y) : x(x), y(y) {}

  Point2D &operator-=(const Point2D &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  friend Point2D operator-(Point2D lhs, const Point2D &rhs) {
    lhs -= rhs;
    return lhs;
  }
  
  double x, y;
};

inline double distance2D(const Point2D &p1, const Point2D &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

inline double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

inline const Point2D rotate(const Point2D &in, const double theta) {
  Point2D out;
  double s = sin(theta);
  double c = cos(theta);

  out.x = in.x * c - in.y * s;
  out.y = in.x * s + in.y * c;

  return out;
}


inline const Point2D localToGlobal(const Point2D &center, const double theta,
                            const Point2D &p) {
  Point2D out = rotate(p, theta);
  out.x += center.x;
  out.y += center.y;
  return out;
}


inline int closestPoint(std::vector<Point2D> ref_path, double x, double y) {
  int index = 0;
  int closest_index = 0;
  double min_dist = std::numeric_limits<double>::max();;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, x, y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}

inline int closestPoint(std::vector<Point2D> ref_path, Point2D query) {
  int index = 0;
  int closest_index = 0;
  double min_dist = std::numeric_limits<double>::max();;
  for (auto p : ref_path) {
    double dist = distance2D(p.x, p.y, query.x, query.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = index;
    }
    index++;
  }
  return closest_index;
}

struct Setting
{
  double tick_t;              // time tick [s]
  std::string global_frame;

  //Collision
  std::vector<double> inner_circle_offsets;
  std::vector<double> inner_circle_radius;
  std::vector<double> outer_circle_offsets;
  std::vector<double> outer_circle_radius;

  // Sampling Parameters
  double max_road_width;
  int num_width;              // number of road width samples
  double max_t;               // max prediction time [s]
  double min_t;               // min prediction time [s]
  int num_t;                  // number of time samples
  double highest_speed;       // highest target speed [m/s]
  double lowest_speed;        // lowest target speed [m/s]
  int num_speed;              // number of speed samples

  // Hard Constraints
  double max_speed;           //  [m/s]
  double max_accel;           //  [m/ss]
  double max_decel;           //  [m/ss]
  double max_curvature;       //  [rad/m]

  // Cost Weights
  double k_lat_jerk;          // jerk cost weight
  double k_lon_jerk;          // jerk cost weight
  double k_time;              // time cost weight
  double k_lat_d;             // lateral cost weight
  double k_lon_speed;         // longitudinal cost weight
  double max_jerk_s;
  double max_jerk_d;
  double k_lat;
  double k_lon;

  double vehicle_width;
  double lane_width;

};


} // namespace planning

#endif // _UTILS_HPP_
