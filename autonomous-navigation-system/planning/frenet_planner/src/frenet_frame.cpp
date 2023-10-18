#include <frenet_frame.hpp>
#include <limits>

namespace planning {

Frenet::Frenet() {
  // Copy the path
  path_m = std::vector<Point2D>(0);
}

Frenet::Frenet(const std::vector<Point2D> &path, bool spec_origin,
               Point2D origin) {
  // Copy the path
  path_m = path;
  CreateSPath(spec_origin, origin);
}

void Frenet::SetPath(const std::vector<Point2D> &path, bool spec_origin,
                     Point2D origin) {
  path_m = path;
  CreateSPath(spec_origin, origin);
}

void Frenet::CreateSPath(bool spec_origin, Point2D origin) {
  if (path_m.empty()) return;

  path_s_m.resize(path_m.size());
  path_theta_m.resize(path_m.size());

  if (!spec_origin) {
    // Create the s path based on distance
    path_s_m[0] = 0.0;
    for (auto i = 1; i < path_s_m.size(); ++i) {
      path_s_m[i] = distance2D(path_m[i], path_m[i - 1]) + path_s_m[i - 1];
      Point2D dir_vec = path_m[i] - path_m[i - 1];
      path_theta_m[i - 1] = atan2(dir_vec.y, dir_vec.x);
    }
    path_theta_m[path_theta_m.size() - 1] =
        path_theta_m[path_theta_m.size() - 2];
  } else {
    int ind_closest = closestPoint(path_m, origin);
    path_s_m[ind_closest] = 0.0;

    for (int i = ind_closest - 1; i >= 0; i--) {
      path_s_m[i] =
          path_s_m[i + 1] - distance2D(path_m[i], path_m[i + 1]);
      Point2D dir_vec = path_m[i + 1] - path_m[i];
      path_theta_m[i] = atan2(dir_vec.y, dir_vec.x);
    }

    for (int i = ind_closest + 1; i < path_m.size(); i++) {
      path_s_m[i] =
          path_s_m[i - 1] + distance2D(path_m[i], path_m[i - 1]);
      Point2D dir_vec = path_m[i] - path_m[i - 1];
      path_theta_m[i - 1] = atan2(dir_vec.y, dir_vec.x);
    }
    path_theta_m[path_theta_m.size() - 1] =
        path_theta_m[path_theta_m.size() - 2];
  }
}

bool Frenet::ToFrenet(const VehicleState& current_state, FrenetState &state) const {
  
  if (path_m.empty() || path_s_m.empty()) {
    std::cout << "Empty path. Cannot compute Frenet" << std::endl;
    return false;
  }

  int closest_index = closestPoint(path_m, Point2D(current_state.x, current_state.y));
  if(closest_index == path_m.size()-1){
    std::cout << "cannot find correspond state in frenet" << std::endl;
    state.s = path_s_m.back();
    state.d = 0;
    state.s_d = 0;
    state.d_d = 0;
    state.s_dd = 0.0;
    state.d_dd = 0.0;
    state.s_ddd = 0.0;
    state.d_ddd = 0.0;
    return false;
  }

  // get next waypoint
  double heading = atan2((path_m[closest_index].y - current_state.y), (path_m[closest_index].x - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);
  
  int next_wp_id;
  if (angle > pi() / 2)
  {
    next_wp_id = closest_index + 1;
  }else{
    next_wp_id = closest_index;
  }

  if (next_wp_id >= path_m.size())
  {
    next_wp_id = path_m.size() - 1;
  }
  int prev_wp_id = next_wp_id - 1;
  if (prev_wp_id < 0)
  {
    prev_wp_id = 0;
    next_wp_id = prev_wp_id + 1;
  }

  // std::vector n from previous waypoint to next waypoint
  const double n_x = path_m[next_wp_id].x - path_m[prev_wp_id].x;
  const double n_y = path_m[next_wp_id].y - path_m[prev_wp_id].y;
  // std::vector x from previous waypoint to current position
  const double x_x = current_state.x - path_m[prev_wp_id].x;
  const double x_y = current_state.y - path_m[prev_wp_id].y;
  
  double proj_norm;
  // find the projection of x on n
  if(std::abs(n_x * n_x + n_y * n_y) < 0.001){
    proj_norm = 0;
  }else{
    proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y + 0.0005);
    if(proj_norm > 1000){
      std::cout << "proj_norm is abnormal." << proj_norm<< std::endl;
    }
  }
  
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  state.d = distance2D(x_x, x_y, proj_x, proj_y);

  const double pre_wp_yaw = atan2(n_y, n_x);

  // find the yaw of std::vector x
  const double x_yaw = atan2(x_y, x_x);
  const double yaw_x_n = unifyAngleRange(x_yaw - pre_wp_yaw);

  if (yaw_x_n < 0.0)
  {
    state.d *= -1;
  }

  double next_wp_yaw;
  if (next_wp_id + 1 < path_m.size()){
    next_wp_yaw = atan2(path_m[next_wp_id+1].y - path_m[next_wp_id].y, path_m[next_wp_id+1].x - path_m[next_wp_id].x);
  }else{
    next_wp_yaw = pre_wp_yaw;
  }

  const double lerp_yaw = pre_wp_yaw + proj_norm * (next_wp_yaw - pre_wp_yaw);
  const double delta_yaw = unifyAngleRange(current_state.yaw - lerp_yaw);
  state.s = path_s_m[prev_wp_id] + distance2D(0.0, 0.0, proj_x, proj_y);
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);
  // Give default values to the rest of the attributes
  state.s_dd = 0.0;
  state.d_dd = 0.0;
  state.s_ddd = 0.0;
  state.d_ddd = 0.0;

  return true;
}

bool Frenet::ToCartesian(const FrenetState p_sd, Point2D &p_xy) const {
  if (path_m.empty() || path_s_m.empty()) {
    std::cout << "Empty path. Cannot compute Cartesian" << std::endl;
    p_xy.x = std::numeric_limits<double>::lowest();
    p_xy.y = std::numeric_limits<double>::lowest();
    return false;
  }
  if(std::abs(p_sd.s)<0.00001){
    p_xy.x = path_m[0].x;
    p_xy.y = path_m[0].y;
    return true;
  }else if(std::abs(p_sd.s-path_s_m.back())<0.00001){
    p_xy.x = path_m.back().x;
    p_xy.y = path_m.back().y;
    return true;
  }
  uint32_t prev_point_ind;

  if (p_sd.s <= path_s_m.front() || p_sd.s >= path_s_m.back()) {
    if (p_sd.s < path_s_m.front()) {
      prev_point_ind = 0;
    } else {
      prev_point_ind = uint32_t(path_s_m.size() - 2);
    }
  } else {
    auto it = std::lower_bound(path_s_m.begin(), path_s_m.end(), p_sd.s);
    prev_point_ind = uint32_t(it - path_s_m.begin() - 1);
  }

  Point2D p1, p2;
  p1.x = path_m[prev_point_ind].x;
  p1.y = path_m[prev_point_ind].y;
  p2.x = path_m[prev_point_ind + 1].x;
  p2.y = path_m[prev_point_ind + 1].y;

  double road_dir = atan2(p2.y - p1.y, p2.x - p1.x);

  p_xy = localToGlobal(p1, road_dir,
                       Point2D(p_sd.s - path_s_m[prev_point_ind], p_sd.d));
  return true;
}

}  // end of namespace planning