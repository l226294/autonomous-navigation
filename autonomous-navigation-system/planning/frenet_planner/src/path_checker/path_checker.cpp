#include "path_checker/path_checker.hpp"
namespace planning {

// Helper Function
std::vector<std::vector<double>> compute_dists_square(std::vector<Point2D> &X,
                                               std::vector<Point2D> &Y) {
  std::vector<std::vector<double>> res(X.size(), std::vector<double>(Y.size()));
  for (int i = 0; i < X.size(); i++) {
    for (int j = 0; j < Y.size(); j++) {
      double delta_x = X[i].x - Y[j].x;
      double delta_y = X[i].y - Y[j].y;
      res[i][j] = std::pow(delta_x,2) + std::pow(delta_y,2);
    }
  }
  return res;
}


// class PathChecker
PathChecker::PathChecker(Setting &settings)
    : settings_(settings) {}


bool PathChecker::checkConstraints(FrenetPath& path, std::vector<std::vector<Point2D>> &obstacles)
{
  path.collision_passed = true;
  bool passed = true;
  for (int i = 0; i < path.c.size(); i++)
  {
    
    if (path.s_d[i] > settings_.max_speed)
    {
      passed = false;
      break;
    }
    else if (path.s_dd[i] > settings_.max_accel)
    {
      passed = false;
      break;
    }
    else if (path.s_dd[i] < settings_.max_decel)
    {
      passed = false;
      break;
    }
    // else if (std::abs(path.c[i]) > settings_.max_curvature)
    // {

    //   passed = false;
    //   break;
    // }
    else if(!checkCollision(path, obstacles))
    {
      
      passed = false;
      path.collision_passed = false;
      break;
    }
  }

  path.constraint_passed = passed;
  return passed;
}


bool PathChecker::checkCollision(FrenetPath &path,
                                 std::vector<std::vector<Point2D>> &obstacles) {
  bool collision_free = true;
  
  // Iterate over the points in the path.
  
  for (int i = 0; i < path.c.size(); i++) {
    // outer layer

    std::vector<Point2D> outer_circle_locations(settings_.outer_circle_offsets.size(),
                                          Point2D(0, 0));
    
    for (int j = 0; j < outer_circle_locations.size(); j++) {
      outer_circle_locations[j].x =
          path.x[i] + settings_.outer_circle_offsets[j] * cos(path.yaw[i]);
      outer_circle_locations[j].y =
          path.y[i] + settings_.outer_circle_offsets[j] * sin(path.yaw[i]);
    }

    for (int j = 0; j < obstacles.size(); j++) {
      std::vector<std::vector<double>> collision_dists =
          compute_dists_square(obstacles[j], outer_circle_locations);
      for (std::vector<double> &dists : collision_dists) {
        for (int k = 0; k < dists.size(); k++) {
          if (dists[k] < std::pow(settings_.outer_circle_radius[k], 2)) {
            collision_free = false;
            break;
          }
        }
        if (!collision_free)
          break;
      }
      if (!collision_free)
        break;
    }

    if(!collision_free){
      collision_free = true;
    }else{
      continue;
    }

    // inner layer
    std::vector<Point2D> inner_circle_locations(settings_.inner_circle_offsets.size(),
                                          Point2D(0, 0));
    
    for (int j = 0; j < inner_circle_locations.size(); j++) {
      inner_circle_locations[j].x =
          path.x[i] + settings_.inner_circle_offsets[j] * cos(path.yaw[i]);
      inner_circle_locations[j].y =
          path.y[i] + settings_.inner_circle_offsets[j] * sin(path.yaw[i]);
    }
    for (int j = 0; j < obstacles.size(); j++) {
      std::vector<std::vector<double>> collision_dists =
          compute_dists_square(obstacles[j], inner_circle_locations);
      for (std::vector<double> &dists : collision_dists) {
        for (int k = 0; k < dists.size(); k++) {
          if (dists[k] < std::pow(settings_.inner_circle_radius[k],2)) {
            collision_free = false;
            break;
          }
        }
        if (!collision_free)
          break;
      }
      if (!collision_free)
        break;
    }
    if (!collision_free)
      break;
  }
  
  return collision_free;
}



} // namespace planning
