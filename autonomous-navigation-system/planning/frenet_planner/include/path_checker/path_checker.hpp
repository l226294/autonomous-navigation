#ifndef _PATH_CHECKER_HPP_
#define _PATH_CHECKER_HPP_

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>
#include "frenet_frame.hpp"
#include "utils.hpp"

namespace planning {


// Helper Functions
std::vector<std::vector<double>> compute_dists_square(std::vector<Point2D> &X,
                                               std::vector<Point2D> &Y);

class PathChecker {
public:
  PathChecker() = default;

  PathChecker(Setting &settings);
 
  bool checkConstraints(FrenetPath &path, std::vector<std::vector<Point2D>> &obstacles);
  bool checkCollision(FrenetPath &path, std::vector<std::vector<Point2D>> &obstacles);

private:
  Setting settings_;
};

} // namespace planning

#endif // _PATH_CHECKER_HPP_
