#include "utils.hpp"

namespace planning {

double unifyAngleRange(const double angle)
{
  auto new_angle = angle;
  while (new_angle > M_PI)
  {
    new_angle -= 2 * M_PI;
  }
  while (new_angle < -M_PI)
  {
    new_angle += 2 * M_PI;
  }
  return new_angle;
};



// Check if a value is legal (not nan or inf)
bool isLegal(const double x)
{
  return (std::isnan(x) || std::isinf(x))? false : true;
};

}