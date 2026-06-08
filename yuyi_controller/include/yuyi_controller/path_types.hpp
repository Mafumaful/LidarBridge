#pragma once

#include <cstddef>

namespace yuyi_controller
{

struct PathPoint
{
  std::size_t index{0U};
  double s_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature{0.0};
};

}  // namespace yuyi_controller
