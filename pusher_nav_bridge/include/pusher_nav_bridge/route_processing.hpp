#ifndef PUSHER_NAV_BRIDGE__ROUTE_PROCESSING_HPP_
#define PUSHER_NAV_BRIDGE__ROUTE_PROCESSING_HPP_

#include "pusher_nav_bridge/types.hpp"

#include <string>
#include <vector>

namespace pusher_nav_bridge
{

double normalize_angle(double angle_rad);

std::vector<Point2D> load_pose_xy_file(const std::string & file_path);

CompiledRoute compile_route(
  const std::vector<Point2D> & raw_points,
  const RouteConfig & config);

RouteLocation locate_on_route(
  const CompiledRoute & route,
  const Point2D & robot_xy);

}  // namespace pusher_nav_bridge

#endif  // PUSHER_NAV_BRIDGE__ROUTE_PROCESSING_HPP_
