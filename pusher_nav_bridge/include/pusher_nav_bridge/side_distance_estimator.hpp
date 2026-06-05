#ifndef PUSHER_NAV_BRIDGE__SIDE_DISTANCE_ESTIMATOR_HPP_
#define PUSHER_NAV_BRIDGE__SIDE_DISTANCE_ESTIMATOR_HPP_

#include "pusher_nav_bridge/types.hpp"

#include <cstddef>
#include <vector>

namespace pusher_nav_bridge
{

struct SideEstimatorConfig
{
  double target_left_distance_m = 1.1;
  double target_right_distance_m = 1.1;
  double x_min_m = -2.0;
  double x_max_m = 3.0;
  double y_min_m = 0.2;
  double y_max_m = 5.0;
  double side_distance_gate_m = 1.0;
  double ransac_inlier_threshold_m = 0.08;
  std::size_t min_inliers = 8U;
  double line_heading_gate_deg = 45.0;
};

struct SideDistanceEstimate
{
  bool detected = false;
  FollowSide side = FollowSide::Left;
  double distance_m = 0.0;
  double active_offset_m = 0.0;
  double heading_error_rad = 0.0;
  double confidence = 0.0;
  std::size_t inlier_count = 0U;
};

SideDistanceEstimate estimate_side_distance(
  const std::vector<Point2D> & base_points,
  FollowSide follow_side,
  const SideEstimatorConfig & config);

}  // namespace pusher_nav_bridge

#endif  // PUSHER_NAV_BRIDGE__SIDE_DISTANCE_ESTIMATOR_HPP_
