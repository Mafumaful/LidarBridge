#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "yuyi_controller/path_types.hpp"

namespace yuyi_controller::path_utils
{

inline double total_path_length(const std::vector<PathPoint> & path_points)
{
  return path_points.empty() ? 0.0 : path_points.back().s_m;
}

inline std::size_t find_lookahead_index(
  const std::vector<PathPoint> & path_points,
  std::size_t nearest_index,
  double lookahead_distance_m,
  bool loop_path)
{
  if (path_points.empty()) {
    return 0U;
  }

  const auto nearest_s = path_points[nearest_index].s_m;
  if (!loop_path || path_points.size() < 2U) {
    const auto target_s = nearest_s + lookahead_distance_m;
    for (std::size_t i = nearest_index; i < path_points.size(); ++i) {
      if (path_points[i].s_m >= target_s) {
        return i;
      }
    }
    return path_points.size() - 1U;
  }

  const auto path_length = total_path_length(path_points);
  if (path_length <= 1e-6) {
    return nearest_index;
  }

  auto target_s = std::fmod(nearest_s + std::max(0.0, lookahead_distance_m), path_length);
  if (target_s < 0.0) {
    target_s += path_length;
  }

  for (std::size_t i = 0; i < path_points.size(); ++i) {
    if (path_points[i].s_m >= target_s) {
      return i;
    }
  }
  return path_points.size() - 1U;
}

inline double remaining_distance_m(
  const std::vector<PathPoint> & path_points,
  std::size_t nearest_index,
  bool loop_path)
{
  if (path_points.empty()) {
    return 0.0;
  }
  if (loop_path) {
    return std::numeric_limits<double>::infinity();
  }
  return path_points.back().s_m - path_points[nearest_index].s_m;
}

inline bool should_stop_at_goal(
  double remaining_distance,
  double goal_tolerance_m,
  bool stop_at_goal,
  bool loop_path)
{
  return stop_at_goal && !loop_path && remaining_distance <= goal_tolerance_m;
}

inline double target_speed_mps(
  double remaining_distance_m,
  double max_speed_mps,
  double max_deceleration_mps2,
  double path_curvature,
  double max_lateral_acceleration_mps2,
  bool loop_path)
{
  auto target_speed = max_speed_mps;

  if (!loop_path) {
    const auto braking_speed = std::sqrt(
      std::max(0.0, 2.0 * max_deceleration_mps2 * remaining_distance_m));
    target_speed = std::min(target_speed, braking_speed);
  }

  if (max_lateral_acceleration_mps2 > 0.0 && std::abs(path_curvature) > 1e-6) {
    const auto curvature_limited_speed = std::sqrt(
      max_lateral_acceleration_mps2 / std::abs(path_curvature));
    target_speed = std::min(target_speed, curvature_limited_speed);
  }

  return target_speed;
}

}  // namespace yuyi_controller::path_utils
