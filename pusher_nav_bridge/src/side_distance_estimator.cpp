#include "pusher_nav_bridge/side_distance_estimator.hpp"

#include "pusher_nav_bridge/route_processing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace pusher_nav_bridge
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

struct LineModel
{
  Point2D point;
  Point2D direction;
  std::size_t inliers = 0U;
  double residual_sum = 0.0;
};

double cross(const Point2D & a, const Point2D & b)
{
  return (a.x * b.y) - (a.y * b.x);
}

Point2D subtract(const Point2D & a, const Point2D & b)
{
  return Point2D{a.x - b.x, a.y - b.y};
}

double norm(const Point2D & point)
{
  return std::hypot(point.x, point.y);
}

Point2D normalize(const Point2D & point)
{
  const auto length = norm(point);
  if (length <= kEpsilon) {
    return Point2D{1.0, 0.0};
  }
  return Point2D{point.x / length, point.y / length};
}

double point_line_distance(const Point2D & point, const LineModel & line)
{
  return std::fabs(cross(line.direction, subtract(point, line.point)));
}

bool side_matches(const Point2D & point, FollowSide side)
{
  return side == FollowSide::Left ? point.y > 0.0 : point.y < 0.0;
}

std::vector<Point2D> filter_roi(
  const std::vector<Point2D> & points,
  FollowSide side,
  const SideEstimatorConfig & config,
  bool use_distance_gate)
{
  const auto target = side == FollowSide::Left ?
    config.target_left_distance_m : config.target_right_distance_m;

  std::vector<Point2D> filtered;
  filtered.reserve(points.size());
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    if (point.x < config.x_min_m || point.x > config.x_max_m) {
      continue;
    }
    if (!side_matches(point, side)) {
      continue;
    }
    const auto lateral = std::fabs(point.y);
    if (lateral < config.y_min_m || lateral > config.y_max_m) {
      continue;
    }
    if (use_distance_gate && std::fabs(lateral - target) > config.side_distance_gate_m) {
      continue;
    }
    filtered.push_back(point);
  }
  return filtered;
}

LineModel score_line(
  const Point2D & a,
  const Point2D & b,
  const std::vector<Point2D> & points,
  double inlier_threshold_m)
{
  LineModel line;
  line.point = a;
  line.direction = normalize(subtract(b, a));

  for (const auto & point : points) {
    const auto dist = point_line_distance(point, line);
    if (dist <= inlier_threshold_m) {
      ++line.inliers;
      line.residual_sum += dist;
    }
  }

  return line;
}

LineModel fit_best_line(
  const std::vector<Point2D> & points,
  double inlier_threshold_m)
{
  LineModel best;
  if (points.size() < 2U) {
    return best;
  }

  for (std::size_t i = 0; i + 1U < points.size(); ++i) {
    for (std::size_t j = i + 1U; j < points.size(); ++j) {
      if (norm(subtract(points[j], points[i])) <= 0.2) {
        continue;
      }
      const auto line = score_line(points[i], points[j], points, inlier_threshold_m);
      const auto best_mean = best.inliers == 0U ?
        std::numeric_limits<double>::infinity() :
        best.residual_sum / static_cast<double>(best.inliers);
      const auto mean = line.inliers == 0U ?
        std::numeric_limits<double>::infinity() :
        line.residual_sum / static_cast<double>(line.inliers);
      if (line.inliers > best.inliers || (line.inliers == best.inliers && mean < best_mean)) {
        best = line;
      }
    }
  }

  return best;
}

double normalize_to_half_pi(double angle)
{
  angle = normalize_angle(angle);
  if (angle > kPi / 2.0) {
    angle -= kPi;
  } else if (angle < -kPi / 2.0) {
    angle += kPi;
  }
  return angle;
}

double confidence_from_line(
  const LineModel & line,
  std::size_t candidate_count,
  const SideEstimatorConfig & config)
{
  if (candidate_count == 0U || line.inliers == 0U) {
    return 0.0;
  }
  const auto inlier_score = std::min(
    1.0,
    static_cast<double>(line.inliers) / static_cast<double>(std::max<std::size_t>(config.min_inliers, 1U)));
  const auto ratio_score =
    static_cast<double>(line.inliers) / static_cast<double>(candidate_count);
  const auto mean_residual = line.residual_sum / static_cast<double>(line.inliers);
  const auto residual_score = std::max(0.0, 1.0 - (mean_residual / std::max(config.ransac_inlier_threshold_m, 0.01)));
  return std::clamp((0.45 * inlier_score) + (0.35 * ratio_score) + (0.20 * residual_score), 0.0, 1.0);
}

}  // namespace

SideDistanceEstimate estimate_side_distance(
  const std::vector<Point2D> & base_points,
  FollowSide follow_side,
  const SideEstimatorConfig & config)
{
  SideDistanceEstimate estimate;
  estimate.side = follow_side;

  auto candidates = filter_roi(base_points, follow_side, config, true);
  if (candidates.size() < config.min_inliers) {
    candidates = filter_roi(base_points, follow_side, config, false);
  }
  if (candidates.size() < config.min_inliers) {
    return estimate;
  }

  auto line = fit_best_line(candidates, config.ransac_inlier_threshold_m);
  if (line.inliers < config.min_inliers) {
    return estimate;
  }

  if (line.direction.x < 0.0) {
    line.direction.x *= -1.0;
    line.direction.y *= -1.0;
  }

  const auto heading = std::atan2(line.direction.y, line.direction.x);
  const auto heading_error = normalize_to_half_pi(heading);
  if (std::fabs(heading_error * 180.0 / kPi) > config.line_heading_gate_deg) {
    return estimate;
  }

  const auto distance_m = std::fabs(cross(line.direction, line.point));
  const auto target = follow_side == FollowSide::Left ?
    config.target_left_distance_m : config.target_right_distance_m;
  const auto side_sign = follow_side == FollowSide::Left ? 1.0 : -1.0;

  estimate.detected = true;
  estimate.distance_m = distance_m;
  estimate.active_offset_m = side_sign * (distance_m - target);
  estimate.heading_error_rad = heading_error;
  estimate.inlier_count = line.inliers;
  estimate.confidence = confidence_from_line(line, candidates.size(), config);
  return estimate;
}

}  // namespace pusher_nav_bridge
