#include "pusher_nav_bridge/route_processing.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace pusher_nav_bridge
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point2D subtract(const Point2D & a, const Point2D & b)
{
  return Point2D{a.x - b.x, a.y - b.y};
}

Point2D add(const Point2D & a, const Point2D & b)
{
  return Point2D{a.x + b.x, a.y + b.y};
}

Point2D scale(const Point2D & point, double value)
{
  return Point2D{point.x * value, point.y * value};
}

double dot(const Point2D & a, const Point2D & b)
{
  return (a.x * b.x) + (a.y * b.y);
}

double cross(const Point2D & a, const Point2D & b)
{
  return (a.x * b.y) - (a.y * b.x);
}

bool finite_point(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

std::vector<Point2D> clean_points(
  const std::vector<Point2D> & raw_points,
  double min_point_gap_m)
{
  std::vector<Point2D> points;
  points.reserve(raw_points.size());

  const double min_gap = std::max(0.0, min_point_gap_m);
  for (const auto & point : raw_points) {
    if (!finite_point(point)) {
      continue;
    }
    if (!points.empty() && distance(points.back(), point) < min_gap) {
      continue;
    }
    points.push_back(point);
  }

  return points;
}

std::vector<double> cumulative_lengths(const std::vector<Point2D> & points)
{
  std::vector<double> lengths(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    lengths[i] = lengths[i - 1] + distance(points[i - 1], points[i]);
  }
  return lengths;
}

Point2D interpolate_at_s(
  const std::vector<Point2D> & points,
  const std::vector<double> & lengths,
  double target_s)
{
  if (points.empty()) {
    return Point2D{};
  }
  if (target_s <= 0.0) {
    return points.front();
  }
  if (target_s >= lengths.back()) {
    return points.back();
  }

  auto upper = std::lower_bound(lengths.begin(), lengths.end(), target_s);
  const std::size_t index = static_cast<std::size_t>(upper - lengths.begin());
  if (index == 0U) {
    return points.front();
  }

  const auto s0 = lengths[index - 1U];
  const auto s1 = lengths[index];
  const auto span = std::max(kEpsilon, s1 - s0);
  const auto ratio = (target_s - s0) / span;
  return add(points[index - 1U], scale(subtract(points[index], points[index - 1U]), ratio));
}

std::vector<Point2D> resample_points(
  const std::vector<Point2D> & points,
  double step_m)
{
  if (points.size() < 2U) {
    return points;
  }

  const auto lengths = cumulative_lengths(points);
  const auto total_length = lengths.back();
  const auto step = std::max(0.05, step_m);

  std::vector<Point2D> sampled;
  for (double s = 0.0; s < total_length; s += step) {
    sampled.push_back(interpolate_at_s(points, lengths, s));
  }
  if (sampled.empty() || distance(sampled.back(), points.back()) > kEpsilon) {
    sampled.push_back(points.back());
  }

  return sampled;
}

std::vector<double> compute_headings(
  const std::vector<Point2D> & points,
  const std::vector<double> & lengths,
  double heading_window_m)
{
  std::vector<double> headings(points.size(), 0.0);
  if (points.size() < 2U) {
    return headings;
  }

  const auto window = std::max(0.05, heading_window_m);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto s = lengths[i];
    auto prev = interpolate_at_s(points, lengths, std::max(0.0, s - window));
    auto next = interpolate_at_s(points, lengths, std::min(lengths.back(), s + window));
    if (distance(prev, next) <= kEpsilon) {
      if (i + 1U < points.size()) {
        next = points[i + 1U];
      } else {
        prev = points[i - 1U];
      }
    }
    headings[i] = std::atan2(next.y - prev.y, next.x - prev.x);
  }

  return headings;
}

double perpendicular_distance(
  const Point2D & point,
  const Point2D & line_start,
  const Point2D & line_end)
{
  const auto line = subtract(line_end, line_start);
  const auto len = std::hypot(line.x, line.y);
  if (len <= kEpsilon) {
    return distance(point, line_start);
  }
  return std::fabs(cross(line, subtract(point, line_start))) / len;
}

void rdp_recursive(
  const std::vector<RoutePoint> & points,
  std::size_t start,
  std::size_t end,
  double tolerance,
  std::vector<std::size_t> & keep)
{
  if (end <= start + 1U) {
    return;
  }

  double max_distance = -1.0;
  std::size_t max_index = start;
  for (std::size_t i = start + 1U; i < end; ++i) {
    const auto dist = perpendicular_distance(points[i].center, points[start].center, points[end].center);
    if (dist > max_distance) {
      max_distance = dist;
      max_index = i;
    }
  }

  if (max_distance > tolerance) {
    keep.push_back(max_index);
    rdp_recursive(points, start, max_index, tolerance, keep);
    rdp_recursive(points, max_index, end, tolerance, keep);
  }
}

std::vector<std::size_t> rdp_indices(
  const std::vector<RoutePoint> & points,
  double tolerance)
{
  std::vector<std::size_t> indices;
  if (points.empty()) {
    return indices;
  }
  indices.push_back(0U);
  if (points.size() > 1U) {
    rdp_recursive(points, 0U, points.size() - 1U, std::max(0.0, tolerance), indices);
    indices.push_back(points.size() - 1U);
  }
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

TurnType turn_type_from_degrees(double angle_deg)
{
  if (std::fabs(angle_deg) < 5.0) {
    return TurnType::None;
  }
  if (std::fabs(angle_deg) > 150.0) {
    return TurnType::UTurn;
  }
  return angle_deg > 0.0 ? TurnType::Left : TurnType::Right;
}

std::vector<RouteSegment> build_segments(
  const std::vector<RoutePoint> & points,
  const RouteConfig & config)
{
  std::vector<RouteSegment> segments;
  if (points.size() < 2U) {
    return segments;
  }

  auto corners = rdp_indices(points, config.rdp_tolerance_m);
  if (corners.size() < 2U) {
    corners = {0U, points.size() - 1U};
  }

  for (std::size_t i = 0; i + 1U < corners.size(); ++i) {
    const auto start_index = corners[i];
    const auto end_index = corners[i + 1U];
    const auto start_s = points[start_index].s_m;
    const auto end_s = points[end_index].s_m;
    const auto length_m = end_s - start_s;
    if (length_m < config.min_segment_length_m && i + 2U < corners.size()) {
      continue;
    }

    const auto delta = subtract(points[end_index].center, points[start_index].center);
    RouteSegment segment;
    segment.id = segments.size() + 1U;
    segment.start_index = start_index;
    segment.end_index = end_index;
    segment.start_s_m = start_s;
    segment.end_s_m = end_s;
    segment.length_m = length_m;
    segment.heading_rad = std::atan2(delta.y, delta.x);
    segments.push_back(segment);
  }

  for (std::size_t i = 0; i + 1U < segments.size(); ++i) {
    const auto turn_rad = normalize_angle(segments[i + 1U].heading_rad - segments[i].heading_rad);
    segments[i].turn_angle_deg = turn_rad * 180.0 / kPi;
    segments[i].turn_type = turn_type_from_degrees(segments[i].turn_angle_deg);
  }

  return segments;
}

RoutePoint make_route_point(
  const Point2D & center,
  double s_m,
  double heading,
  const RouteConfig & config)
{
  const Point2D n_left{-std::sin(heading), std::cos(heading)};
  const Point2D n_right{std::sin(heading), -std::cos(heading)};

  RoutePoint point;
  point.center = center;
  point.s_m = s_m;
  point.heading_rad = heading;
  point.left_edge = add(center, scale(n_left, config.target_left_distance_m));
  point.right_edge = add(center, scale(n_right, config.target_right_distance_m));
  return point;
}

std::vector<RoutePoint> build_route_points(
  const std::vector<Point2D> & centers,
  const RouteConfig & config)
{
  const auto lengths = cumulative_lengths(centers);
  const auto headings = compute_headings(centers, lengths, config.heading_window_m);

  std::vector<RoutePoint> route_points;
  route_points.reserve(centers.size());
  for (std::size_t i = 0; i < centers.size(); ++i) {
    route_points.push_back(make_route_point(centers[i], lengths[i], headings[i], config));
  }
  return route_points;
}

}  // namespace

double normalize_angle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

std::vector<Point2D> load_pose_xy_file(const std::string & file_path)
{
  std::ifstream stream(file_path);
  if (!stream.is_open()) {
    throw std::runtime_error("cannot open poses file: " + file_path);
  }

  std::vector<Point2D> points;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
      tokens.push_back(token);
    }

    std::size_t offset = 0U;
    if (tokens.size() == 8U) {
      offset = 1U;
    } else if (tokens.size() == 9U) {
      offset = 2U;
    } else if (tokens.size() == 7U) {
      offset = 0U;
    } else {
      continue;
    }

    try {
      const auto x = std::stod(tokens[offset]);
      const auto y = std::stod(tokens[offset + 1U]);
      Point2D point{x, y};
      if (finite_point(point)) {
        points.push_back(point);
      }
    } catch (const std::exception &) {
      continue;
    }
  }

  return points;
}

CompiledRoute compile_route(
  const std::vector<Point2D> & raw_points,
  const RouteConfig & config)
{
  auto cleaned = clean_points(raw_points, config.min_point_gap_m);
  if (config.travel_direction == TravelDirection::Reverse) {
    std::reverse(cleaned.begin(), cleaned.end());
  }

  auto centers = resample_points(cleaned, config.resample_step_m);
  if (centers.size() < 2U) {
    throw std::runtime_error("route requires at least two valid points");
  }

  auto route_points = build_route_points(centers, config);

  if (config.poses_reference_type != PosesReferenceType::VehicleCenter) {
    std::vector<Point2D> adjusted_centers;
    adjusted_centers.reserve(route_points.size());
    for (const auto & point : route_points) {
      const Point2D n_left{-std::sin(point.heading_rad), std::cos(point.heading_rad)};
      const Point2D n_right{std::sin(point.heading_rad), -std::cos(point.heading_rad)};
      if (config.poses_reference_type == PosesReferenceType::LeftGuardrail) {
        adjusted_centers.push_back(add(point.center, scale(n_right, config.target_left_distance_m)));
      } else {
        adjusted_centers.push_back(add(point.center, scale(n_left, config.target_right_distance_m)));
      }
    }
    route_points = build_route_points(adjusted_centers, config);
  }

  CompiledRoute route;
  route.points = std::move(route_points);
  route.target_left_distance_m = config.target_left_distance_m;
  route.target_right_distance_m = config.target_right_distance_m;
  route.segments = build_segments(route.points, config);
  return route;
}

RouteLocation locate_on_route(
  const CompiledRoute & route,
  const Point2D & robot_xy)
{
  RouteLocation best;
  if (route.segments.empty() || route.points.empty()) {
    return best;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto & segment : route.segments) {
    const auto & start = route.points[segment.start_index].center;
    const auto & end = route.points[segment.end_index].center;
    const auto line = subtract(end, start);
    const auto len2 = dot(line, line);
    if (len2 <= kEpsilon) {
      continue;
    }

    const auto raw_t = dot(subtract(robot_xy, start), line) / len2;
    const auto t = std::clamp(raw_t, 0.0, 1.0);
    const auto projection = add(start, scale(line, t));
    const auto vector_to_robot = subtract(robot_xy, projection);
    const auto dist = std::hypot(vector_to_robot.x, vector_to_robot.y);
    if (dist < best_distance) {
      best_distance = dist;
      best.valid = true;
      best.segment_id = segment.id;
      best.along_segment_m = t * segment.length_m;
      best.s_robot_m = segment.start_s_m + best.along_segment_m;
      best.cross_track_error_m = dist;
      best.heading_rad = segment.heading_rad;
    }
  }

  return best;
}

}  // namespace pusher_nav_bridge
