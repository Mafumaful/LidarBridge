#include "lio_scan_monitor/scan_analysis.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace lio_scan_monitor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct DirectionTemplate
{
  const char * name;
  double center_angle_rad;
};

const std::array<DirectionTemplate, 8> & direction_templates()
{
  static const std::array<DirectionTemplate, 8> templates = {{
    {"front", 0.0},
    {"front_left", kPi / 4.0},
    {"left", kPi / 2.0},
    {"back_left", 3.0 * kPi / 4.0},
    {"back", kPi},
    {"back_right", -3.0 * kPi / 4.0},
    {"right", -kPi / 2.0},
    {"front_right", -kPi / 4.0},
  }};
  return templates;
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double shortest_angular_distance(double from, double to)
{
  return std::fabs(normalize_angle(from - to));
}

bool valid_range(float range, double range_min, double range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

std::vector<DirectionObstacle> make_empty_direction_obstacles()
{
  std::vector<DirectionObstacle> obstacles;
  obstacles.reserve(direction_templates().size());
  for (const auto & direction : direction_templates()) {
    obstacles.push_back(DirectionObstacle{direction.name, direction.center_angle_rad});
  }
  return obstacles;
}

}  // namespace

double yaw_from_quaternion(double x, double y, double z, double w)
{
  const auto norm = std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  const auto siny_cosp = 2.0 * ((w * z) + (x * y));
  const auto cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

RgbaColor direction_sector_color(std::size_t direction_index, float alpha)
{
  static const std::array<RgbaColor, 8> palette = {{
    {0.95F, 0.12F, 0.12F, 1.0F},  // front: red
    {1.00F, 0.48F, 0.05F, 1.0F},  // front_left: orange
    {0.95F, 0.82F, 0.10F, 1.0F},  // left: yellow
    {0.15F, 0.75F, 0.22F, 1.0F},  // back_left: green
    {0.00F, 0.75F, 0.78F, 1.0F},  // back: cyan
    {0.10F, 0.35F, 1.00F, 1.0F},  // back_right: blue
    {0.58F, 0.22F, 0.95F, 1.0F},  // right: purple
    {1.00F, 0.18F, 0.68F, 1.0F},  // front_right: pink
  }};

  auto color = palette[direction_index % palette.size()];
  color.a = alpha;
  return color;
}

std::vector<DirectionSector> build_direction_sectors_8(
  double sector_width_rad,
  double radius_m)
{
  std::vector<DirectionSector> sectors;
  sectors.reserve(direction_templates().size());

  if (sector_width_rad <= 0.0 || radius_m <= 0.0) {
    return sectors;
  }

  const auto half_sector = sector_width_rad / 2.0;
  for (const auto & direction : direction_templates()) {
    sectors.push_back(DirectionSector{
      direction.name,
      direction.center_angle_rad,
      direction.center_angle_rad - half_sector,
      direction.center_angle_rad + half_sector,
      radius_m});
  }

  return sectors;
}

std::vector<DirectionObstacle> analyze_points_8_directions(
  const std::vector<ScanPoint> & points,
  double sector_width_rad)
{
  auto obstacles = make_empty_direction_obstacles();

  if (points.empty() || sector_width_rad <= 0.0) {
    return obstacles;
  }

  const auto half_sector = sector_width_rad / 2.0;
  for (const auto & point : points) {
    if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m)) {
      continue;
    }

    const auto distance_m = std::hypot(point.x_m, point.y_m);
    if (distance_m <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const auto angle = std::atan2(point.y_m, point.x_m);
    for (auto & obstacle : obstacles) {
      if (shortest_angular_distance(angle, obstacle.target_angle_rad) > half_sector) {
        continue;
      }

      if (!obstacle.detected || distance_m < obstacle.distance_m) {
        obstacle.detected = true;
        obstacle.distance_m = distance_m;
        obstacle.obstacle_angle_rad = angle;
        obstacle.x_m = point.x_m;
        obstacle.y_m = point.y_m;
        obstacle.scan_index = point.scan_index;
      }
    }
  }

  return obstacles;
}

std::vector<DirectionObstacle> analyze_scan_8_directions(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  double sector_width_rad)
{
  if (ranges.empty() || angle_increment == 0.0 || sector_width_rad <= 0.0) {
    return make_empty_direction_obstacles();
  }

  std::vector<ScanPoint> points;
  points.reserve(ranges.size());

  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const auto range = ranges[index];
    if (!valid_range(range, range_min, range_max)) {
      continue;
    }

    const auto angle = normalize_angle(angle_min + (static_cast<double>(index) * angle_increment));
    const auto range_m = static_cast<double>(range);
    points.push_back(ScanPoint{range_m * std::cos(angle), range_m * std::sin(angle), index});
  }

  return analyze_points_8_directions(points, sector_width_rad);
}

}  // namespace lio_scan_monitor
