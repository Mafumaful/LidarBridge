#include "lio_scan_monitor/scan_analysis.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace
{

constexpr double kTolerance = 1e-6;

bool near(double lhs, double rhs)
{
  return std::fabs(lhs - rhs) < kTolerance;
}

const lio_scan_monitor::DirectionObstacle & find_direction(
  const std::vector<lio_scan_monitor::DirectionObstacle> & obstacles,
  const std::string & name)
{
  for (const auto & obstacle : obstacles) {
    if (obstacle.name == name) {
      return obstacle;
    }
  }

  assert(false && "direction not found");
  return obstacles.front();
}

void yaw_from_quaternion_returns_heading_in_radians()
{
  assert(near(lio_scan_monitor::yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0));

  const auto half_yaw = M_PI / 4.0;
  assert(near(
    lio_scan_monitor::yaw_from_quaternion(0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)),
    M_PI / 2.0));
}

void scan_analysis_keeps_nearest_valid_point_per_direction()
{
  const auto inf = std::numeric_limits<float>::infinity();
  std::vector<float> ranges = {
    4.0F, inf, inf, 0.5F, 1.0F, 3.0F, 2.0F, inf, 4.2F,
  };

  const auto obstacles = lio_scan_monitor::analyze_scan_8_directions(
    ranges,
    -M_PI,
    M_PI / 4.0,
    0.1,
    10.0,
    M_PI / 4.0);

  assert(obstacles.size() == 8U);

  const auto & front = find_direction(obstacles, "front");
  assert(front.detected);
  assert(near(front.distance_m, 1.0));
  assert(near(front.x_m, 1.0));
  assert(near(front.y_m, 0.0));

  const auto & front_right = find_direction(obstacles, "front_right");
  assert(front_right.detected);
  assert(near(front_right.distance_m, 0.5));
  assert(near(front_right.x_m, std::sqrt(0.125)));
  assert(near(front_right.y_m, -std::sqrt(0.125)));

  const auto & left = find_direction(obstacles, "left");
  assert(left.detected);
  assert(near(left.distance_m, 2.0));
  assert(near(left.x_m, 0.0));
  assert(near(left.y_m, 2.0));

  const auto & right = find_direction(obstacles, "right");
  assert(!right.detected);
}

void scan_analysis_rejects_out_of_range_values()
{
  std::vector<float> ranges = {0.05F, 2.0F, 11.0F};

  const auto obstacles = lio_scan_monitor::analyze_scan_8_directions(
    ranges,
    -M_PI / 4.0,
    M_PI / 4.0,
    0.1,
    10.0,
    M_PI / 4.0);

  const auto & front = find_direction(obstacles, "front");
  assert(front.detected);
  assert(near(front.distance_m, 2.0));
}

void base_link_point_analysis_keeps_nearest_point_per_direction()
{
  const std::vector<lio_scan_monitor::ScanPoint> points = {
    {2.0, 0.0, 0U},
    {1.5, 1.5, 1U},
    {0.4, 0.4, 2U},
    {-1.2, 0.0, 3U},
    {0.0, -0.8, 4U},
  };

  const auto obstacles = lio_scan_monitor::analyze_points_8_directions(points, M_PI / 4.0);

  const auto & front = find_direction(obstacles, "front");
  assert(front.detected);
  assert(near(front.distance_m, 2.0));
  assert(near(front.x_m, 2.0));
  assert(near(front.y_m, 0.0));

  const auto & front_left = find_direction(obstacles, "front_left");
  assert(front_left.detected);
  assert(near(front_left.distance_m, std::sqrt(0.32)));
  assert(near(front_left.x_m, 0.4));
  assert(near(front_left.y_m, 0.4));
  assert(front_left.scan_index == 2U);

  const auto & back = find_direction(obstacles, "back");
  assert(back.detected);
  assert(near(back.distance_m, 1.2));
  assert(near(back.x_m, -1.2));
  assert(near(back.y_m, 0.0));

  const auto & right = find_direction(obstacles, "right");
  assert(right.detected);
  assert(near(right.distance_m, 0.8));
  assert(near(right.x_m, 0.0));
  assert(near(right.y_m, -0.8));
}

void direction_sector_geometry_uses_width_and_radius()
{
  const auto sectors = lio_scan_monitor::build_direction_sectors_8(M_PI / 4.0, 2.5);

  assert(sectors.size() == 8U);

  const auto & front = sectors[0];
  assert(front.name == "front");
  assert(near(front.center_angle_rad, 0.0));
  assert(near(front.start_angle_rad, -M_PI / 8.0));
  assert(near(front.end_angle_rad, M_PI / 8.0));
  assert(near(front.radius_m, 2.5));

  const auto & left = sectors[2];
  assert(left.name == "left");
  assert(near(left.center_angle_rad, M_PI / 2.0));
  assert(near(left.start_angle_rad, 3.0 * M_PI / 8.0));
  assert(near(left.end_angle_rad, 5.0 * M_PI / 8.0));

  const auto & back = sectors[4];
  assert(back.name == "back");
  assert(near(back.center_angle_rad, M_PI));
  assert(near(back.start_angle_rad, 7.0 * M_PI / 8.0));
  assert(near(back.end_angle_rad, 9.0 * M_PI / 8.0));
}

void direction_sector_colors_are_distinct_and_keep_alpha()
{
  std::set<std::string> colors;

  for (std::size_t index = 0; index < 8U; ++index) {
    const auto color = lio_scan_monitor::direction_sector_color(index, 0.42F);
    assert(near(color.a, 0.42F));

    const auto key = std::to_string(color.r) + "," +
      std::to_string(color.g) + "," +
      std::to_string(color.b);
    colors.insert(key);
  }

  assert(colors.size() == 8U);
}

}  // namespace

int main()
{
  yaw_from_quaternion_returns_heading_in_radians();
  scan_analysis_keeps_nearest_valid_point_per_direction();
  scan_analysis_rejects_out_of_range_values();
  base_link_point_analysis_keeps_nearest_point_per_direction();
  direction_sector_geometry_uses_width_and_radius();
  direction_sector_colors_are_distinct_and_keep_alpha();
  return 0;
}
