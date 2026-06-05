#ifndef LIO_SCAN_MONITOR__SCAN_ANALYSIS_HPP_
#define LIO_SCAN_MONITOR__SCAN_ANALYSIS_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace lio_scan_monitor
{

struct DirectionObstacle
{
  std::string name;
  double target_angle_rad = 0.0;
  bool detected = false;
  double distance_m = std::numeric_limits<double>::quiet_NaN();
  double obstacle_angle_rad = std::numeric_limits<double>::quiet_NaN();
  double x_m = std::numeric_limits<double>::quiet_NaN();
  double y_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t scan_index = 0U;
};

struct ScanPoint
{
  double x_m = 0.0;
  double y_m = 0.0;
  std::size_t scan_index = 0U;
};

struct DirectionSector
{
  std::string name;
  double center_angle_rad = 0.0;
  double start_angle_rad = 0.0;
  double end_angle_rad = 0.0;
  double radius_m = 0.0;
};

struct RgbaColor
{
  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;
  float a = 1.0F;
};

double yaw_from_quaternion(double x, double y, double z, double w);

RgbaColor direction_sector_color(std::size_t direction_index, float alpha);

std::vector<DirectionSector> build_direction_sectors_8(
  double sector_width_rad,
  double radius_m);

std::vector<DirectionObstacle> analyze_points_8_directions(
  const std::vector<ScanPoint> & points,
  double sector_width_rad);

std::vector<DirectionObstacle> analyze_scan_8_directions(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  double sector_width_rad);

}  // namespace lio_scan_monitor

#endif  // LIO_SCAN_MONITOR__SCAN_ANALYSIS_HPP_
