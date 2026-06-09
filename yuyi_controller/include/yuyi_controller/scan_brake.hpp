#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace yuyi_controller::scan_brake
{

enum class SectorId : std::size_t
{
  Front = 0U,
  LeftFront,
  Left,
  LeftRear,
  Rear,
  RightRear,
  Right,
  RightFront,
  Count
};

struct SectorConfig
{
  bool enabled{false};
  double brake_distance_m{0.5};
};

struct SectorObservation
{
  bool has_valid_range{false};
  double nearest_range_m{std::numeric_limits<double>::infinity()};
};

struct Config
{
  bool use_scan_brake{false};
  double max_age_sec{0.5};
  std::array<SectorConfig, static_cast<std::size_t>(SectorId::Count)> sectors{};
};

struct CachedScan
{
  sensor_msgs::msg::LaserScan scan;
  rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  bool has_scan{false};
};

struct Evaluation
{
  bool waiting_for_scan{false};
  bool scan_stale{false};
  bool should_brake{false};
  SectorId blocking_sector{SectorId::Front};
  std::array<SectorObservation, static_cast<std::size_t>(SectorId::Count)> observations{};
  std::string reason;
};

SectorId classify_sector(double angle_rad);

std::array<SectorObservation, static_cast<std::size_t>(SectorId::Count)> observe_sectors(
  const sensor_msgs::msg::LaserScan & scan);

Evaluation evaluate(
  const Config & config,
  const CachedScan & cached_scan,
  const rclcpp::Time & now);

const char * sector_name(SectorId sector);

}  // namespace yuyi_controller::scan_brake
