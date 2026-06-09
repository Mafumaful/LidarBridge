#include "yuyi_controller/scan_brake.hpp"

#include <algorithm>
#include <cmath>

namespace yuyi_controller::scan_brake
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle_rad)
{
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  return angle_rad;
}

bool is_valid_range(float range, const sensor_msgs::msg::LaserScan & scan)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

}  // namespace

SectorId classify_sector(double angle_rad)
{
  const auto angle = normalize_angle(angle_rad);
  if (angle >= (-kPi / 8.0) && angle < (kPi / 8.0)) {
    return SectorId::Front;
  }
  if (angle >= (kPi / 8.0) && angle < (3.0 * kPi / 8.0)) {
    return SectorId::LeftFront;
  }
  if (angle >= (3.0 * kPi / 8.0) && angle < (5.0 * kPi / 8.0)) {
    return SectorId::Left;
  }
  if (angle >= (5.0 * kPi / 8.0) && angle < (7.0 * kPi / 8.0)) {
    return SectorId::LeftRear;
  }
  if (angle >= (7.0 * kPi / 8.0) || angle < (-7.0 * kPi / 8.0)) {
    return SectorId::Rear;
  }
  if (angle >= (-7.0 * kPi / 8.0) && angle < (-5.0 * kPi / 8.0)) {
    return SectorId::RightRear;
  }
  if (angle >= (-5.0 * kPi / 8.0) && angle < (-3.0 * kPi / 8.0)) {
    return SectorId::Right;
  }
  return SectorId::RightFront;
}

std::array<SectorObservation, static_cast<std::size_t>(SectorId::Count)> observe_sectors(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::array<SectorObservation, static_cast<std::size_t>(SectorId::Count)> observations{};

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const auto range = scan.ranges[i];
    if (!is_valid_range(range, scan)) {
      continue;
    }

    const auto angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const auto sector_index = static_cast<std::size_t>(classify_sector(angle));
    auto & observation = observations[sector_index];
    observation.has_valid_range = true;
    observation.nearest_range_m = std::min(observation.nearest_range_m, static_cast<double>(range));
  }

  return observations;
}

Evaluation evaluate(
  const Config & config,
  const CachedScan & cached_scan,
  const rclcpp::Time & now)
{
  Evaluation evaluation;
  if (!config.use_scan_brake) {
    return evaluation;
  }

  if (!cached_scan.has_scan) {
    evaluation.waiting_for_scan = true;
    evaluation.should_brake = true;
    evaluation.reason = "waiting_for_scan";
    return evaluation;
  }

  if ((now - cached_scan.received_at).seconds() > config.max_age_sec) {
    evaluation.scan_stale = true;
    evaluation.should_brake = true;
    evaluation.reason = "scan_stale";
    return evaluation;
  }

  evaluation.observations = observe_sectors(cached_scan.scan);
  for (std::size_t i = 0; i < evaluation.observations.size(); ++i) {
    const auto & sector_config = config.sectors[i];
    const auto & observation = evaluation.observations[i];
    if (!sector_config.enabled) {
      continue;
    }

    if (!observation.has_valid_range || observation.nearest_range_m < sector_config.brake_distance_m) {
      evaluation.should_brake = true;
      evaluation.blocking_sector = static_cast<SectorId>(i);
      evaluation.reason = sector_name(static_cast<SectorId>(i));
      return evaluation;
    }
  }

  return evaluation;
}

const char * sector_name(SectorId sector)
{
  switch (sector) {
    case SectorId::Front:
      return "front";
    case SectorId::LeftFront:
      return "left_front";
    case SectorId::Left:
      return "left";
    case SectorId::LeftRear:
      return "left_rear";
    case SectorId::Rear:
      return "rear";
    case SectorId::RightRear:
      return "right_rear";
    case SectorId::Right:
      return "right";
    case SectorId::RightFront:
      return "right_front";
    case SectorId::Count:
      return "count";
  }
  return "unknown";
}

}  // namespace yuyi_controller::scan_brake
