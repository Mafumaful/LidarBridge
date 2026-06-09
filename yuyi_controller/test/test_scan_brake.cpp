#include "yuyi_controller/scan_brake.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace
{

sensor_msgs::msg::LaserScan make_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(M_PI / 4.0);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges = {5.0F, 4.0F, 3.0F, 2.0F, 1.0F, 6.0F, 7.0F, 8.0F};
  return scan;
}

void classify_front_boundary_angles()
{
  using yuyi_controller::scan_brake::SectorId;
  assert(yuyi_controller::scan_brake::classify_sector(0.0) == SectorId::Front);
  assert(yuyi_controller::scan_brake::classify_sector(M_PI / 3.0) == SectorId::LeftFront);
  assert(yuyi_controller::scan_brake::classify_sector(-M_PI / 2.0) == SectorId::Right);
}

void observe_sectors_picks_nearest_valid_range()
{
  const auto observations = yuyi_controller::scan_brake::observe_sectors(make_scan());
  const auto front_index = static_cast<std::size_t>(yuyi_controller::scan_brake::SectorId::Front);
  assert(observations[front_index].has_valid_range);
  assert(std::fabs(observations[front_index].nearest_range_m - 1.0) < 1e-6);
}

void evaluate_brakes_when_enabled_sector_is_close()
{
  yuyi_controller::scan_brake::Config config;
  config.use_scan_brake = true;
  config.max_age_sec = 0.5;
  config.sectors[static_cast<std::size_t>(yuyi_controller::scan_brake::SectorId::Front)] = {
    true, 1.5
  };

  yuyi_controller::scan_brake::CachedScan cached_scan;
  cached_scan.scan = make_scan();
  cached_scan.received_at = rclcpp::Time(0, 0, RCL_ROS_TIME);
  cached_scan.has_scan = true;

  const auto evaluation = yuyi_controller::scan_brake::evaluate(
    config,
    cached_scan,
    rclcpp::Time(0, 100000000, RCL_ROS_TIME));

  assert(evaluation.should_brake);
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto log_dir = std::filesystem::path("/tmp/ros_logs");
  std::filesystem::create_directories(log_dir);
  setenv("ROS_LOG_DIR", log_dir.c_str(), 1);

  rclcpp::init(argc, argv);
  classify_front_boundary_angles();
  observe_sectors_picks_nearest_valid_range();
  evaluate_brakes_when_enabled_sector_is_close();
  rclcpp::shutdown();
  return 0;
}
