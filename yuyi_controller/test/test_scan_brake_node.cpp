#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#define YUYI_CONTROLLER_DISABLE_MAIN
#include "../src/yuyi_controller_node.cpp"

namespace
{

std::string test_path_file()
{
  return "/home/miakho/ProjTest/RosProjTeset/src/LidarBridge/simulation/paths/generated_path.yaml";
}

void configure_test_environment()
{
  const auto workspace_root = std::filesystem::path(
    "/home/miakho/ProjTest/RosProjTeset/src/LidarBridge");
  const auto package_prefix = workspace_root / "install" / "yuyi_controller";
  const auto previous_ament_prefix =
    std::string(std::getenv("AMENT_PREFIX_PATH") == nullptr ? "" : std::getenv("AMENT_PREFIX_PATH"));
  const auto updated_ament_prefix = previous_ament_prefix.empty() ?
    package_prefix.string() :
    package_prefix.string() + ":" + previous_ament_prefix;
  setenv("AMENT_PREFIX_PATH", updated_ament_prefix.c_str(), 1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
}

sensor_msgs::msg::LaserScan make_uniform_scan(float range_value)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_link";
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_max = static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(M_PI / 180.0);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges.assign(361U, range_value);
  return scan;
}

void publish_static_tf(const std::shared_ptr<rclcpp::Node> & node)
{
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  broadcaster->sendTransform(transform);
}

void spin_executor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

std::shared_ptr<yuyi_controller::YuyiControllerNode> make_controller_with_scan_brake(
  const std::vector<std::string> & extra_arguments = {})
{
  std::vector<std::string> arguments = {
    "--ros-args",
    "-p", "path_file:=" + test_path_file(),
    "-p", "cmd_vel_topic:=/test_cmd_vel",
    "-p", "show_cmd_vel_marker:=false",
    "-p", "use_scan_brake:=true",
    "-p", "scan_brake.front.enabled:=true",
    "-p", "scan_brake.front.brake_distance_m:=0.5",
  };
  arguments.insert(arguments.end(), extra_arguments.begin(), extra_arguments.end());
  return std::make_shared<yuyi_controller::YuyiControllerNode>(
    rclcpp::NodeOptions().arguments(arguments));
}

void scan_brake_waits_for_first_scan()
{
  auto controller = make_controller_with_scan_brake();
  double linear_x = 1.0;
  controller->set_cmd_publish_observer(
    [&](double observed_linear_x, double) {
      linear_x = observed_linear_x;
    });

  auto helper_node = std::make_shared<rclcpp::Node>("scan_brake_test_helper");
  publish_static_tf(helper_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(helper_node);
  spin_executor(executor, std::chrono::milliseconds(300));

  assert(std::fabs(linear_x) < 1e-6);
}

void safe_scan_allows_motion()
{
  auto controller = make_controller_with_scan_brake();
  double linear_x = 0.0;
  controller->set_cmd_publish_observer(
    [&](double observed_linear_x, double) {
      linear_x = observed_linear_x;
    });

  auto helper_node = std::make_shared<rclcpp::Node>("scan_brake_test_helper_safe");
  auto scan_publisher = helper_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  publish_static_tf(helper_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(helper_node);
  scan_publisher->publish(make_uniform_scan(5.0F));
  spin_executor(executor, std::chrono::milliseconds(500));

  assert(linear_x > 0.0);
}

void unsafe_scan_decelerates_to_stop()
{
  auto controller = make_controller_with_scan_brake();
  auto helper_node = std::make_shared<rclcpp::Node>("scan_brake_test_helper_brake");
  auto scan_publisher = helper_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  publish_static_tf(helper_node);

  double latest_linear_x = 0.0;
  controller->set_cmd_publish_observer(
    [&](double observed_linear_x, double) {
      latest_linear_x = observed_linear_x;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(helper_node);

  scan_publisher->publish(make_uniform_scan(5.0F));
  spin_executor(executor, std::chrono::milliseconds(500));
  assert(latest_linear_x > 0.0);

  auto blocked_scan = make_uniform_scan(5.0F);
  blocked_scan.ranges[180] = 0.2F;
  scan_publisher->publish(blocked_scan);
  spin_executor(executor, std::chrono::milliseconds(800));
  assert(std::fabs(latest_linear_x) < 1e-3);
}

void stale_scan_reapplies_stop()
{
  auto controller = make_controller_with_scan_brake({
    "-p", "scan_max_age_sec:=0.1",
  });
  auto helper_node = std::make_shared<rclcpp::Node>("scan_brake_test_helper_stale");
  auto scan_publisher = helper_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  publish_static_tf(helper_node);

  double latest_linear_x = 0.0;
  controller->set_cmd_publish_observer(
    [&](double observed_linear_x, double) {
      latest_linear_x = observed_linear_x;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(helper_node);

  scan_publisher->publish(make_uniform_scan(5.0F));
  spin_executor(executor, std::chrono::milliseconds(80));
  assert(latest_linear_x > 0.0);

  spin_executor(executor, std::chrono::milliseconds(300));
  assert(std::fabs(latest_linear_x) < 1e-3);
}

void dynamic_sector_threshold_update_releases_brake()
{
  auto controller = make_controller_with_scan_brake();
  auto helper_node = std::make_shared<rclcpp::Node>("scan_brake_test_helper_dynamic");
  auto scan_publisher = helper_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  publish_static_tf(helper_node);

  double latest_linear_x = 0.0;
  controller->set_cmd_publish_observer(
    [&](double observed_linear_x, double) {
      latest_linear_x = observed_linear_x;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(helper_node);

  auto blocked_scan = make_uniform_scan(5.0F);
  blocked_scan.ranges[180] = 0.2F;
  scan_publisher->publish(blocked_scan);
  spin_executor(executor, std::chrono::milliseconds(500));
  assert(std::fabs(latest_linear_x) < 1e-3);

  const auto results = controller->set_parameters({
    rclcpp::Parameter("scan_brake.front.brake_distance_m", 0.1),
    rclcpp::Parameter("scan_brake.front.enabled", true),
  });
  assert(results[0].successful);
  assert(results[1].successful);

  scan_publisher->publish(blocked_scan);
  spin_executor(executor, std::chrono::milliseconds(500));
  assert(latest_linear_x > 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto log_dir = std::filesystem::path("/tmp/ros_logs");
  std::filesystem::create_directories(log_dir);
  setenv("ROS_LOG_DIR", log_dir.c_str(), 1);
  configure_test_environment();

  rclcpp::init(argc, argv);
  scan_brake_waits_for_first_scan();
  safe_scan_allows_motion();
  unsafe_scan_decelerates_to_stop();
  stale_scan_reapplies_stop();
  dynamic_sector_threshold_update_releases_brake();
  rclcpp::shutdown();
  return 0;
}
