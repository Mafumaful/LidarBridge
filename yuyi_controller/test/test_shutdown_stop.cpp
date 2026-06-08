#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#define YUYI_CONTROLLER_DISABLE_MAIN
#include "../src/yuyi_controller_node.cpp"

namespace
{

std::string test_path_file()
{
  return "/home/miakho/ProjTest/RosProjTeset/src/LidarBridge/simulation/paths/generated_path.yaml";
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto log_dir = std::filesystem::path("/tmp/ros_logs");
  std::filesystem::create_directories(log_dir);
  setenv("ROS_LOG_DIR", log_dir.c_str(), 1);

  rclcpp::init(argc, argv);

  bool observed = false;
  double observed_linear_x = 1.0;
  double observed_angular_z = 1.0;

  {
    auto controller = std::make_shared<yuyi_controller::YuyiControllerNode>(
      rclcpp::NodeOptions().arguments({
      "--ros-args",
      "-p", "path_file:=" + test_path_file(),
      "-p", "cmd_vel_topic:=/test_cmd_vel",
      "-p", "show_cmd_vel_marker:=false",
    }));

    controller->set_cmd_publish_observer(
      [&](double linear_x, double angular_z) {
        observed = true;
        observed_linear_x = linear_x;
        observed_angular_z = angular_z;
      });

    controller.reset();
  }

  rclcpp::shutdown();

  if (!observed) {
    return 1;
  }

  if (std::abs(observed_linear_x) > 1e-9 || std::abs(observed_angular_z) > 1e-9) {
    return 2;
  }

  return 0;
}
