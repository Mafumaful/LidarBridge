#include "lio_scan_monitor/terminal_display.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace lio_scan_monitor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string colorize(const std::string & text, const char * color_code, bool use_color)
{
  if (!use_color) {
    return text;
  }
  return std::string(color_code) + text + "\033[0m";
}

const char * status_color(double distance_m)
{
  if (distance_m < 0.8) {
    return "\033[1;31m";
  }
  if (distance_m < 1.5) {
    return "\033[1;33m";
  }
  return "\033[1;32m";
}

}  // namespace

std::string render_terminal_panel(
  const std::optional<TerminalPose> & pose,
  const std::vector<DirectionObstacle> & obstacles,
  const std::string & odom_topic,
  const std::string & scan_topic,
  const std::string & frame_id,
  const TerminalDisplayOptions & options)
{
  std::ostringstream out;

  if (options.refresh_in_place) {
    out << "\033[H\033[2J";
  }

  out << colorize("LIO Scan Monitor", "\033[1;36m", options.use_color) << '\n';
  out << "Frame: " << colorize(frame_id, "\033[1;34m", options.use_color)
      << "  Scan: " << scan_topic << '\n';
  out << "------------------------------------------------------------\n";

  out << std::fixed << std::setprecision(3);
  if (pose) {
    out << "Odom " << odom_topic
        << ' ' << colorize("OK", "\033[1;32m", options.use_color)
        << "  x=" << pose->x << " m"
        << "  y=" << pose->y << " m"
        << "  z=" << pose->z << " m"
        << "  yaw=" << pose->yaw_rad << " rad"
        << " (" << (pose->yaw_rad * 180.0 / kPi) << " deg)\n";
  } else {
    out << "Odom " << odom_topic << ' '
        << colorize("waiting for odom", "\033[1;33m", options.use_color) << '\n';
  }

  if (obstacles.empty()) {
    out << "Scan " << scan_topic << ' '
        << colorize("waiting for scan", "\033[1;33m", options.use_color) << '\n';
    return out.str();
  }

  out << colorize("Obstacles", "\033[1;36m", options.use_color)
      << "  nearest points in " << frame_id << '\n';

  for (const auto & obstacle : obstacles) {
    out << "  " << std::setw(11) << std::left << obstacle.name << " ";
    if (!obstacle.detected) {
      out << colorize("none", "\033[2;37m", options.use_color) << '\n';
      continue;
    }

    out << colorize("detected", status_color(obstacle.distance_m), options.use_color)
        << std::fixed << std::right << std::setprecision(3)
        << "  d=" << std::setw(7) << obstacle.distance_m << " m"
        << "  x=" << std::setw(7) << obstacle.x_m << " m"
        << "  y=" << std::setw(7) << obstacle.y_m << " m"
        << "  angle=" << std::setw(7) << obstacle.obstacle_angle_rad << " rad\n";
  }

  return out.str();
}

}  // namespace lio_scan_monitor
