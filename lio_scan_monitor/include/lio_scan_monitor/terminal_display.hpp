#ifndef LIO_SCAN_MONITOR__TERMINAL_DISPLAY_HPP_
#define LIO_SCAN_MONITOR__TERMINAL_DISPLAY_HPP_

#include <optional>
#include <string>
#include <vector>

#include "lio_scan_monitor/scan_analysis.hpp"

namespace lio_scan_monitor
{

struct TerminalPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;
};

struct TerminalDisplayOptions
{
  bool use_color = true;
  bool refresh_in_place = true;
};

std::string render_terminal_panel(
  const std::optional<TerminalPose> & pose,
  const std::vector<DirectionObstacle> & obstacles,
  const std::string & odom_topic,
  const std::string & scan_topic,
  const std::string & frame_id,
  const TerminalDisplayOptions & options);

}  // namespace lio_scan_monitor

#endif  // LIO_SCAN_MONITOR__TERMINAL_DISPLAY_HPP_
