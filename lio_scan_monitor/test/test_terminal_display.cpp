#include "lio_scan_monitor/terminal_display.hpp"

#include <cassert>
#include <string>
#include <vector>

namespace
{

void panel_uses_ansi_refresh_and_color_when_enabled()
{
  const lio_scan_monitor::TerminalPose pose{1.0, 2.0, 0.1, 1.57};
  std::vector<lio_scan_monitor::DirectionObstacle> obstacles;
  obstacles.push_back(lio_scan_monitor::DirectionObstacle{
    "front",
    0.0,
    true,
    1.2,
    0.0,
    1.2,
    0.0,
    10U});

  const auto panel = lio_scan_monitor::render_terminal_panel(
    pose,
    obstacles,
    "/fastlio2/lio_odom",
    "/scan",
    "base_link",
    lio_scan_monitor::TerminalDisplayOptions{true, true});

  assert(panel.find("\033[H\033[2J") == 0U);
  assert(panel.find("\033[1;36mLIO Scan Monitor\033[0m") != std::string::npos);
  assert(panel.find("Odom /fastlio2/lio_odom") != std::string::npos);
  assert(panel.find("front") != std::string::npos);
  assert(panel.find("1.200 m") != std::string::npos);
}

void panel_can_disable_color_and_in_place_refresh()
{
  const auto panel = lio_scan_monitor::render_terminal_panel(
    std::nullopt,
    {},
    "/odom",
    "/scan",
    "base_link",
    lio_scan_monitor::TerminalDisplayOptions{false, false});

  assert(panel.find("\033[") == std::string::npos);
  assert(panel.find("waiting for odom") != std::string::npos);
  assert(panel.find("waiting for scan") != std::string::npos);
}

}  // namespace

int main()
{
  panel_uses_ansi_refresh_and_color_when_enabled();
  panel_can_disable_color_and_in_place_refresh();
  return 0;
}
