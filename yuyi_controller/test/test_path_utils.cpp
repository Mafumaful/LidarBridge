#include "yuyi_controller/path_types.hpp"
#include "yuyi_controller/path_utils.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

std::vector<yuyi_controller::PathPoint> sample_path()
{
  return {
    {0U, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1U, 1.0, 1.0, 0.0, 0.0, 0.0},
    {2U, 2.0, 2.0, 0.0, 0.0, 0.0},
    {3U, 3.0, 3.0, 0.0, 0.0, 0.0},
  };
}

void open_path_lookahead_stays_at_end()
{
  const auto path_points = sample_path();
  const auto target_index = yuyi_controller::path_utils::find_lookahead_index(
    path_points, 2U, 2.0, false);

  assert(target_index == 3U);
}

void loop_path_lookahead_wraps_to_path_start()
{
  const auto path_points = sample_path();
  const auto target_index = yuyi_controller::path_utils::find_lookahead_index(
    path_points, 3U, 0.5, true);

  assert(target_index == 1U);
}

void loop_path_never_triggers_goal_stop()
{
  const auto should_stop = yuyi_controller::path_utils::should_stop_at_goal(
    0.0, 0.2, true, true);

  assert(!should_stop);
}

void loop_path_uses_cruise_speed()
{
  const auto speed = yuyi_controller::path_utils::target_speed_mps(
    std::numeric_limits<double>::infinity(), 0.8, 0.8, true);

  assert(std::fabs(speed - 0.8) < 1e-9);
}

}  // namespace

int main()
{
  open_path_lookahead_stays_at_end();
  loop_path_lookahead_wraps_to_path_start();
  loop_path_never_triggers_goal_stop();
  loop_path_uses_cruise_speed();
  return 0;
}
