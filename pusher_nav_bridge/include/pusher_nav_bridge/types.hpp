#ifndef PUSHER_NAV_BRIDGE__TYPES_HPP_
#define PUSHER_NAV_BRIDGE__TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace pusher_nav_bridge
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

enum class FollowSide
{
  Left,
  Right,
};

enum class TravelDirection
{
  Forward,
  Reverse,
};

enum class PosesReferenceType
{
  VehicleCenter,
  LeftGuardrail,
  RightGuardrail,
};

enum class TurnType : uint8_t
{
  None = 0,
  Left = 1,
  Right = 2,
  UTurn = 3,
  InPlace = 4,
};

struct RouteConfig
{
  double target_left_distance_m = 1.1;
  double target_right_distance_m = 1.1;
  double min_point_gap_m = 0.05;
  double resample_step_m = 0.5;
  double heading_window_m = 0.5;
  double rdp_tolerance_m = 0.15;
  double min_segment_length_m = 0.5;
  TravelDirection travel_direction = TravelDirection::Forward;
  PosesReferenceType poses_reference_type = PosesReferenceType::VehicleCenter;
};

struct RoutePoint
{
  Point2D center;
  Point2D left_edge;
  Point2D right_edge;
  double heading_rad = 0.0;
  double s_m = 0.0;
};

struct RouteSegment
{
  std::size_t id = 0U;
  std::size_t start_index = 0U;
  std::size_t end_index = 0U;
  double start_s_m = 0.0;
  double end_s_m = 0.0;
  double length_m = 0.0;
  double heading_rad = 0.0;
  double turn_angle_deg = 0.0;
  TurnType turn_type = TurnType::None;
};

struct CompiledRoute
{
  std::vector<RoutePoint> points;
  std::vector<RouteSegment> segments;
  double target_left_distance_m = 1.1;
  double target_right_distance_m = 1.1;
};

struct RouteLocation
{
  bool valid = false;
  std::size_t segment_id = 0U;
  double s_robot_m = 0.0;
  double along_segment_m = 0.0;
  double cross_track_error_m = std::numeric_limits<double>::quiet_NaN();
  double heading_rad = 0.0;
};

}  // namespace pusher_nav_bridge

#endif  // PUSHER_NAV_BRIDGE__TYPES_HPP_
