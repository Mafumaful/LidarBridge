#ifndef PUSHER_NAV_BRIDGE__PROTOCOL_HPP_
#define PUSHER_NAV_BRIDGE__PROTOCOL_HPP_

#include "pusher_nav_bridge/side_distance_estimator.hpp"
#include "pusher_nav_bridge/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pusher_nav_bridge
{

constexpr std::size_t kPusherPayloadSize = 26U;

struct PusherFrame
{
  uint8_t path_count = 0U;
  uint8_t path_id = 0U;
  uint32_t left_distance_mm = 0U;
  int16_t lateral_offset_mm = 0;
  uint32_t right_distance_mm = 0U;
  uint16_t travel_speed_mmps = 0U;
  uint16_t ultrasonic_work_distance_mm = 0U;
  uint16_t ultrasonic_adjust_distance_mm = 0U;
  uint8_t turn_angle_deg = 0U;
  TurnType turn_type = TurnType::None;
  uint16_t rotation_speed = 0U;
  uint32_t reserved = 0U;
};

std::array<uint8_t, kPusherPayloadSize> encode_payload(const PusherFrame & frame);

PusherFrame decode_payload(const std::array<uint8_t, kPusherPayloadSize> & payload);

std::string payload_to_hex(const std::array<uint8_t, kPusherPayloadSize> & payload);

PusherFrame build_frame_from_segment(
  const CompiledRoute & route,
  const RouteSegment & segment,
  const SideDistanceEstimate & side_estimate,
  uint16_t travel_speed_mmps,
  uint16_t rotation_speed,
  uint16_t ultrasonic_work_distance_mm,
  uint16_t ultrasonic_adjust_distance_mm);

}  // namespace pusher_nav_bridge

#endif  // PUSHER_NAV_BRIDGE__PROTOCOL_HPP_
