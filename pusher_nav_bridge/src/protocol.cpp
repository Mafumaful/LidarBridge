#include "pusher_nav_bridge/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace pusher_nav_bridge
{

namespace
{

uint16_t clamp_u16(uint32_t value)
{
  return static_cast<uint16_t>(std::min<uint32_t>(value, 0xFFFFU));
}

uint8_t clamp_u8(uint32_t value)
{
  return static_cast<uint8_t>(std::min<uint32_t>(value, 0xFFU));
}

uint32_t meters_to_mm_u32(double meters)
{
  if (meters <= 0.0 || !std::isfinite(meters)) {
    return 0U;
  }
  return static_cast<uint32_t>(std::llround(meters * 1000.0));
}

int16_t meters_to_mm_i16(double meters)
{
  if (!std::isfinite(meters)) {
    return 0;
  }
  const auto value = static_cast<int32_t>(std::llround(meters * 1000.0));
  return static_cast<int16_t>(std::clamp<int32_t>(value, -32768, 32767));
}

void write_u16_be(std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index, uint16_t value)
{
  payload[index++] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  payload[index++] = static_cast<uint8_t>(value & 0xFFU);
}

void write_i16_be(std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index, int16_t value)
{
  write_u16_be(payload, index, static_cast<uint16_t>(value));
}

void write_u32_be(std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index, uint32_t value)
{
  payload[index++] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
  payload[index++] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  payload[index++] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  payload[index++] = static_cast<uint8_t>(value & 0xFFU);
}

uint16_t read_u16_be(const std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index)
{
  const auto value = static_cast<uint16_t>(
    (static_cast<uint16_t>(payload[index]) << 8U) |
    static_cast<uint16_t>(payload[index + 1U]));
  index += 2U;
  return value;
}

int16_t read_i16_be(const std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index)
{
  return static_cast<int16_t>(read_u16_be(payload, index));
}

uint32_t read_u32_be(const std::array<uint8_t, kPusherPayloadSize> & payload, std::size_t & index)
{
  const auto value =
    (static_cast<uint32_t>(payload[index]) << 24U) |
    (static_cast<uint32_t>(payload[index + 1U]) << 16U) |
    (static_cast<uint32_t>(payload[index + 2U]) << 8U) |
    static_cast<uint32_t>(payload[index + 3U]);
  index += 4U;
  return value;
}

}  // namespace

std::array<uint8_t, kPusherPayloadSize> encode_payload(const PusherFrame & frame)
{
  std::array<uint8_t, kPusherPayloadSize> payload{};
  std::size_t index = 0U;
  payload[index++] = frame.path_count;
  payload[index++] = frame.path_id;
  write_u32_be(payload, index, frame.left_distance_mm);
  write_i16_be(payload, index, frame.lateral_offset_mm);
  write_u32_be(payload, index, frame.right_distance_mm);
  write_u16_be(payload, index, frame.travel_speed_mmps);
  write_u16_be(payload, index, frame.ultrasonic_work_distance_mm);
  write_u16_be(payload, index, frame.ultrasonic_adjust_distance_mm);
  payload[index++] = frame.turn_angle_deg;
  payload[index++] = static_cast<uint8_t>(frame.turn_type);
  write_u16_be(payload, index, frame.rotation_speed);
  write_u32_be(payload, index, frame.reserved);
  if (index != kPusherPayloadSize) {
    throw std::runtime_error("internal pusher payload size mismatch");
  }
  return payload;
}

PusherFrame decode_payload(const std::array<uint8_t, kPusherPayloadSize> & payload)
{
  PusherFrame frame;
  std::size_t index = 0U;
  frame.path_count = payload[index++];
  frame.path_id = payload[index++];
  frame.left_distance_mm = read_u32_be(payload, index);
  frame.lateral_offset_mm = read_i16_be(payload, index);
  frame.right_distance_mm = read_u32_be(payload, index);
  frame.travel_speed_mmps = read_u16_be(payload, index);
  frame.ultrasonic_work_distance_mm = read_u16_be(payload, index);
  frame.ultrasonic_adjust_distance_mm = read_u16_be(payload, index);
  frame.turn_angle_deg = payload[index++];
  frame.turn_type = static_cast<TurnType>(payload[index++]);
  frame.rotation_speed = read_u16_be(payload, index);
  frame.reserved = read_u32_be(payload, index);
  return frame;
}

std::string payload_to_hex(const std::array<uint8_t, kPusherPayloadSize> & payload)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < payload.size(); ++i) {
    stream << "0x" << std::setw(2) << static_cast<int>(payload[i]);
    if (i + 1U < payload.size()) {
      stream << ' ';
    }
  }
  return stream.str();
}

PusherFrame build_frame_from_segment(
  const CompiledRoute & route,
  const RouteSegment & segment,
  const SideDistanceEstimate & side_estimate,
  uint16_t travel_speed_mmps,
  uint16_t rotation_speed,
  uint16_t ultrasonic_work_distance_mm,
  uint16_t ultrasonic_adjust_distance_mm)
{
  PusherFrame frame;
  frame.path_count = clamp_u8(static_cast<uint32_t>(route.segments.size()));
  frame.path_id = clamp_u8(static_cast<uint32_t>(segment.id));
  frame.left_distance_mm = meters_to_mm_u32(route.target_left_distance_m);
  frame.lateral_offset_mm = meters_to_mm_i16(side_estimate.detected ? side_estimate.active_offset_m : 0.0);
  frame.right_distance_mm = meters_to_mm_u32(route.target_right_distance_m);
  frame.travel_speed_mmps = travel_speed_mmps;
  frame.ultrasonic_work_distance_mm = ultrasonic_work_distance_mm;
  frame.ultrasonic_adjust_distance_mm = ultrasonic_adjust_distance_mm;
  frame.turn_angle_deg = clamp_u8(
    static_cast<uint32_t>(std::llround(std::fabs(segment.turn_angle_deg))));
  frame.turn_type = segment.turn_type;
  frame.rotation_speed = clamp_u16(rotation_speed);
  frame.reserved = side_estimate.detected ? 0U : 1U;
  return frame;
}

}  // namespace pusher_nav_bridge
