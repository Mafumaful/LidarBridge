#include "rs485_tester/rs485_port.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rs485_tester
{

namespace
{

std::string trim(std::string value)
{
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

uint16_t parse_u16_token(const std::string & token)
{
  const auto clean = trim(token);
  if (clean.empty()) {
    throw std::runtime_error("Encountered empty value in register list");
  }

  size_t parsed = 0;
  const auto value = std::stoul(clean, &parsed, 0);
  if (parsed != clean.size() || value > 0xFFFFU) {
    throw std::runtime_error("Invalid 16-bit value: " + clean);
  }
  return static_cast<uint16_t>(value);
}

std::vector<uint16_t> parse_u16_list(const std::string & input)
{
  std::vector<uint16_t> values;
  std::stringstream ss(input);
  std::string token;

  while (std::getline(ss, token, ',')) {
    token = trim(token);
    if (!token.empty()) {
      values.push_back(parse_u16_token(token));
    }
  }

  return values;
}

uint16_t modbus_crc16(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFFU;
  for (const auto byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      const bool lsb_set = (crc & 0x0001U) != 0U;
      crc >>= 1U;
      if (lsb_set) {
        crc ^= 0xA001U;
      }
    }
  }
  return crc;
}

void append_u16_be(std::vector<uint8_t> & frame, uint16_t value)
{
  frame.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  frame.push_back(static_cast<uint8_t>(value & 0xFFU));
}

std::vector<uint8_t> build_modbus_request(
  uint8_t slave_id,
  const std::string & request_type,
  uint16_t start_address,
  uint16_t register_count,
  uint16_t write_value,
  const std::vector<uint16_t> & write_values)
{
  std::vector<uint8_t> frame;

  if (request_type == "read_holding_registers") {
    if (register_count == 0U || register_count > 125U) {
      throw std::runtime_error("register_count for read_holding_registers must be in [1, 125]");
    }
    frame.push_back(slave_id);
    frame.push_back(0x03U);
    append_u16_be(frame, start_address);
    append_u16_be(frame, register_count);
  } else if (request_type == "write_single_register") {
    frame.push_back(slave_id);
    frame.push_back(0x06U);
    append_u16_be(frame, start_address);
    append_u16_be(frame, write_value);
  } else if (request_type == "write_multiple_registers") {
    if (write_values.empty() || write_values.size() > 123U) {
      throw std::runtime_error("write_values for write_multiple_registers must contain 1 to 123 values");
    }
    frame.push_back(slave_id);
    frame.push_back(0x10U);
    append_u16_be(frame, start_address);
    append_u16_be(frame, static_cast<uint16_t>(write_values.size()));
    frame.push_back(static_cast<uint8_t>(write_values.size() * 2U));
    for (const auto value : write_values) {
      append_u16_be(frame, value);
    }
  } else {
    throw std::runtime_error(
            "Unsupported request_type: " + request_type +
            ". Expected read_holding_registers, write_single_register, or write_multiple_registers");
  }

  const auto crc = modbus_crc16(frame);
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0xFFU));
  return frame;
}

std::string request_summary(
  const std::string & request_type,
  uint8_t slave_id,
  uint16_t start_address,
  uint16_t register_count,
  uint16_t write_value,
  const std::vector<uint16_t> & write_values)
{
  std::ostringstream oss;
  oss << "slave_id=" << static_cast<int>(slave_id)
      << ", request_type=" << request_type
      << ", start_address=0x" << std::hex << std::setw(4) << std::setfill('0') << start_address;

  if (request_type == "read_holding_registers") {
    oss << ", register_count=" << std::dec << register_count;
  } else if (request_type == "write_single_register") {
    oss << ", write_value=0x" << std::hex << std::setw(4) << std::setfill('0') << write_value;
  } else if (request_type == "write_multiple_registers") {
    oss << ", write_values=";
    for (std::size_t i = 0; i < write_values.size(); ++i) {
      oss << (i == 0 ? "" : ",") << "0x"
          << std::hex << std::setw(4) << std::setfill('0') << write_values[i];
    }
  }

  return oss.str();
}

std::string bytes_to_hex(const std::vector<uint8_t> & data)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < data.size(); ++i) {
    oss << "0x" << std::setw(2) << static_cast<int>(data[i]);
    if (i + 1 < data.size()) {
      oss << ' ';
    }
  }
  return oss.str();
}

}  // namespace

class Rs485TestNode : public rclcpp::Node
{
public:
  Rs485TestNode()
  : Node("rs485_test_node")
  {
    declare_parameter<std::string>("device", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<int>("data_bits", 8);
    declare_parameter<std::string>("parity", "none");
    declare_parameter<int>("stop_bits", 1);
    declare_parameter<std::string>("flow_control", "none");
    declare_parameter<bool>("enable_rs485_mode", false);
    declare_parameter<int>("rs485_delay_before_send_us", 0);
    declare_parameter<int>("rs485_delay_after_send_us", 0);
    declare_parameter<bool>("rs485_rx_during_tx", false);
    declare_parameter<int>("send_interval_ms", 1000);
    declare_parameter<int>("slave_id", 1);
    declare_parameter<std::string>("request_type", "read_holding_registers");
    declare_parameter<int>("start_address", 0);
    declare_parameter<int>("register_count", 2);
    declare_parameter<int>("write_value", 1);
    declare_parameter<std::string>("write_values", "0x0001,0x0002");

    Rs485Config config;
    config.device = get_parameter("device").as_string();
    config.baud_rate = static_cast<int>(get_parameter("baud_rate").as_int());
    config.data_bits = static_cast<int>(get_parameter("data_bits").as_int());
    config.parity = get_parameter("parity").as_string();
    config.stop_bits = static_cast<int>(get_parameter("stop_bits").as_int());
    config.flow_control = get_parameter("flow_control").as_string();
    config.enable_rs485_mode = get_parameter("enable_rs485_mode").as_bool();
    config.rs485_delay_before_send_us = static_cast<int>(get_parameter("rs485_delay_before_send_us").as_int());
    config.rs485_delay_after_send_us = static_cast<int>(get_parameter("rs485_delay_after_send_us").as_int());
    config.rs485_rx_during_tx = get_parameter("rs485_rx_during_tx").as_bool();

    const auto slave_id = static_cast<int>(get_parameter("slave_id").as_int());
    const auto request_type = get_parameter("request_type").as_string();
    const auto start_address = static_cast<int>(get_parameter("start_address").as_int());
    const auto register_count = static_cast<int>(get_parameter("register_count").as_int());
    const auto write_value = static_cast<int>(get_parameter("write_value").as_int());
    const auto write_values = parse_u16_list(get_parameter("write_values").as_string());

    if (slave_id < 0 || slave_id > 247) {
      throw std::runtime_error("slave_id must be in [0, 247]");
    }
    if (start_address < 0 || start_address > 0xFFFF) {
      throw std::runtime_error("start_address must be in [0, 65535]");
    }
    if (register_count < 0 || register_count > 0xFFFF) {
      throw std::runtime_error("register_count must be in [0, 65535]");
    }
    if (write_value < 0 || write_value > 0xFFFF) {
      throw std::runtime_error("write_value must be in [0, 65535]");
    }

    payload_ = build_modbus_request(
      static_cast<uint8_t>(slave_id),
      request_type,
      static_cast<uint16_t>(start_address),
      static_cast<uint16_t>(register_count),
      static_cast<uint16_t>(write_value),
      write_values);
    interval_ = std::chrono::milliseconds(
      static_cast<int>(get_parameter("send_interval_ms").as_int()));
    request_summary_ = request_summary(
      request_type,
      static_cast<uint8_t>(slave_id),
      static_cast<uint16_t>(start_address),
      static_cast<uint16_t>(register_count),
      static_cast<uint16_t>(write_value),
      write_values);

    port_.open_port(config);
    RCLCPP_INFO(
      get_logger(),
      "Opened %s, %s, frame=%s, interval=%lld ms",
      config.device.c_str(),
      request_summary_.c_str(),
      bytes_to_hex(payload_).c_str(),
      static_cast<long long>(interval_.count()));

    timer_ = create_wall_timer(interval_, std::bind(&Rs485TestNode::send_payload, this));
  }

private:
  void send_payload()
  {
    try {
      port_.write_bytes(payload_);
      RCLCPP_INFO(get_logger(), "Sent %zu bytes: %s", payload_.size(), bytes_to_hex(payload_).c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Send failed: %s", ex.what());
    }
  }

  Rs485Port port_;
  std::vector<uint8_t> payload_;
  std::string request_summary_;
  std::chrono::milliseconds interval_{1000};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rs485_tester

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rs485_tester::Rs485TestNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("rs485_test_node"), "Fatal error: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
