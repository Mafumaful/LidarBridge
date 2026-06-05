#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rs485_tester
{

struct Rs485Config
{
  std::string device{"/dev/ttyUSB0"};
  int baud_rate{115200};
  int data_bits{8};
  std::string parity{"none"};
  int stop_bits{1};
  std::string flow_control{"none"};
  bool enable_rs485_mode{false};
  int rs485_delay_before_send_us{0};
  int rs485_delay_after_send_us{0};
  bool rs485_rx_during_tx{false};
};

class Rs485Port
{
public:
  Rs485Port() = default;
  ~Rs485Port();

  Rs485Port(const Rs485Port &) = delete;
  Rs485Port & operator=(const Rs485Port &) = delete;

  void open_port(const Rs485Config & config);
  void close_port();
  bool is_open() const;
  void write_bytes(const std::vector<uint8_t> & data);

private:
  int fd_{-1};

  static void configure_termios(int fd, const Rs485Config & config);
  static void configure_rs485_mode(int fd, const Rs485Config & config);
};

}  // namespace rs485_tester
