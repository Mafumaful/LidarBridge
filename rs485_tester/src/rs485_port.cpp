#include "rs485_tester/rs485_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#endif

namespace rs485_tester
{

namespace
{

[[noreturn]] void throw_system_error(const std::string & context)
{
  throw std::runtime_error(context + ": " + std::strerror(errno));
}

tcflag_t data_bits_flag(int data_bits)
{
  switch (data_bits) {
    case 5:
      return CS5;
    case 6:
      return CS6;
    case 7:
      return CS7;
    case 8:
      return CS8;
    default:
      throw std::runtime_error("Unsupported data bits value: " + std::to_string(data_bits));
  }
}

speed_t to_baud_constant(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate));
  }
}

}  // namespace

Rs485Port::~Rs485Port()
{
  close_port();
}

void Rs485Port::open_port(const Rs485Config & config)
{
  if (is_open()) {
    close_port();
  }

  fd_ = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    throw_system_error("Failed to open serial device " + config.device);
  }

  try {
    configure_termios(fd_, config);
    configure_rs485_mode(fd_, config);
  } catch (...) {
    close_port();
    throw;
  }
}

void Rs485Port::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Rs485Port::is_open() const
{
  return fd_ >= 0;
}

void Rs485Port::write_bytes(const std::vector<uint8_t> & data)
{
  if (!is_open()) {
    throw std::runtime_error("Serial port is not open");
  }

  const uint8_t * buffer = data.data();
  std::size_t remaining = data.size();

  while (remaining > 0) {
    const auto written = ::write(fd_, buffer, remaining);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw_system_error("Failed to write to serial device");
    }
    remaining -= static_cast<std::size_t>(written);
    buffer += written;
  }

  if (::tcdrain(fd_) != 0) {
    throw_system_error("Failed to drain serial device");
  }
}

void Rs485Port::configure_termios(int fd, const Rs485Config & config)
{
  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) {
    throw_system_error("Failed to get serial attributes");
  }

  ::cfmakeraw(&tty);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= data_bits_flag(config.data_bits);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;

  if (config.parity == "none") {
    tty.c_cflag &= ~PARENB;
  } else if (config.parity == "even") {
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;
  } else if (config.parity == "odd") {
    tty.c_cflag |= PARENB;
    tty.c_cflag |= PARODD;
  } else {
    throw std::runtime_error("Unsupported parity: " + config.parity);
  }

  if (config.stop_bits == 1) {
    tty.c_cflag &= ~CSTOPB;
  } else if (config.stop_bits == 2) {
    tty.c_cflag |= CSTOPB;
  } else {
    throw std::runtime_error("Unsupported stop bits value: " + std::to_string(config.stop_bits));
  }

  if (config.flow_control == "none") {
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  } else if (config.flow_control == "software") {
    tty.c_iflag |= IXON | IXOFF;
  } else if (config.flow_control == "hardware") {
    tty.c_cflag |= CRTSCTS;
  } else {
    throw std::runtime_error("Unsupported flow control: " + config.flow_control);
  }

  const auto baud = to_baud_constant(config.baud_rate);
  if (::cfsetispeed(&tty, baud) != 0 || ::cfsetospeed(&tty, baud) != 0) {
    throw_system_error("Failed to set baud rate");
  }

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    throw_system_error("Failed to apply serial attributes");
  }
}

void Rs485Port::configure_rs485_mode(int fd, const Rs485Config & config)
{
  (void)fd;
  (void)config;

#ifdef __linux__
  if (!config.enable_rs485_mode) {
    return;
  }

  serial_rs485 rs485conf{};
  rs485conf.flags |= SER_RS485_ENABLED;
  if (config.rs485_rx_during_tx) {
    rs485conf.flags |= SER_RS485_RX_DURING_TX;
  }
  rs485conf.delay_rts_before_send = config.rs485_delay_before_send_us;
  rs485conf.delay_rts_after_send = config.rs485_delay_after_send_us;

  if (::ioctl(fd, TIOCSRS485, &rs485conf) != 0) {
    throw_system_error("Failed to enable RS485 mode");
  }
#else
  if (config.enable_rs485_mode) {
    throw std::runtime_error("RS485 hardware mode is only supported on Linux");
  }
#endif
}

}  // namespace rs485_tester
