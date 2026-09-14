// Copyright 2026
// Chassis communication node for the WHEELTEC STM32 mobile base.
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace
{
constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr std::size_t kCommandSize = 11;
constexpr std::size_t kFeedbackSize = 24;

uint8_t xor_checksum(const uint8_t * data, std::size_t size)
{
  uint8_t result = 0;
  for (std::size_t i = 0; i < size; ++i) {
    result ^= data[i];
  }
  return result;
}

int16_t read_i16_be(const uint8_t high, const uint8_t low)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}

int16_t velocity_to_wire(const double value)
{
  const double scaled = std::round(value * 1000.0);
  return static_cast<int16_t>(std::clamp(scaled, -32768.0, 32767.0));
}

void put_i16_be(std::array<uint8_t, kCommandSize> & frame, std::size_t offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  frame[offset] = static_cast<uint8_t>(raw >> 8U);
  frame[offset + 1] = static_cast<uint8_t>(raw & 0xFFU);
}
}  // namespace

class ChassisCommunicationNode : public rclcpp::Node
{
public:
  ChassisCommunicationNode()
  : Node("chassis_communication")
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM1");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);

    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("car/actual_velocity", 10);
    imu_raw_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("car/imu_raw", 10);
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("car/battery_voltage", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) 
      { send_command(*msg);});

    if (!open_serial()) {
      throw std::runtime_error("Could not open serial device " + device_);
    }
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5), [this]() {read_serial();});
  //   test_timer_ = create_wall_timer(
  //     std::chrono::milliseconds(100),
  //     [this]() {
  //       geometry_msgs::msg::Twist test_command;
  //       test_command.linear.x = 0.0;
  //       test_command.linear.y = 0.0;
  //       test_command.angular.z = 0.0;
  //       send_command(test_command);
  //       RCLCPP_WARN(get_logger(), "Test mode: sending linear.x=%f m/s at 10 Hz", static_cast<double>(test_command.linear.x));
  //     });
  //   RCLCPP_INFO(get_logger(), "Serial car interface ready: %s at %d baud", device_.c_str(), baud_rate_);
  }

  ~ChassisCommunicationNode() override
  {
    if (serial_fd_ >= 0) {
      geometry_msgs::msg::Twist stop;
      send_command(stop);
      close(serial_fd_);
    }
  }

private:
  bool open_serial()
  {
    serial_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", device_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    speed_t speed;
    switch (baud_rate_) {
      case 9600: speed = B9600; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default:
        RCLCPP_ERROR(get_logger(), "Unsupported baud rate: %d", baud_rate_);
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcflush(serial_fd_, TCIOFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    return true;
  }

  void send_command(const geometry_msgs::msg::Twist & cmd)
  {
    if (serial_fd_ < 0) {
      return;
    }
    std::array<uint8_t, kCommandSize> frame{};
    frame[0] = kFrameHeader;
    frame[1] = 0x00;  // Normal ROS velocity-control mode.
    frame[2] = 0x00;  // Reserved.
    put_i16_be(frame, 3, velocity_to_wire(cmd.linear.x));
    put_i16_be(frame, 5, velocity_to_wire(cmd.linear.y));
    put_i16_be(frame, 7, velocity_to_wire(cmd.angular.z));
    frame[9] = xor_checksum(frame.data(), 9);
    frame[10] = kFrameTail;

    std::size_t sent = 0;
    while (sent < frame.size()) {
      const ssize_t count = write(serial_fd_, frame.data() + sent, frame.size() - sent);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EAGAIN && errno != EINTR) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Serial write failed: %s", std::strerror(errno));
        return;
      } else {
        break;
      }
    }
  }

  void read_serial()
  {
    std::array<uint8_t, 256> data{};
    for (;;) {
      const ssize_t count = read(serial_fd_, data.data(), data.size());
      if (count > 0) {
        rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.begin() + count);
      } else {
        if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Serial read failed: %s", std::strerror(errno));
        }
        break;
      }
    }
    parse_feedback();
  }

  void parse_feedback()//解析数据并发布话题
  {
    while (rx_buffer_.size() >= kFeedbackSize) {
      const auto header = std::find(rx_buffer_.begin(), rx_buffer_.end(), kFrameHeader);
      rx_buffer_.erase(rx_buffer_.begin(), header);
      if (rx_buffer_.size() < kFeedbackSize) {
        return;
      }
      if (rx_buffer_[23] != kFrameTail || xor_checksum(rx_buffer_.data(), 22) != rx_buffer_[22]) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = read_i16_be(rx_buffer_[2], rx_buffer_[3]) / 1000.0;
      velocity.linear.y = read_i16_be(rx_buffer_[4], rx_buffer_[5]) / 1000.0;
      velocity.angular.z = read_i16_be(rx_buffer_[6], rx_buffer_[7]) / 1000.0;
      velocity_pub_->publish(velocity);
      
      //打印接收到的x,y,z的速度
      RCLCPP_INFO(get_logger(), "Received command: linear.x=%f, linear.y=%f, angular.z=%f",
      static_cast<double>(velocity.linear.x),
      static_cast<double>(velocity.linear.y),
      static_cast<double>(velocity.angular.z));

      std_msgs::msg::Int16MultiArray imu;
      imu.data = {
        read_i16_be(rx_buffer_[8], rx_buffer_[9]),
        read_i16_be(rx_buffer_[10], rx_buffer_[11]),
        read_i16_be(rx_buffer_[12], rx_buffer_[13]),
        read_i16_be(rx_buffer_[14], rx_buffer_[15]),
        read_i16_be(rx_buffer_[16], rx_buffer_[17]),
        read_i16_be(rx_buffer_[18], rx_buffer_[19])};
      imu_raw_pub_->publish(imu);

      std_msgs::msg::Float32 voltage;
      voltage.data = read_i16_be(rx_buffer_[20], rx_buffer_[21]) / 1000.0F;
      voltage_pub_->publish(voltage);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kFeedbackSize);
    }
  }

  std::string device_;
  int baud_rate_{115200};
  int serial_fd_{-1};
  std::vector<uint8_t> rx_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ChassisCommunicationNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_communication"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
