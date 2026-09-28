// Copyright 2026
// Terminal keyboard teleoperation for the chassis command input.
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_input");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
    key_timeout_ = declare_parameter<double>("key_timeout", 0.7);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    if (linear_speed_ <= 0.0 || key_timeout_ <= 0.0 || publish_rate_ <= 0.0) {
      throw std::invalid_argument("linear_speed, key_timeout and publish_rate must be positive");
    }

    configure_terminal();
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {update();});

    RCLCPP_INFO(
      get_logger(),
      "Keyboard control ready: arrows/WASD move, space stops, Q quits; publishing to %s",
      output_topic_.c_str());
  }

  ~KeyboardTeleop() override
  {
    if (command_pub_) {
      command_pub_->publish(geometry_msgs::msg::Twist{});
    }
    restore_terminal();
  }

private:
  void configure_terminal()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("keyboard_teleop must be run from an interactive terminal");
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      throw std::runtime_error("failed to read terminal settings");
    }
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0) {
      throw std::runtime_error("failed to read terminal flags");
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("failed to configure terminal for keyboard input");
    }
    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      throw std::runtime_error("failed to configure terminal for keyboard input");
    }
    terminal_configured_ = true;
  }

  void restore_terminal()
  {
    if (!terminal_configured_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    terminal_configured_ = false;
  }

  void update()
  {
    char key;
    errno = 0;
    while (read(STDIN_FILENO, &key, 1) == 1) {
      process_key(key);
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to read keyboard input");
    }

    if (moving_ && (get_clock()->now() - last_key_time_).seconds() > key_timeout_) {
      command_ = geometry_msgs::msg::Twist{};
      moving_ = false;
    }
    command_pub_->publish(command_);
  }

  void process_key(char key)
  {
    if (escape_state_ == 1) {
      escape_state_ = key == '[' ? 2 : 0;
      return;
    }
    if (escape_state_ == 2) {
      escape_state_ = 0;
      switch (key) {
        case 'A': set_motion(linear_speed_, 0.0); return;
        case 'B': set_motion(-linear_speed_, 0.0); return;
        case 'C': set_motion(0.0, -linear_speed_); return;
        case 'D': set_motion(0.0, linear_speed_); return;
        default: return;
      }
    }
    if (key == '\x1b') {
      escape_state_ = 1;
      return;
    }

    switch (key) {
      case 'w':
      case 'W': set_motion(linear_speed_, 0.0); break;
      case 's':
      case 'S': set_motion(-linear_speed_, 0.0); break;
      case 'a':
      case 'A': set_motion(0.0, linear_speed_); break;
      case 'd':
      case 'D': set_motion(0.0, -linear_speed_); break;
      case ' ':
        command_ = geometry_msgs::msg::Twist{};
        moving_ = false;
        break;
      case 'q':
      case 'Q':
        command_pub_->publish(geometry_msgs::msg::Twist{});
        rclcpp::shutdown();
        break;
      default: break;
    }
  }

  void set_motion(double linear_x, double linear_y)
  {
    command_ = geometry_msgs::msg::Twist{};
    command_.linear.x = linear_x;
    command_.linear.y = linear_y;
    last_key_time_ = get_clock()->now();
    moving_ = true;
  }

  std::string output_topic_;
  double linear_speed_{0.2};
  double key_timeout_{0.7};
  double publish_rate_{20.0};
  geometry_msgs::msg::Twist command_;
  rclcpp::Time last_key_time_{0, 0, RCL_ROS_TIME};
  int escape_state_{0};
  int original_flags_{0};
  bool moving_{false};
  bool terminal_configured_{false};
  termios original_termios_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_teleop"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
