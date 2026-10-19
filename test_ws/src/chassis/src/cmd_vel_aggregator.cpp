// Copyright 2026
// Relays chassis velocity commands to the hardware interface with a timeout.
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelAggregator : public rclcpp::Node
{
public:
  CmdVelAggregator()
  : Node("cmd_vel_aggregator")
  {
    base_command_topic_ =
      declare_parameter<std::string>("base_command_topic", "/cmd_vel_input");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    base_command_timeout_ = declare_parameter<double>("base_command_timeout", 0.5);

    if (publish_rate_ <= 0.0 || base_command_timeout_ < 0.0) {
      throw std::invalid_argument(
              "publish_rate must be positive and base_command_timeout must be non-negative");
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    base_command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      base_command_topic_, 10,
      std::bind(&CmdVelAggregator::base_command_callback, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelAggregator::publish_command, this));

    RCLCPP_INFO(
      get_logger(), "Relaying velocity commands from %s to %s",
      base_command_topic_.c_str(), output_topic_.c_str());
  }

  ~CmdVelAggregator() override
  {
    if (output_pub_) {
      output_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

private:
  void base_command_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    base_command_ = *msg;
    last_base_command_time_ = get_clock()->now();
    has_base_command_ = true;
  }

  void publish_command()
  {
    const rclcpp::Time now = get_clock()->now();
    geometry_msgs::msg::Twist output;

    if (
      has_base_command_ &&
      (now - last_base_command_time_).seconds() <= base_command_timeout_)
    {
      output = base_command_;
    }

    output_pub_->publish(output);
  }

  std::string base_command_topic_;
  std::string output_topic_;
  double publish_rate_{50.0};
  double base_command_timeout_{0.5};
  bool has_base_command_{false};
  geometry_msgs::msg::Twist base_command_;
  rclcpp::Time last_base_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CmdVelAggregator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_aggregator"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
