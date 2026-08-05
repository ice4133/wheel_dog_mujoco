#pragma once

#include <termios.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"

namespace wheel_dog_mujoco
{

class KeyboardController : public rclcpp::Node
{
public:
  explicit KeyboardController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~KeyboardController() override;

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  void ConfigureTerminal();
  void RestoreTerminal() noexcept;
  void PollKeyboard();
  void HandleKey(char key);
  void PrintHelp() const;
  void OnPublishTimer();
  void SetMotion(double linear_scale, double angular_scale);
  void StopMotion();
  void SelectSpeedLevel(std::size_t level_index);
  void PublishCommand();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  geometry_msgs::msg::Twist velocity_command_{};

  std::array<double, 3> linear_speed_levels_{{0.15, 0.30, 0.50}};
  std::array<double, 3> angular_speed_levels_{{0.5, 1.0, 1.5}};
  std::size_t speed_level_index_{0};
  double curve_turn_ratio_{0.65};
  double publish_period_seconds_{0.02};
  double key_timeout_seconds_{0.65};

  bool motion_key_active_{false};
  bool exit_requested_{false};
  SteadyTimePoint last_motion_key_time_{};

  termios original_terminal_{};
  int original_stdin_flags_{-1};
  bool terminal_configured_{false};
};

}  // namespace wheel_dog_mujoco
