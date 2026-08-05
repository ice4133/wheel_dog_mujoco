#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "wheel_dog_mujoco/stand_controller.h"

namespace wheel_dog_mujoco
{

class StandNode : public rclcpp::Node
{
public:
  explicit StandNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  StandController::Config LoadControllerConfig();
  static StandController::JointPositions ToJointPositions(
    const std::vector<double> & values, const std::string & parameter_name);

  void InitializeCommand();
  void OnLowState(const unitree_go::msg::LowState::SharedPtr message);
  void OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void OnControlTimer();
  void UpdateWheelSpeeds(double elapsed_seconds, bool driving_enabled);
  void ApplyControllerCommand();
  void LogStateTransition(StandController::State state);
  static double Approach(double current, double target, double max_change) noexcept;

  StandController controller_{};
  unitree_go::msg::LowCmd low_command_{};
  unitree_go::msg::LowState low_state_{};

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr command_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double desired_linear_velocity_{0.0};
  double desired_yaw_velocity_{0.0};
  double current_right_wheel_speed_{0.0};
  double current_left_wheel_speed_{0.0};
  double wheel_radius_{0.086};
  double track_width_{0.284};
  double max_wheel_speed_{6.0};
  double wheel_acceleration_{12.0};
  bool has_low_state_{false};
  bool has_velocity_command_{false};
  bool controller_started_{false};
  double control_period_seconds_{0.002};
  double startup_delay_seconds_{1.0};
  double state_timeout_seconds_{0.2};
  double velocity_timeout_seconds_{0.25};
  SteadyTimePoint first_state_time_{};
  SteadyTimePoint last_state_time_{};
  SteadyTimePoint last_velocity_command_time_{};
  SteadyTimePoint last_control_time_{};
  StandController::State last_logged_state_{StandController::State::kIdle};
};

}  // namespace wheel_dog_mujoco
