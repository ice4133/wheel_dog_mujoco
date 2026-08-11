#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "wheel_dog_mujoco/actuator/actuator_manager.h"
#include "wheel_dog_mujoco/driver/drv_dds.h"
#include "wheel_dog_mujoco/stand_controller.h"

namespace wheel_dog_mujoco
{

class StandNode : public rclcpp::Node
{
public:
  explicit StandNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool LieDownBeforeShutdown();

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  StandController::Config LoadControllerConfig();
  static StandController::JointPositions ToJointPositions(
    const std::vector<double> & values, const std::string & parameter_name);

  void OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void OnControlTimer();
  void UpdateWheelSpeeds(double elapsed_seconds, bool driving_enabled);
  bool RefreshFeedback();
  void BuildJointCommand(SteadyTimePoint now);
  bool SendActuatorCommand(SteadyTimePoint now, double elapsed_seconds);
  void LogStateTransition(StandController::State state);
  static double Approach(double current, double target, double max_change) noexcept;

  StandController controller_{};
  actuator::JointCommandFrame joint_command_frame_{};
  actuator::JointStateFrame joint_state_frame_{};
  driver::RobotCommand robot_command_{};
  std::unique_ptr<actuator::ActuatorManager> actuator_manager_;
  std::unique_ptr<driver::DrvDds> driver_;

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
  std::uint64_t command_sequence_{0};
  double control_period_seconds_{0.002};
  double startup_delay_seconds_{1.0};
  double state_timeout_seconds_{0.2};
  double velocity_timeout_seconds_{0.25};
  SteadyTimePoint first_state_time_{};
  SteadyTimePoint last_feedback_time_{};
  SteadyTimePoint last_velocity_command_time_{};
  SteadyTimePoint last_control_time_{};
  StandController::State last_logged_state_{StandController::State::kIdle};
};

}  // namespace wheel_dog_mujoco
