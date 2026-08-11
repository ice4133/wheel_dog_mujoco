#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "wheel_dog_mujoco/actuator/actuator_manager.h"
#include "wheel_dog_mujoco/body/body_manager.h"
#include "wheel_dog_mujoco/driver/drv_dds.h"
#include "wheel_dog_mujoco/skill/stand_skill.h"

namespace wheel_dog_mujoco
{

// Runtime composition adapter. Motion policy belongs to StandSkill; body,
// actuator and transport calculations remain in their respective managers.
class BodyNode : public rclcpp::Node
{
public:
  explicit BodyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool LieDownBeforeShutdown();

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  void OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void OnControlTimer();
  bool RefreshFeedback();
  body::BodySensorFrame DecodeBodySensors(const driver::RobotFeedback & feedback) const;
  bool SendSkillCommand(SteadyTimePoint now, double elapsed_seconds);
  bool SendBodyCommand(
    const body::BodyCommandFrame & command, SteadyTimePoint now,
    double elapsed_seconds);
  void SendDisabledFallback(SteadyTimePoint now);
  void LogSkillPhase();
  void LogBodyResult(const body::ManagerOutput & output);
  void LogActuatorResult(
    const actuator::ManagerOutput & output,
    const actuator::JointStateFrame & state);

  body::BodySensorFrame body_sensor_frame_{};
  body::ManagerOutput body_output_{};
  actuator::JointStateFrame joint_state_frame_{};
  driver::RobotCommand robot_command_{};
  std::unique_ptr<body::BodyManager> body_manager_;
  std::unique_ptr<actuator::ActuatorManager> actuator_manager_;
  std::unique_ptr<skill::StandSkill> stand_skill_;
  std::unique_ptr<driver::DrvDds> driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double shutdown_timeout_seconds_{4.0};
  double control_period_seconds_{0.002};
  double state_timeout_seconds_{0.2};
  bool contact_force_feedback_available_{false};
  bool has_low_state_{false};
  bool phase_logged_{false};
  std::uint64_t fallback_sequence_{0};
  SteadyTimePoint last_feedback_time_{};
  SteadyTimePoint last_control_time_{};
  skill::StandSkill::Phase last_logged_phase_{
    skill::StandSkill::Phase::kWaitingForState};
};

}  // namespace wheel_dog_mujoco
