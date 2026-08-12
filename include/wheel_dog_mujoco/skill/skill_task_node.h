#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/string.hpp"
#include "wheel_dog_mujoco/actuator/actuator_manager.h"
#include "wheel_dog_mujoco/driver/drv_dds.h"
#include "wheel_dog_mujoco/skill/skill_task.h"
#include "wheel_dog_mujoco/skill/stand_skill.h"

namespace wheel_dog_mujoco::skill
{

// Runtime executor and sole /lowcmd owner. SkillTask selects an action;
// SkillTaskNode calculates its trajectory and routes it to ActuatorManager.
class SkillTaskNode : public rclcpp::Node
{
public:
  explicit SkillTaskNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool ReturnToIdleBeforeShutdown();

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;
  using LegJointPositions = StandSkill::LegJointPositions;

  void OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void OnSkillRequest(const std_msgs::msg::String::SharedPtr message);
  void OnControlTimer();
  bool RefreshFeedback();
  bool SendTaskCommand(SteadyTimePoint now, double elapsed_seconds);
  actuator::JointCommandFrame ExecuteSelectedAction(
    SkillTask::Action action, SteadyTimePoint now, double elapsed_seconds);
  actuator::JointCommandFrame MakeDampingCommand(SteadyTimePoint now);
  actuator::JointCommandFrame MakePostureCommand(
    const LegJointPositions & target, double duration, bool allow_wheel_motion,
    SteadyTimePoint now, double elapsed_seconds);
  actuator::JointCommandFrame MakeJointCommand(
    const LegJointPositions & leg_positions, bool allow_wheel_motion,
    SteadyTimePoint now, double elapsed_seconds);
  LegJointPositions ReadLegPositions() const noexcept;
  void OnActionChanged(SkillTask::Action action);
  void ResetExecutionState() noexcept;
  void LogTaskOutput(const SkillTask::Output & output);
  void LogActuatorResult(
    const actuator::ManagerOutput & output,
    const actuator::JointStateFrame & state);
  void PublishTaskState();
  static LegJointPositions Interpolate(
    const LegJointPositions & from, const LegJointPositions & to,
    double ratio) noexcept;
  static double SmoothStep(double ratio) noexcept;

  actuator::JointStateFrame joint_state_frame_{};
  driver::RobotCommand robot_command_{};
  std::unique_ptr<actuator::ActuatorManager> actuator_manager_;
  std::unique_ptr<StandSkill> stand_skill_;
  std::unique_ptr<SkillTask> skill_task_;
  std::unique_ptr<driver::DrvDds> driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr skill_request_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr skill_state_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  SkillTask::ActionResult last_action_result_{};
  SkillTask::Action last_executed_action_{SkillTask::Action::kDamping};
  LegJointPositions transition_start_pose_{};
  LegJointPositions desired_leg_pose_{};
  double shutdown_timeout_seconds_{4.0};
  double control_period_seconds_{0.002};
  double state_timeout_seconds_{0.2};
  double action_elapsed_seconds_{0.0};
  double desired_linear_velocity_{0.0};
  double desired_yaw_velocity_{0.0};
  double right_wheel_velocity_{0.0};
  double left_wheel_velocity_{0.0};
  bool has_low_state_{false};
  bool has_executed_action_{false};
  bool has_velocity_command_{false};
  std::uint64_t command_sequence_{0};
  SteadyTimePoint last_feedback_time_{};
  SteadyTimePoint last_control_time_{};
  SteadyTimePoint last_velocity_command_at_{};
};

}  // namespace wheel_dog_mujoco::skill
