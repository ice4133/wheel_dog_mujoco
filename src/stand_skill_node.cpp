#include "wheel_dog_mujoco/stand_skill_node.h"

#include <algorithm>
#include <csignal>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheel_dog_mujoco
{
namespace
{

skill::StandSkill::LegJointPositions ConvertPose(
  const std::vector<double> & values, const char * parameter_name)
{
  if (values.size() != skill::StandSkill::kLegJointCount ||
    !std::all_of(
      values.begin(), values.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument(
            std::string(parameter_name) + " must contain 12 finite joint angles");
  }
  skill::StandSkill::LegJointPositions pose{};
  std::copy(values.begin(), values.end(), pose.begin());
  return pose;
}

}  // namespace

StandSkillNode::StandSkillNode(const rclcpp::NodeOptions & options)
: Node("go2w_stand_skill", options)
{
  control_period_seconds_ = declare_parameter<double>("control_period", 0.002);
  state_timeout_seconds_ = declare_parameter<double>("state_timeout", 0.2);
  shutdown_timeout_seconds_ = declare_parameter<double>("shutdown_timeout", 4.0);
  const auto command_topic = declare_parameter<std::string>("command_topic", "/lowcmd");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/lowstate");
  const auto velocity_topic = declare_parameter<std::string>("velocity_topic", "/cmd_vel");
  const auto default_config =
    ament_index_cpp::get_package_share_directory("wheel_dog_mujoco") + "/config.yaml";
  const auto config_path = declare_parameter<std::string>("config_path", default_config);
  if (control_period_seconds_ <= 0.0 || state_timeout_seconds_ <= 0.0 ||
    shutdown_timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument("StandSkillNode timing parameters are invalid");
  }

  skill::StandSkill::Config skill_config;
  skill_config.crouch_pose = ConvertPose(
    declare_parameter<std::vector<double>>(
      "crouch_pose",
      {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
        -0.2, 1.36, -2.65, 0.2, 1.36, -2.65}),
    "crouch_pose");
  skill_config.stand_pose = ConvertPose(
    declare_parameter<std::vector<double>>(
      "stand_pose",
      {0.0, 0.67, -1.30, 0.0, 0.67, -1.30,
        0.0, 0.67, -1.30, 0.0, 0.67, -1.30}),
    "stand_pose");
  skill_config.crouch_duration = declare_parameter<double>("crouch_duration", 1.0);
  skill_config.rise_duration = declare_parameter<double>("rise_duration", 1.6);
  skill_config.lie_down_duration = declare_parameter<double>("lie_down_duration", 1.5);
  skill_config.velocity_timeout = declare_parameter<double>("velocity_timeout", 0.25);
  skill_config.wheel_radius = declare_parameter<double>("wheel_radius", 0.086);
  skill_config.track_width = declare_parameter<double>("track_width", 0.284);
  skill_config.max_wheel_speed = declare_parameter<double>("max_wheel_speed", 6.0);
  skill_config.wheel_acceleration = declare_parameter<double>("wheel_acceleration", 12.0);

  actuator_manager_ = std::make_unique<actuator::ActuatorManager>(config_path);
  stand_skill_ = std::make_unique<skill::StandSkill>(skill_config);
  driver_ = std::make_unique<driver::DrvDds>(
    *this, driver::DrvDds::Config{command_topic, state_topic, 10U});
  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    velocity_topic, 10,
    std::bind(&StandSkillNode::OnVelocityCommand, this, std::placeholders::_1));
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(control_period_seconds_));
  control_timer_ = create_wall_timer(
    timer_period, std::bind(&StandSkillNode::OnControlTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Joint-space StandSkill waiting on '%s', publishing '%s', velocity topic '%s'",
    state_topic.c_str(), command_topic.c_str(), velocity_topic.c_str());
}

bool StandSkillNode::LieDownBeforeShutdown()
{
  if (control_timer_) {
    control_timer_->cancel();
  }
  if (!RefreshFeedback()) {
    const auto feedback = driver_->GetFeedback();
    if (!feedback) {
      RCLCPP_WARN(get_logger(), "Shutdown requested without feedback; lie-down skipped");
      return false;
    }
  }

  const auto shutdown_started = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(shutdown_started - last_feedback_time_).count() >
    state_timeout_seconds_)
  {
    RCLCPP_WARN(get_logger(), "Feedback is stale; lie-down skipped");
    return false;
  }
  stand_skill_->RequestLieDown();
  RCLCPP_INFO(get_logger(), "Shutdown requested; returning to crouch pose");

  rclcpp::executors::SingleThreadedExecutor feedback_executor;
  feedback_executor.add_node(shared_from_this());
  auto previous_time = shutdown_started;
  auto next_update = shutdown_started;
  while (rclcpp::ok() && !stand_skill_->IsLying()) {
    feedback_executor.spin_some();
    RefreshFeedback();
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_feedback_time_).count() >
      state_timeout_seconds_)
    {
      RCLCPP_ERROR(get_logger(), "Feedback became stale during lie-down");
      break;
    }
    const double elapsed = std::max(
      std::chrono::duration<double>(now - previous_time).count(),
      control_period_seconds_);
    previous_time = now;
    SendSkillCommand(now, elapsed);
    if (std::chrono::duration<double>(now - shutdown_started).count() >=
      shutdown_timeout_seconds_)
    {
      RCLCPP_WARN(get_logger(), "Lie-down timed out before reaching crouch pose");
      break;
    }
    next_update += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_period_seconds_));
    std::this_thread::sleep_until(next_update);
  }

  for (int index = 0; index < 10; ++index) {
    feedback_executor.spin_some();
    RefreshFeedback();
    SendSkillCommand(std::chrono::steady_clock::now(), control_period_seconds_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  feedback_executor.remove_node(shared_from_this());
  RCLCPP_INFO(get_logger(), "Lie-down sequence finished; shutting down");
  return stand_skill_->IsLying();
}

void StandSkillNode::OnVelocityCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_ERROR(get_logger(), "Rejected /cmd_vel containing a non-finite value");
    return;
  }
  stand_skill_->SetVelocityTarget(
    message->linear.x, message->angular.z, std::chrono::steady_clock::now());
}

void StandSkillNode::OnControlTimer()
{
  if (RefreshFeedback() && !has_low_state_) {
    has_low_state_ = true;
    actuator_manager_->Reset();
    stand_skill_->Reset();
    last_control_time_ = last_feedback_time_;
    phase_logged_ = false;
    RCLCPP_INFO(
      get_logger(), "Received first low state; joint-space stand starts immediately");
  }
  if (!has_low_state_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "No /lowstate data; no command is being sent");
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double state_age = std::chrono::duration<double>(now - last_feedback_time_).count();
  if (state_age > state_timeout_seconds_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Low state is stale (%.3f s); sending damping fallback", state_age);
    SendDampingFallback(now);
    actuator_manager_->Reset();
    stand_skill_->Reset();
    has_low_state_ = false;
    phase_logged_ = false;
    return;
  }

  const double measured_elapsed =
    std::chrono::duration<double>(now - last_control_time_).count();
  const double elapsed = measured_elapsed > 0.0 ? measured_elapsed : control_period_seconds_;
  last_control_time_ = now;
  SendSkillCommand(now, elapsed);
}

bool StandSkillNode::RefreshFeedback()
{
  const auto feedback = driver_->GetFeedback();
  if (!feedback || feedback->received_at == last_feedback_time_) {
    return false;
  }
  joint_state_frame_ = actuator_manager_->DecodeFeedback(*feedback);
  last_feedback_time_ = feedback->received_at;
  return true;
}

bool StandSkillNode::SendSkillCommand(
  const SteadyTimePoint now, const double elapsed_seconds)
{
  const auto joint_command = stand_skill_->Update(
    joint_state_frame_, now, elapsed_seconds);
  LogSkillPhase();
  const auto actuator_output = actuator_manager_->Update(
    joint_command, joint_state_frame_, now, elapsed_seconds);
  robot_command_ = actuator_output.robot_command;
  if (!driver_->SendCommand(robot_command_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "DDS driver rejected the actuator output");
    return false;
  }
  LogActuatorResult(actuator_output, joint_state_frame_);
  return actuator_output.all_requests_accepted;
}

void StandSkillNode::SendDampingFallback(const SteadyTimePoint now)
{
  actuator::JointCommandFrame command;
  for (auto & joint : command.joints) {
    joint.control_mode = actuator::ControlMode::kDamping;
    joint.gain_profile = actuator::GainProfile::kSoft;
  }
  command.created_at = now;
  command.sequence = ++fallback_sequence_;
  const auto output = actuator_manager_->Update(
    command, joint_state_frame_, now, control_period_seconds_);
  driver_->SendCommand(output.robot_command);
}

void StandSkillNode::LogSkillPhase()
{
  const auto phase = stand_skill_->GetPhase();
  if (phase_logged_ && phase == last_logged_phase_) {
    return;
  }
  last_logged_phase_ = phase;
  phase_logged_ = true;
  RCLCPP_INFO(
    get_logger(), "StandSkill phase: %s", skill::StandSkill::PhaseName(phase));
}

void StandSkillNode::LogActuatorResult(
  const actuator::ManagerOutput & output, const actuator::JointStateFrame & state)
{
  if (output.combined_faults == actuator::ToMask(actuator::Fault::kNone)) {
    return;
  }
  std::ostringstream details;
  for (std::size_t index = 0U; index < output.status.size(); ++index) {
    if (output.status[index].faults == actuator::ToMask(actuator::Fault::kNone)) {
      continue;
    }
    const auto joint_id = static_cast<actuator::JointId>(index);
    const auto & config = actuator_manager_->GetModel().GetConfiguration(joint_id);
    details << " joint[" << index << "]/motor[" << config.motor_index << "]"
            << "=0x" << std::hex << output.status[index].faults << std::dec
            << "(q=" << state.joints[index].position
            << ", dq=" << state.joints[index].velocity << ')';
  }
  if (output.all_requests_accepted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Actuator requests accepted with limiting (status 0x%08x):%s",
      output.combined_faults, details.str().c_str());
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Actuator layer rejected request (status 0x%08x):%s",
      output.combined_faults, details.str().c_str());
  }
}

}  // namespace wheel_dog_mujoco

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void HandleShutdownSignal([[maybe_unused]] const int signal_number)
{
  shutdown_requested = 1;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, HandleShutdownSignal);
  std::signal(SIGTERM, HandleShutdownSignal);

  auto node = std::make_shared<wheel_dog_mujoco::StandSkillNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && shutdown_requested == 0) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  executor.remove_node(node);
  if (rclcpp::ok() && shutdown_requested != 0) {
    node->LieDownBeforeShutdown();
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
