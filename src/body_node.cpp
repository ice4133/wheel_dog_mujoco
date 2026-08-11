#include "wheel_dog_mujoco/body_node.h"

#include <algorithm>
#include <array>
#include <csignal>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheel_dog_mujoco
{
namespace
{

bool AllFinite(const std::array<float, 3> & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(),
    [](const float value) {return std::isfinite(value);});
}

bool AllFinite(const std::array<float, 4> & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(),
    [](const float value) {return std::isfinite(value);});
}

}  // namespace

BodyNode::BodyNode(const rclcpp::NodeOptions & options)
: Node("go2w_body_node", options)
{
  control_period_seconds_ = declare_parameter<double>("control_period", 0.002);
  state_timeout_seconds_ = declare_parameter<double>("state_timeout", 0.2);
  shutdown_timeout_seconds_ = declare_parameter<double>("shutdown_timeout", 4.0);
  contact_force_feedback_available_ = declare_parameter<bool>(
    "contact_force_feedback_available", false);
  const auto command_topic = declare_parameter<std::string>("command_topic", "/lowcmd");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/lowstate");
  const auto velocity_topic = declare_parameter<std::string>("velocity_topic", "/cmd_vel");
  const auto default_config =
    ament_index_cpp::get_package_share_directory("wheel_dog_mujoco") + "/config.yaml";
  const auto config_path = declare_parameter<std::string>("config_path", default_config);

  if (control_period_seconds_ <= 0.0 || state_timeout_seconds_ <= 0.0 ||
    shutdown_timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument("BodyNode timing parameters are outside their valid range");
  }

  body_manager_ = std::make_unique<body::BodyManager>(config_path);
  actuator_manager_ = std::make_unique<actuator::ActuatorManager>(config_path);

  skill::StandSkill::Config skill_config;
  skill_config.recovery_ground_clearance = declare_parameter<double>(
    "recovery_ground_clearance", 0.20);
  skill_config.stand_ground_clearance = declare_parameter<double>(
    "stand_ground_clearance", 0.42);
  skill_config.lie_down_ground_clearance = declare_parameter<double>(
    "lie_down_ground_clearance", 0.20);
  skill_config.height_tolerance = declare_parameter<double>("height_tolerance", 0.03);
  skill_config.attitude_tolerance = declare_parameter<double>("attitude_tolerance", 0.15);
  skill_config.angular_velocity_tolerance = declare_parameter<double>(
    "angular_velocity_tolerance", 0.40);
  skill_config.settle_duration = declare_parameter<double>("settle_duration", 0.25);
  skill_config.velocity_timeout = declare_parameter<double>("velocity_timeout", 0.25);

  const auto & limits = body_manager_->GetConfig().limits;
  if (skill_config.recovery_ground_clearance < limits.min_ground_clearance ||
    skill_config.recovery_ground_clearance > limits.max_ground_clearance ||
    skill_config.stand_ground_clearance < limits.min_ground_clearance ||
    skill_config.stand_ground_clearance > limits.max_ground_clearance ||
    skill_config.lie_down_ground_clearance < limits.min_ground_clearance ||
    skill_config.lie_down_ground_clearance > limits.max_ground_clearance)
  {
    throw std::invalid_argument("StandSkill ground clearance is outside body limits");
  }
  stand_skill_ = std::make_unique<skill::StandSkill>(skill_config);

  driver_ = std::make_unique<driver::DrvDds>(
    *this, driver::DrvDds::Config{command_topic, state_topic, 10U});
  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    velocity_topic, 10,
    std::bind(&BodyNode::OnVelocityCommand, this, std::placeholders::_1));
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(control_period_seconds_));
  control_timer_ = create_wall_timer(timer_period, std::bind(&BodyNode::OnControlTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Body runtime waiting on '%s', publishing '%s', velocity topic '%s'",
    state_topic.c_str(), command_topic.c_str(), velocity_topic.c_str());
  if (!contact_force_feedback_available_) {
    RCLCPP_INFO(
      get_logger(),
      "Contact-force feedback disabled; estimator will use kinematic support assumption");
  }
}

bool BodyNode::LieDownBeforeShutdown()
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
  RCLCPP_INFO(get_logger(), "Shutdown requested; StandSkill is lowering the body");

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
      RCLCPP_WARN(get_logger(), "Lie-down timed out before the skill reached lying state");
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

void BodyNode::OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_ERROR(get_logger(), "Rejected /cmd_vel containing a non-finite value");
    return;
  }
  stand_skill_->SetVelocityTarget(
    message->linear.x, message->angular.z, std::chrono::steady_clock::now());
}

void BodyNode::OnControlTimer()
{
  if (RefreshFeedback() && !has_low_state_) {
    has_low_state_ = true;
    body_manager_->Reset();
    actuator_manager_->Reset();
    stand_skill_->Reset();
    body_output_ = body::ManagerOutput{};
    last_control_time_ = last_feedback_time_;
    phase_logged_ = false;
    RCLCPP_INFO(
      get_logger(), "Received first low state; body control starts without passive delay");
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
      "Low state is stale (%.3f s); sending disabled fallback", state_age);
    SendDisabledFallback(now);
    body_manager_->Reset();
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

bool BodyNode::RefreshFeedback()
{
  const auto feedback = driver_->GetFeedback();
  if (!feedback || feedback->received_at == last_feedback_time_) {
    return false;
  }
  joint_state_frame_ = actuator_manager_->DecodeFeedback(*feedback);
  body_sensor_frame_ = DecodeBodySensors(*feedback);
  last_feedback_time_ = feedback->received_at;
  return true;
}

body::BodySensorFrame BodyNode::DecodeBodySensors(
  const driver::RobotFeedback & feedback) const
{
  body::BodySensorFrame frame;
  frame.received_at = feedback.received_at;
  frame.sequence = feedback.tick;
  frame.imu.orientation_world_from_body = body::Quaternion{
    feedback.imu.quaternion[0], feedback.imu.quaternion[1],
    feedback.imu.quaternion[2], feedback.imu.quaternion[3]};
  frame.imu.angular_velocity_body = body::Vector3{
    feedback.imu.gyroscope[0], feedback.imu.gyroscope[1], feedback.imu.gyroscope[2]};
  frame.imu.linear_acceleration_body = body::Vector3{
    feedback.imu.accelerometer[0], feedback.imu.accelerometer[1],
    feedback.imu.accelerometer[2]};
  frame.imu.temperature = static_cast<double>(feedback.imu.temperature);
  const double quaternion_norm_squared =
    frame.imu.orientation_world_from_body.w * frame.imu.orientation_world_from_body.w +
    frame.imu.orientation_world_from_body.x * frame.imu.orientation_world_from_body.x +
    frame.imu.orientation_world_from_body.y * frame.imu.orientation_world_from_body.y +
    frame.imu.orientation_world_from_body.z * frame.imu.orientation_world_from_body.z;
  frame.imu.orientation_valid = AllFinite(feedback.imu.quaternion) &&
    quaternion_norm_squared > 1.0E-12;
  frame.imu.angular_velocity_valid = AllFinite(feedback.imu.gyroscope);
  frame.imu.linear_acceleration_valid = AllFinite(feedback.imu.accelerometer);

  const double threshold = body_manager_->GetConfig().estimator.contact_force_threshold;
  for (std::size_t index = 0U; index < body::kLegCount; ++index) {
    const double measured = std::max(
      0.0, static_cast<double>(std::max(
        feedback.foot_force[index], feedback.estimated_foot_force[index])));
    frame.contacts[index].normal_force = measured;
    frame.contacts[index].valid = contact_force_feedback_available_;
    frame.contacts[index].in_contact =
      contact_force_feedback_available_ && measured >= threshold;
  }
  return frame;
}

bool BodyNode::SendSkillCommand(
  const SteadyTimePoint now, const double elapsed_seconds)
{
  const auto command = stand_skill_->Update(body_output_.body_state_frame, now);
  LogSkillPhase();
  return SendBodyCommand(command, now, elapsed_seconds);
}

bool BodyNode::SendBodyCommand(
  const body::BodyCommandFrame & command, const SteadyTimePoint now,
  const double elapsed_seconds)
{
  body_output_ = body_manager_->Update(
    command, joint_state_frame_, body_sensor_frame_, now, elapsed_seconds);
  const auto actuator_output = actuator_manager_->Update(
    body_output_.joint_command_frame, joint_state_frame_, now, elapsed_seconds);
  robot_command_ = actuator_output.robot_command;
  if (!driver_->SendCommand(robot_command_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "DDS driver rejected the actuator output");
    return false;
  }
  LogBodyResult(body_output_);
  LogActuatorResult(actuator_output, joint_state_frame_);
  return body_output_.accepted && actuator_output.all_requests_accepted;
}

void BodyNode::SendDisabledFallback(const SteadyTimePoint now)
{
  body::BodyCommandFrame command;
  command.command.control_mode = body::ControlMode::kDisabled;
  command.created_at = now;
  command.sequence = ++fallback_sequence_;
  SendBodyCommand(command, now, control_period_seconds_);
}

void BodyNode::LogSkillPhase()
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

void BodyNode::LogBodyResult(const body::ManagerOutput & output)
{
  if (!output.accepted) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Body layer rejected command; damping fallback sent (fault mask: 0x%08x)",
      output.combined_faults);
  } else if (output.combined_faults != body::ToMask(body::Fault::kNone)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Body layer reports nonfatal status 0x%08x", output.combined_faults);
  }
}

void BodyNode::LogActuatorResult(
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

  auto node = std::make_shared<wheel_dog_mujoco::BodyNode>();
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
