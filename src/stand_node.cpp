#include "wheel_dog_mujoco/stand_node.h"

#include <algorithm>
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

StandNode::StandNode(const rclcpp::NodeOptions & options)
: Node("go2w_stand_node", options)
{
  controller_.SetConfig(LoadControllerConfig());

  control_period_seconds_ = declare_parameter<double>("control_period", 0.002);
  startup_delay_seconds_ = declare_parameter<double>("startup_delay", 1.0);
  state_timeout_seconds_ = declare_parameter<double>("state_timeout", 0.2);
  velocity_timeout_seconds_ = declare_parameter<double>("velocity_timeout", 0.25);
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.086);
  track_width_ = declare_parameter<double>("track_width", 0.284);
  max_wheel_speed_ = declare_parameter<double>("max_wheel_speed", 6.0);
  wheel_acceleration_ = declare_parameter<double>("wheel_acceleration", 12.0);
  const auto command_topic = declare_parameter<std::string>("command_topic", "/lowcmd");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/lowstate");
  const auto velocity_topic = declare_parameter<std::string>("velocity_topic", "/cmd_vel");
  const auto default_actuator_config =
    ament_index_cpp::get_package_share_directory("wheel_dog_mujoco") + "/config.yaml";
  const auto actuator_config_path = declare_parameter<std::string>(
    "actuator_config_path", default_actuator_config);

  if (control_period_seconds_ <= 0.0 || startup_delay_seconds_ < 0.0 ||
    state_timeout_seconds_ <= 0.0 || velocity_timeout_seconds_ <= 0.0 ||
    wheel_radius_ <= 0.0 || track_width_ <= 0.0 || max_wheel_speed_ <= 0.0 ||
    wheel_acceleration_ <= 0.0)
  {
    throw std::invalid_argument("Timing parameters are outside their valid range");
  }

  actuator_manager_ = std::make_unique<actuator::ActuatorManager>(actuator_config_path);
  driver_ = std::make_unique<driver::DrvDds>(
    *this, driver::DrvDds::Config{command_topic, state_topic, 10U});
  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    velocity_topic, 10,
    std::bind(&StandNode::OnVelocityCommand, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(control_period_seconds_));
  control_timer_ = create_wall_timer(timer_period, std::bind(&StandNode::OnControlTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Waiting for Go2W state on '%s' and velocity commands on '%s'; publishing '%s'",
    state_topic.c_str(), velocity_topic.c_str(), command_topic.c_str());
}

StandController::Config StandNode::LoadControllerConfig()
{
  StandController::Config config;
  config.crouch_duration = declare_parameter<double>(
    "crouch_duration", config.crouch_duration);
  config.stand_duration = declare_parameter<double>("stand_duration", config.stand_duration);
  config.lie_down_duration = declare_parameter<double>(
    "lie_down_duration", config.lie_down_duration);

  config.crouch_pose = ToJointPositions(
    declare_parameter<std::vector<double>>(
      "crouch_pose", std::vector<double>(config.crouch_pose.begin(), config.crouch_pose.end())),
    "crouch_pose");
  config.stand_pose = ToJointPositions(
    declare_parameter<std::vector<double>>(
      "stand_pose", std::vector<double>(config.stand_pose.begin(), config.stand_pose.end())),
    "stand_pose");
  return config;
}

bool StandNode::LieDownBeforeShutdown()
{
  if (control_timer_) {
    control_timer_->cancel();
  }

  const auto feedback = driver_->GetFeedback();
  if (!feedback) {
    RCLCPP_WARN(get_logger(), "Shutdown requested without low-state feedback; lie-down skipped");
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double state_age = std::chrono::duration<double>(now - feedback->received_at).count();
  if (state_age > state_timeout_seconds_) {
    RCLCPP_WARN(
      get_logger(), "Low-state feedback is stale (%.3f s); lie-down skipped", state_age);
    return false;
  }

  joint_state_frame_ = actuator_manager_->DecodeFeedback(*feedback);
  last_feedback_time_ = joint_state_frame_.received_at;
  StandController::JointPositions current_pose{};
  for (std::size_t index = 0; index < current_pose.size(); ++index) {
    const auto & joint = joint_state_frame_.joints[index];
    if (!joint.online) {
      RCLCPP_WARN(
        get_logger(), "Joint %zu is offline; lie-down skipped", index);
      return false;
    }
    current_pose[index] = joint.position;
  }
  if (!std::all_of(
      current_pose.begin(), current_pose.end(),
      [](const double position) {return std::isfinite(position);}))
  {
    RCLCPP_WARN(get_logger(), "Low state contains a non-finite position; lie-down skipped");
    return false;
  }

  has_velocity_command_ = false;
  desired_linear_velocity_ = 0.0;
  desired_yaw_velocity_ = 0.0;
  controller_.StartLieDown(current_pose);
  RCLCPP_INFO(get_logger(), "Shutdown requested; stopping wheels and lying down");

  rclcpp::executors::SingleThreadedExecutor feedback_executor;
  feedback_executor.add_node(shared_from_this());
  auto previous_time = std::chrono::steady_clock::now();
  auto next_update = previous_time;
  while (!controller_.IsLying()) {
    feedback_executor.spin_some();
    RefreshFeedback();
    const auto update_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(update_time - previous_time).count();
    previous_time = update_time;

    controller_.Update(elapsed);
    UpdateWheelSpeeds(elapsed, false);
    if (!SendActuatorCommand(update_time, elapsed)) {
      RCLCPP_ERROR(get_logger(), "Actuator safety rejected lie-down; fallback sent and lie-down aborted");
      feedback_executor.remove_node(shared_from_this());
      return false;
    }

    next_update += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_period_seconds_));
    std::this_thread::sleep_until(next_update);
  }

  current_right_wheel_speed_ = 0.0;
  current_left_wheel_speed_ = 0.0;
  for (int index = 0; index < 10; ++index) {
    feedback_executor.spin_some();
    RefreshFeedback();
    const auto update_time = std::chrono::steady_clock::now();
    SendActuatorCommand(update_time, control_period_seconds_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  feedback_executor.remove_node(shared_from_this());
  RCLCPP_INFO(get_logger(), "Lie-down complete; shutting down the node");
  return true;
}

StandController::JointPositions StandNode::ToJointPositions(
  const std::vector<double> & values, const std::string & parameter_name)
{
  if (values.size() != StandController::kLegMotorCount) {
    throw std::invalid_argument(parameter_name + " must contain exactly 12 joint positions");
  }

  StandController::JointPositions positions{};
  std::copy(values.begin(), values.end(), positions.begin());
  return positions;
}

void StandNode::OnVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_ERROR(get_logger(), "Rejected /cmd_vel containing a non-finite value");
    return;
  }

  desired_linear_velocity_ = message->linear.x;
  desired_yaw_velocity_ = message->angular.z;
  has_velocity_command_ = true;
  last_velocity_command_time_ = std::chrono::steady_clock::now();
}

void StandNode::OnControlTimer()
{
  const bool received_new_feedback = RefreshFeedback();
  if (received_new_feedback) {
    if (!has_low_state_) {
      first_state_time_ = last_feedback_time_;
      RCLCPP_INFO(get_logger(), "Received the first low state; observing before stand-up");
    }
    has_low_state_ = true;
  }

  if (!has_low_state_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "No /lowstate data; no motor command is being sent");
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double state_age = std::chrono::duration<double>(now - last_feedback_time_).count();
  if (state_age > state_timeout_seconds_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Low state is stale (%.3f s); command publishing is paused", state_age);
    current_right_wheel_speed_ = 0.0;
    current_left_wheel_speed_ = 0.0;
    if (controller_started_) {
      SendActuatorCommand(now, control_period_seconds_);
    }
    controller_started_ = false;
    controller_.Reset();
    actuator_manager_->Reset();
    has_low_state_ = false;
    return;
  }

  const double observation_time = std::chrono::duration<double>(now - first_state_time_).count();
  if (!controller_started_ && observation_time < startup_delay_seconds_) {
    return;
  }

  if (!controller_started_) {
    StandController::JointPositions current_pose{};
    for (std::size_t index = 0; index < current_pose.size(); ++index) {
      if (!joint_state_frame_.joints[index].online) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joint %zu is offline; stand-up is blocked", index);
        return;
      }
      current_pose[index] = joint_state_frame_.joints[index].position;
    }
    if (!std::all_of(
        current_pose.begin(), current_pose.end(),
        [](const double position) {return std::isfinite(position);}))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Low state contains a non-finite joint position; stand-up is blocked");
      return;
    }
    controller_.Start(current_pose);
    controller_started_ = true;
    last_control_time_ = now;
    RCLCPP_INFO(get_logger(), "Starting the Go2W stand-up sequence");
  }

  const double measured_elapsed =
    std::chrono::duration<double>(now - last_control_time_).count();
  const double elapsed = measured_elapsed > 0.0 ? measured_elapsed : control_period_seconds_;
  last_control_time_ = now;
  controller_.Update(elapsed);
  UpdateWheelSpeeds(elapsed, controller_.IsHolding());
  SendActuatorCommand(now, elapsed);
  LogStateTransition(controller_.GetState());
}

void StandNode::UpdateWheelSpeeds(
  const double elapsed_seconds, const bool driving_enabled)
{
  double target_right_speed = 0.0;
  double target_left_speed = 0.0;

  if (driving_enabled && has_velocity_command_) {
    const auto now = std::chrono::steady_clock::now();
    const double command_age =
      std::chrono::duration<double>(now - last_velocity_command_time_).count();
    if (command_age <= velocity_timeout_seconds_) {
      target_right_speed =
        (desired_linear_velocity_ + 0.5 * track_width_ * desired_yaw_velocity_) /
        wheel_radius_;
      target_left_speed =
        (desired_linear_velocity_ - 0.5 * track_width_ * desired_yaw_velocity_) /
        wheel_radius_;

      const double largest_speed = std::max(
        std::abs(target_right_speed), std::abs(target_left_speed));
      if (largest_speed > max_wheel_speed_) {
        const double scale = max_wheel_speed_ / largest_speed;
        target_right_speed *= scale;
        target_left_speed *= scale;
      }
    } else {
      has_velocity_command_ = false;
      RCLCPP_WARN(
        get_logger(), "Velocity command timed out after %.3f s; stopping wheels", command_age);
    }
  }

  const double max_change = wheel_acceleration_ * std::max(0.0, elapsed_seconds);
  current_right_wheel_speed_ = Approach(
    current_right_wheel_speed_, target_right_speed, max_change);
  current_left_wheel_speed_ = Approach(
    current_left_wheel_speed_, target_left_speed, max_change);
}

bool StandNode::RefreshFeedback()
{
  const auto feedback = driver_->GetFeedback();
  if (!feedback || feedback->received_at == last_feedback_time_) {
    return false;
  }
  joint_state_frame_ = actuator_manager_->DecodeFeedback(*feedback);
  last_feedback_time_ = joint_state_frame_.received_at;
  return true;
}

void StandNode::BuildJointCommand(const SteadyTimePoint now)
{
  const auto & desired_pose = controller_.GetDesiredPose();

  for (std::size_t index = 0; index < StandController::kLegMotorCount; ++index) {
    auto & command = joint_command_frame_.joints[index];
    command.control_mode = actuator::ControlMode::kPosition;
    command.gain_profile = actuator::GainProfile::kNormal;
    command.position = desired_pose[index];
    command.velocity = 0.0;
    command.torque_feedforward = 0.0;
  }

  const std::array<double, StandController::kWheelMotorCount> wheel_speeds{
    current_right_wheel_speed_, current_left_wheel_speed_,
    current_right_wheel_speed_, current_left_wheel_speed_};
  for (std::size_t offset = 0; offset < wheel_speeds.size(); ++offset) {
    auto & command =
      joint_command_frame_.joints[StandController::kWheelMotorOffset + offset];
    command.control_mode = actuator::ControlMode::kVelocity;
    command.gain_profile = actuator::GainProfile::kNormal;
    command.position = 0.0;
    command.velocity = wheel_speeds[offset];
    command.torque_feedforward = 0.0;
  }
  joint_command_frame_.created_at = now;
  joint_command_frame_.sequence = ++command_sequence_;
}

bool StandNode::SendActuatorCommand(
  const SteadyTimePoint now, const double elapsed_seconds)
{
  BuildJointCommand(now);
  const auto output = actuator_manager_->Update(
    joint_command_frame_, joint_state_frame_, now, elapsed_seconds);
  robot_command_ = output.robot_command;
  if (!driver_->SendCommand(robot_command_)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "DDS driver rejected the actuator manager output");
    return false;
  }
  if (output.combined_faults != actuator::ToMask(actuator::Fault::kNone)) {
    std::ostringstream fault_details;
    for (std::size_t index = 0; index < output.status.size(); ++index) {
      if (output.status[index].faults == actuator::ToMask(actuator::Fault::kNone)) {
        continue;
      }
      const auto joint_id = static_cast<actuator::JointId>(index);
      const auto & config = actuator_manager_->GetModel().GetConfiguration(joint_id);
      const auto & state = joint_state_frame_.joints[index];
      fault_details << " joint[" << index << "]/motor[" << config.motor_index << "]"
                    << "=0x" << std::hex << output.status[index].faults << std::dec
                    << "(q=" << state.position << ", dq=" << state.velocity
                    << ", range=[" << config.limits.min_position << ','
                    << config.limits.max_position << "], vmax="
                    << config.limits.max_velocity << ')';
    }
    if (!output.all_requests_accepted) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Actuator layer rejected a request (fault mask: 0x%08x); safe fallback was sent:%s",
        output.combined_faults, fault_details.str().c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Actuator layer is recovering a bounded position violation (fault mask: 0x%08x):%s",
        output.combined_faults, fault_details.str().c_str());
    }
  }
  return output.all_requests_accepted;
}

double StandNode::Approach(
  const double current, const double target, const double max_change) noexcept
{
  if (target > current) {
    return std::min(target, current + max_change);
  }
  return std::max(target, current - max_change);
}

void StandNode::LogStateTransition(const StandController::State state)
{
  if (state == last_logged_state_) {
    return;
  }
  last_logged_state_ = state;

  switch (state) {
    case StandController::State::kMovingToCrouch:
      RCLCPP_INFO(get_logger(), "Stand phase: moving to crouch pose");
      break;
    case StandController::State::kStandingUp:
      RCLCPP_INFO(get_logger(), "Stand phase: rising");
      break;
    case StandController::State::kHolding:
      RCLCPP_INFO(get_logger(), "Stand phase: holding stand pose");
      break;
    case StandController::State::kLyingDown:
      RCLCPP_INFO(get_logger(), "Posture phase: lying down");
      break;
    case StandController::State::kLying:
      RCLCPP_INFO(get_logger(), "Posture phase: lying pose reached");
      break;
    case StandController::State::kIdle:
      break;
  }
}

}  // namespace wheel_dog_mujoco

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void HandleShutdownSignal(int)
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

  auto node = std::make_shared<wheel_dog_mujoco::StandNode>();
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
