#include "wheel_dog_mujoco/stand_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "wheel_dog_mujoco/motor_crc.h"

namespace wheel_dog_mujoco
{

StandNode::StandNode(const rclcpp::NodeOptions & options)
: Node("go2w_stand_node", options)
{
  controller_.SetConfig(LoadControllerConfig());

  control_period_seconds_ = declare_parameter<double>("control_period", 0.002);
  startup_delay_seconds_ = declare_parameter<double>("startup_delay", 1.0);
  state_timeout_seconds_ = declare_parameter<double>("state_timeout", 0.2);
  const auto command_topic = declare_parameter<std::string>("command_topic", "/lowcmd");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/lowstate");

  if (control_period_seconds_ <= 0.0 || startup_delay_seconds_ < 0.0 ||
    state_timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument("Timing parameters are outside their valid range");
  }

  InitializeCommand();
  command_publisher_ = create_publisher<unitree_go::msg::LowCmd>(command_topic, 10);
  state_subscription_ = create_subscription<unitree_go::msg::LowState>(
    state_topic, 10, std::bind(&StandNode::OnLowState, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(control_period_seconds_));
  control_timer_ = create_wall_timer(timer_period, std::bind(&StandNode::OnControlTimer, this));

  RCLCPP_INFO(
    get_logger(), "Waiting for Go2W low state on '%s'; commands will be published on '%s'",
    state_topic.c_str(), command_topic.c_str());
}

StandController::Config StandNode::LoadControllerConfig()
{
  StandController::Config config;
  config.crouch_duration = declare_parameter<double>(
    "crouch_duration", config.crouch_duration);
  config.stand_duration = declare_parameter<double>("stand_duration", config.stand_duration);
  config.leg_kp = declare_parameter<double>("leg_kp", config.leg_kp);
  config.leg_kd = declare_parameter<double>("leg_kd", config.leg_kd);
  config.wheel_kd = declare_parameter<double>("wheel_kd", config.wheel_kd);

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

void StandNode::InitializeCommand()
{
  low_command_.head[0] = 0xFE;
  low_command_.head[1] = 0xEF;
  low_command_.level_flag = 0xFF;
  low_command_.gpio = 0;

  for (auto & motor : low_command_.motor_cmd) {
    motor.mode = 0x01;
    motor.q = kPositionStop;
    motor.dq = kVelocityStop;
    motor.kp = 0.0F;
    motor.kd = 0.0F;
    motor.tau = 0.0F;
  }
}

void StandNode::OnLowState(const unitree_go::msg::LowState::SharedPtr message)
{
  low_state_ = *message;
  const auto now = std::chrono::steady_clock::now();
  if (!has_low_state_) {
    first_state_time_ = now;
    RCLCPP_INFO(get_logger(), "Received the first low state; observing before stand-up");
  }
  has_low_state_ = true;
  last_state_time_ = now;
}

void StandNode::OnControlTimer()
{
  if (!has_low_state_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "No /lowstate data; no motor command is being sent");
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double state_age = std::chrono::duration<double>(now - last_state_time_).count();
  if (state_age > state_timeout_seconds_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Low state is stale (%.3f s); command publishing is paused", state_age);
    controller_started_ = false;
    controller_.Reset();
    return;
  }

  const double observation_time = std::chrono::duration<double>(now - first_state_time_).count();
  if (!controller_started_ && observation_time < startup_delay_seconds_) {
    return;
  }

  if (!controller_started_) {
    StandController::JointPositions current_pose{};
    for (std::size_t index = 0; index < current_pose.size(); ++index) {
      current_pose[index] = low_state_.motor_state[index].q;
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

  const double elapsed = std::chrono::duration<double>(now - last_control_time_).count();
  last_control_time_ = now;
  controller_.Update(elapsed);
  ApplyControllerCommand();
  SetCrc(low_command_);
  command_publisher_->publish(low_command_);
  LogStateTransition(controller_.GetState());
}

void StandNode::ApplyControllerCommand()
{
  const auto & desired_pose = controller_.GetDesiredPose();
  const auto & config = controller_.GetConfig();

  for (std::size_t index = 0; index < StandController::kLegMotorCount; ++index) {
    auto & motor = low_command_.motor_cmd[index];
    motor.mode = 0x01;
    motor.q = static_cast<float>(desired_pose[index]);
    motor.dq = 0.0F;
    motor.kp = static_cast<float>(config.leg_kp);
    motor.kd = static_cast<float>(config.leg_kd);
    motor.tau = 0.0F;
  }

  for (std::size_t offset = 0; offset < StandController::kWheelMotorCount; ++offset) {
    auto & motor = low_command_.motor_cmd[StandController::kWheelMotorOffset + offset];
    motor.mode = 0x01;
    motor.q = kPositionStop;
    motor.dq = 0.0F;
    motor.kp = 0.0F;
    motor.kd = static_cast<float>(config.wheel_kd);
    motor.tau = 0.0F;
  }
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
    case StandController::State::kIdle:
      break;
  }
}

}  // namespace wheel_dog_mujoco
