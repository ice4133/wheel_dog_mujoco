#include "wheel_dog_mujoco/skill/skill_task_node.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace wheel_dog_mujoco::skill
{
namespace
{

constexpr std::array<actuator::JointId, 4> kWheelJointIds{{
  actuator::JointId::kFrontRightWheel,
  actuator::JointId::kFrontLeftWheel,
  actuator::JointId::kRearRightWheel,
  actuator::JointId::kRearLeftWheel,
}};

struct SystemConfig
{
  std::string driver_config_path;
  std::string actuator_config_path;
  std::string skill_config_path;
  std::string velocity_topic;
  std::string skill_request_topic;
  std::string skill_state_topic;
  double control_period{0.002};
  double state_timeout{0.2};
  double shutdown_timeout{4.0};
};

struct DriverConfig
{
  std::string command_topic;
  std::string state_topic;
  std::size_t queue_depth{10U};
};

struct SkillConfig
{
  SkillTask::Config task{};
  StandSkill::Config stand{};
};

bool IsFinite(const double value) noexcept
{
  return std::isfinite(value);
}

double Approach(const double current, const double target, const double max_change) noexcept
{
  return current + std::clamp(target - current, -max_change, max_change);
}

std::string ResolveConfigPath(
  const std::filesystem::path & system_config_path,
  const YAML::Node & configs, const char * key)
{
  const auto configured_path = std::filesystem::path(configs[key].as<std::string>());
  if (configured_path.is_absolute()) {
    return configured_path.lexically_normal().string();
  }
  return (system_config_path.parent_path() / configured_path).lexically_normal().string();
}

SystemConfig LoadSystemConfig(const std::string & config_path)
{
  try {
    const std::filesystem::path system_path(config_path);
    const YAML::Node system = YAML::LoadFile(config_path)["system"];
    if (!system.IsMap() || !system["configs"].IsMap()) {
      throw std::invalid_argument("system.yaml requires system and system.configs maps");
    }

    SystemConfig config;
    const YAML::Node files = system["configs"];
    config.driver_config_path = ResolveConfigPath(system_path, files, "driver");
    config.actuator_config_path = ResolveConfigPath(system_path, files, "actuator");
    config.skill_config_path = ResolveConfigPath(system_path, files, "skill");
    config.control_period = system["control_loop"]["period"].as<double>();
    config.state_timeout = system["control_loop"]["state_timeout"].as<double>();
    config.velocity_topic = system["interfaces"]["velocity_topic"].as<std::string>();
    config.skill_request_topic =
      system["interfaces"]["skill_request_topic"].as<std::string>();
    config.skill_state_topic = system["interfaces"]["skill_state_topic"].as<std::string>();
    config.shutdown_timeout = system["shutdown"]["timeout"].as<double>();
    return config;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to parse system configuration '" + config_path + "': " + error.what());
  }
}

DriverConfig LoadDriverConfig(const std::string & config_path)
{
  try {
    const YAML::Node driver = YAML::LoadFile(config_path)["driver"];
    if (!driver.IsMap()) {
      throw std::invalid_argument("driver.yaml requires a driver map");
    }
    DriverConfig config;
    config.command_topic = driver["command_topic"].as<std::string>();
    config.state_topic = driver["state_topic"].as<std::string>();
    config.queue_depth = driver["queue_depth"].as<std::size_t>();
    return config;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to parse driver configuration '" + config_path + "': " + error.what());
  }
}

StandSkill::LegJointPositions ConvertPose(
  const std::vector<double> & values, const char * parameter_name)
{
  if (values.size() != StandSkill::kLegJointCount ||
    !std::all_of(values.begin(), values.end(), IsFinite))
  {
    throw std::invalid_argument(
            std::string(parameter_name) + " must contain 12 finite joint angles");
  }
  StandSkill::LegJointPositions pose{};
  std::copy(values.begin(), values.end(), pose.begin());
  return pose;
}

SkillTask::Target ParseInitialTarget(const std::string & value)
{
  if (value == "idle") {
    return SkillTask::Target::kIdle;
  }
  if (value == "stand") {
    return SkillTask::Target::kStand;
  }
  throw std::invalid_argument("skill.task.initial_state must be 'idle' or 'stand'");
}

SkillConfig LoadSkillConfig(const std::string & config_path)
{
  try {
    const YAML::Node skill = YAML::LoadFile(config_path)["skill"];
    const YAML::Node task = skill["task"];
    const YAML::Node stand = skill["stand"];
    if (!task.IsMap() || !stand.IsMap()) {
      throw std::invalid_argument("skill.yaml requires skill.task and skill.stand maps");
    }

    SkillConfig config;
    config.task.initial_target = ParseInitialTarget(task["initial_state"].as<std::string>());
    config.stand.crouch_pose = ConvertPose(
      stand["crouch_pose"].as<std::vector<double>>(), "skill.stand.crouch_pose");
    config.stand.stand_pose = ConvertPose(
      stand["stand_pose"].as<std::vector<double>>(), "skill.stand.stand_pose");
    config.stand.crouch_duration = stand["crouch_duration"].as<double>();
    config.stand.rise_duration = stand["rise_duration"].as<double>();
    config.stand.lie_down_duration = stand["lie_down_duration"].as<double>();
    config.stand.position_tolerance = stand["position_tolerance"].as<double>();
    config.stand.velocity_tolerance = stand["velocity_tolerance"].as<double>();
    config.stand.velocity_timeout = stand["velocity_timeout"].as<double>();
    config.stand.wheel_radius = stand["wheel_radius"].as<double>();
    config.stand.track_width = stand["track_width"].as<double>();
    config.stand.max_wheel_speed = stand["max_wheel_speed"].as<double>();
    config.stand.wheel_acceleration = stand["wheel_acceleration"].as<double>();
    return config;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to parse skill configuration '" + config_path + "': " + error.what());
  }
}

std::string NormalizeRequest(std::string request)
{
  request.erase(
    std::remove_if(
      request.begin(), request.end(),
      [](const unsigned char character) {return std::isspace(character) != 0;}),
    request.end());
  std::transform(
    request.begin(), request.end(), request.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return request;
}

}  // namespace

SkillTaskNode::SkillTaskNode(const rclcpp::NodeOptions & options)
: Node("go2w_skill_task", options)
{
  const auto package_share =
    ament_index_cpp::get_package_share_directory("wheel_dog_mujoco");
  const auto system_config_path = declare_parameter<std::string>(
    "system_config_path", package_share + "/config/system.yaml");
  const SystemConfig system_config = LoadSystemConfig(system_config_path);
  const auto driver_config_path = declare_parameter<std::string>(
    "driver_config_path", system_config.driver_config_path);
  const auto actuator_config_path = declare_parameter<std::string>(
    "actuator_config_path", system_config.actuator_config_path);
  const auto skill_config_path = declare_parameter<std::string>(
    "skill_config_path", system_config.skill_config_path);
  const DriverConfig driver_config = LoadDriverConfig(driver_config_path);
  SkillConfig skill_config = LoadSkillConfig(skill_config_path);

  control_period_seconds_ = declare_parameter<double>(
    "control_period", system_config.control_period);
  state_timeout_seconds_ = declare_parameter<double>(
    "state_timeout", system_config.state_timeout);
  shutdown_timeout_seconds_ = declare_parameter<double>(
    "shutdown_timeout", system_config.shutdown_timeout);
  const auto command_topic = declare_parameter<std::string>(
    "command_topic", driver_config.command_topic);
  const auto state_topic = declare_parameter<std::string>(
    "state_topic", driver_config.state_topic);
  const auto velocity_topic = declare_parameter<std::string>(
    "velocity_topic", system_config.velocity_topic);
  const auto skill_request_topic = declare_parameter<std::string>(
    "skill_request_topic", system_config.skill_request_topic);
  const auto skill_state_topic = declare_parameter<std::string>(
    "skill_state_topic", system_config.skill_state_topic);
  const auto queue_depth = declare_parameter<std::int64_t>(
    "driver_queue_depth", static_cast<std::int64_t>(driver_config.queue_depth));
  if (control_period_seconds_ <= 0.0 || state_timeout_seconds_ <= 0.0 ||
    shutdown_timeout_seconds_ <= 0.0 || queue_depth <= 0)
  {
    throw std::invalid_argument("SkillTaskNode timing parameters are invalid");
  }

  skill_config.stand.crouch_pose = ConvertPose(
    declare_parameter<std::vector<double>>(
      "crouch_pose", std::vector<double>(
        skill_config.stand.crouch_pose.begin(), skill_config.stand.crouch_pose.end())),
    "crouch_pose");
  skill_config.stand.stand_pose = ConvertPose(
    declare_parameter<std::vector<double>>(
      "stand_pose", std::vector<double>(
        skill_config.stand.stand_pose.begin(), skill_config.stand.stand_pose.end())),
    "stand_pose");
  skill_config.stand.crouch_duration = declare_parameter<double>(
    "crouch_duration", skill_config.stand.crouch_duration);
  skill_config.stand.rise_duration = declare_parameter<double>(
    "rise_duration", skill_config.stand.rise_duration);
  skill_config.stand.lie_down_duration = declare_parameter<double>(
    "lie_down_duration", skill_config.stand.lie_down_duration);
  skill_config.stand.position_tolerance = declare_parameter<double>(
    "position_tolerance", skill_config.stand.position_tolerance);
  skill_config.stand.velocity_tolerance = declare_parameter<double>(
    "velocity_tolerance", skill_config.stand.velocity_tolerance);
  skill_config.stand.velocity_timeout = declare_parameter<double>(
    "velocity_timeout", skill_config.stand.velocity_timeout);
  skill_config.stand.wheel_radius = declare_parameter<double>(
    "wheel_radius", skill_config.stand.wheel_radius);
  skill_config.stand.track_width = declare_parameter<double>(
    "track_width", skill_config.stand.track_width);
  skill_config.stand.max_wheel_speed = declare_parameter<double>(
    "max_wheel_speed", skill_config.stand.max_wheel_speed);
  skill_config.stand.wheel_acceleration = declare_parameter<double>(
    "wheel_acceleration", skill_config.stand.wheel_acceleration);

  actuator_manager_ = std::make_unique<actuator::ActuatorManager>(actuator_config_path);
  stand_skill_ = std::make_unique<StandSkill>(skill_config.stand);
  skill_task_ = std::make_unique<SkillTask>(skill_config.task);
  driver_ = std::make_unique<driver::DrvDds>(
    *this, driver::DrvDds::Config{
      command_topic, state_topic, static_cast<std::size_t>(queue_depth)});
  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    velocity_topic, 10,
    std::bind(&SkillTaskNode::OnVelocityCommand, this, std::placeholders::_1));
  skill_request_subscription_ = create_subscription<std_msgs::msg::String>(
    skill_request_topic, 10,
    std::bind(&SkillTaskNode::OnSkillRequest, this, std::placeholders::_1));
  skill_state_publisher_ = create_publisher<std_msgs::msg::String>(
    skill_state_topic, rclcpp::QoS(1).transient_local().reliable());
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(control_period_seconds_));
  control_timer_ = create_wall_timer(
    timer_period, std::bind(&SkillTaskNode::OnControlTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "SkillTask target starts as '%s'; request topic '%s', state topic '%s'",
    SkillTask::TargetName(skill_task_->GetRequestedTarget()), skill_request_topic.c_str(),
    skill_state_topic.c_str());
  RCLCPP_INFO(
    get_logger(), "Waiting on '%s', publishing '%s', velocity topic '%s'",
    state_topic.c_str(), command_topic.c_str(), velocity_topic.c_str());
  PublishTaskState();
}

bool SkillTaskNode::ReturnToIdleBeforeShutdown()
{
  if (control_timer_) {
    control_timer_->cancel();
  }
  skill_request_subscription_.reset();
  velocity_subscription_.reset();

  if (!RefreshFeedback() && !driver_->GetFeedback()) {
    RCLCPP_WARN(get_logger(), "Shutdown requested without feedback; safe return skipped");
    return false;
  }
  const auto shutdown_started = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(shutdown_started - last_feedback_time_).count() >
    state_timeout_seconds_)
  {
    RCLCPP_WARN(get_logger(), "Feedback is stale; safe return skipped");
    return false;
  }

  skill_task_->RequestTarget(SkillTask::Target::kIdle);
  RCLCPP_INFO(get_logger(), "Shutdown requested; transitioning SkillTask to idle");

  rclcpp::executors::SingleThreadedExecutor feedback_executor;
  feedback_executor.add_node(shared_from_this());
  auto previous_time = shutdown_started;
  auto next_update = shutdown_started;
  while (rclcpp::ok() && !skill_task_->IsIdle()) {
    feedback_executor.spin_some();
    RefreshFeedback();
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_feedback_time_).count() >
      state_timeout_seconds_)
    {
      RCLCPP_ERROR(get_logger(), "Feedback became stale during safe return");
      break;
    }
    const double elapsed = std::max(
      std::chrono::duration<double>(now - previous_time).count(),
      control_period_seconds_);
    previous_time = now;
    SendTaskCommand(now, elapsed);
    if (std::chrono::duration<double>(now - shutdown_started).count() >=
      shutdown_timeout_seconds_)
    {
      RCLCPP_WARN(get_logger(), "Safe return timed out before reaching idle");
      break;
    }
    next_update += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_period_seconds_));
    std::this_thread::sleep_until(next_update);
  }

  for (int index = 0; index < 10; ++index) {
    feedback_executor.spin_some();
    RefreshFeedback();
    SendTaskCommand(std::chrono::steady_clock::now(), control_period_seconds_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  feedback_executor.remove_node(shared_from_this());
  RCLCPP_INFO(get_logger(), "SkillTask safe return finished; shutting down");
  return skill_task_->IsIdle();
}

void SkillTaskNode::OnVelocityCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!IsFinite(message->linear.x) || !IsFinite(message->angular.z)) {
    RCLCPP_ERROR(get_logger(), "Rejected /cmd_vel containing a non-finite value");
    return;
  }
  if (skill_task_->GetState() != SkillTask::State::kStanding) {
    return;
  }
  desired_linear_velocity_ = message->linear.x;
  desired_yaw_velocity_ = message->angular.z;
  last_velocity_command_at_ = std::chrono::steady_clock::now();
  has_velocity_command_ = true;
}

void SkillTaskNode::OnSkillRequest(const std_msgs::msg::String::SharedPtr message)
{
  if (!has_low_state_) {
    RCLCPP_WARN(
      get_logger(), "Ignored skill request '%s' before the first valid low state",
      message->data.c_str());
    return;
  }
  const std::string request = NormalizeRequest(message->data);
  SkillTask::Target target;
  if (request == "stand") {
    target = SkillTask::Target::kStand;
  } else if (request == "idle" || request == "lie_down" || request == "stop") {
    target = SkillTask::Target::kIdle;
  } else {
    RCLCPP_WARN(get_logger(), "Rejected unknown skill request '%s'", message->data.c_str());
    return;
  }

  skill_task_->RequestTarget(target);
  RCLCPP_INFO(get_logger(), "Accepted skill target '%s'", SkillTask::TargetName(target));
}

void SkillTaskNode::OnControlTimer()
{
  if (RefreshFeedback() && !has_low_state_) {
    has_low_state_ = true;
    actuator_manager_->Reset();
    skill_task_->Reset();
    ResetExecutionState();
    last_control_time_ = last_feedback_time_;
    RCLCPP_INFO(
      get_logger(), "Received first low state; SkillTask is ready in '%s'",
      SkillTask::StateName(skill_task_->GetState()));
    PublishTaskState();
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
    skill_task_->ForceFault();
    const auto command = MakeDampingCommand(now);
    const auto output = actuator_manager_->Update(
      command, joint_state_frame_, now, control_period_seconds_);
    driver_->SendCommand(output.robot_command);
    actuator_manager_->Reset();
    ResetExecutionState();
    has_low_state_ = false;
    PublishTaskState();
    return;
  }

  const double measured_elapsed =
    std::chrono::duration<double>(now - last_control_time_).count();
  const double elapsed = measured_elapsed > 0.0 ? measured_elapsed : control_period_seconds_;
  last_control_time_ = now;
  SendTaskCommand(now, elapsed);
}

bool SkillTaskNode::RefreshFeedback()
{
  const auto feedback = driver_->GetFeedback();
  if (!feedback || feedback->received_at == last_feedback_time_) {
    return false;
  }
  joint_state_frame_ = actuator_manager_->DecodeFeedback(*feedback);
  last_feedback_time_ = feedback->received_at;
  return true;
}

bool SkillTaskNode::SendTaskCommand(
  const SteadyTimePoint now, const double elapsed_seconds)
{
  const bool feedback_valid = stand_skill_->IsFeedbackValid(joint_state_frame_);
  const SkillTask::Output task_output = skill_task_->Update(
    feedback_valid, last_action_result_);
  LogTaskOutput(task_output);
  if (!has_executed_action_ || task_output.action != last_executed_action_) {
    OnActionChanged(task_output.action);
    last_executed_action_ = task_output.action;
    has_executed_action_ = true;
  }

  const auto joint_command = ExecuteSelectedAction(
    task_output.action, now, elapsed_seconds);
  const auto actuator_output = actuator_manager_->Update(
    joint_command, joint_state_frame_, now, elapsed_seconds);
  robot_command_ = actuator_output.robot_command;
  if (!driver_->SendCommand(robot_command_)) {
    last_action_result_.failed = true;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "DDS driver rejected the actuator output");
    return false;
  }
  if (!actuator_output.all_requests_accepted) {
    last_action_result_.failed = true;
  }
  LogActuatorResult(actuator_output, joint_state_frame_);
  return actuator_output.all_requests_accepted;
}

actuator::JointCommandFrame SkillTaskNode::ExecuteSelectedAction(
  const SkillTask::Action action, const SteadyTimePoint now,
  const double elapsed_seconds)
{
  last_action_result_ = SkillTask::ActionResult{};
  const auto & profile = stand_skill_->GetConfig();
  switch (action) {
    case SkillTask::Action::kDamping:
      last_action_result_.completed = true;
      return MakeDampingCommand(now);
    case SkillTask::Action::kMoveToCrouch: {
        auto command = MakePostureCommand(
          profile.crouch_pose, profile.crouch_duration, false, now, elapsed_seconds);
        last_action_result_.completed =
          action_elapsed_seconds_ >= profile.crouch_duration &&
          stand_skill_->IsCrouchReached(joint_state_frame_);
        return command;
      }
    case SkillTask::Action::kRiseToStand: {
        auto command = MakePostureCommand(
          profile.stand_pose, profile.rise_duration, false, now, elapsed_seconds);
        last_action_result_.completed =
          action_elapsed_seconds_ >= profile.rise_duration &&
          stand_skill_->IsStandReached(joint_state_frame_);
        return command;
      }
    case SkillTask::Action::kHoldStand:
      return MakeJointCommand(profile.stand_pose, true, now, elapsed_seconds);
    case SkillTask::Action::kLieDown: {
        auto command = MakePostureCommand(
          profile.crouch_pose, profile.lie_down_duration, false, now, elapsed_seconds);
        last_action_result_.completed =
          action_elapsed_seconds_ >= profile.lie_down_duration &&
          stand_skill_->IsCrouchReached(joint_state_frame_);
        return command;
      }
  }
  last_action_result_.failed = true;
  return MakeDampingCommand(now);
}

actuator::JointCommandFrame SkillTaskNode::MakeDampingCommand(
  const SteadyTimePoint now)
{
  actuator::JointCommandFrame command;
  for (auto & joint : command.joints) {
    joint.control_mode = actuator::ControlMode::kDamping;
    joint.gain_profile = actuator::GainProfile::kSoft;
  }
  command.created_at = now;
  command.sequence = ++command_sequence_;
  return command;
}

actuator::JointCommandFrame SkillTaskNode::MakePostureCommand(
  const LegJointPositions & target, const double duration,
  const bool allow_wheel_motion, const SteadyTimePoint now,
  const double elapsed_seconds)
{
  action_elapsed_seconds_ += elapsed_seconds;
  desired_leg_pose_ = Interpolate(
    transition_start_pose_, target, SmoothStep(action_elapsed_seconds_ / duration));
  return MakeJointCommand(desired_leg_pose_, allow_wheel_motion, now, elapsed_seconds);
}

actuator::JointCommandFrame SkillTaskNode::MakeJointCommand(
  const LegJointPositions & leg_positions, const bool allow_wheel_motion,
  const SteadyTimePoint now, const double elapsed_seconds)
{
  actuator::JointCommandFrame command;
  for (std::size_t index = 0U; index < leg_positions.size(); ++index) {
    command.joints[index].control_mode = actuator::ControlMode::kPosition;
    command.joints[index].gain_profile = actuator::GainProfile::kNormal;
    command.joints[index].position = leg_positions[index];
  }

  const auto & profile = stand_skill_->GetConfig();
  double target_right = 0.0;
  double target_left = 0.0;
  if (allow_wheel_motion && has_velocity_command_) {
    const double command_age =
      std::chrono::duration<double>(now - last_velocity_command_at_).count();
    if (IsFinite(command_age) && command_age >= 0.0 &&
      command_age <= profile.velocity_timeout)
    {
      target_right =
        (desired_linear_velocity_ + 0.5 * profile.track_width * desired_yaw_velocity_) /
        profile.wheel_radius;
      target_left =
        (desired_linear_velocity_ - 0.5 * profile.track_width * desired_yaw_velocity_) /
        profile.wheel_radius;
      const double largest = std::max(std::abs(target_right), std::abs(target_left));
      if (largest > profile.max_wheel_speed) {
        const double scale = profile.max_wheel_speed / largest;
        target_right *= scale;
        target_left *= scale;
      }
    } else {
      desired_linear_velocity_ = 0.0;
      desired_yaw_velocity_ = 0.0;
      has_velocity_command_ = false;
    }
  }

  const double max_change = profile.wheel_acceleration * elapsed_seconds;
  right_wheel_velocity_ = Approach(right_wheel_velocity_, target_right, max_change);
  left_wheel_velocity_ = Approach(left_wheel_velocity_, target_left, max_change);
  const std::array<double, 4> wheel_velocities{{
    right_wheel_velocity_, left_wheel_velocity_,
    right_wheel_velocity_, left_wheel_velocity_}};
  for (std::size_t index = 0U; index < kWheelJointIds.size(); ++index) {
    auto & wheel = command.joints[actuator::ToIndex(kWheelJointIds[index])];
    wheel.control_mode = actuator::ControlMode::kVelocity;
    wheel.gain_profile = actuator::GainProfile::kNormal;
    wheel.velocity = wheel_velocities[index];
  }

  command.created_at = now;
  command.sequence = ++command_sequence_;
  return command;
}

SkillTaskNode::LegJointPositions SkillTaskNode::ReadLegPositions() const noexcept
{
  LegJointPositions positions{};
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    positions[index] = joint_state_frame_.joints[index].position;
  }
  return positions;
}

void SkillTaskNode::OnActionChanged(const SkillTask::Action action)
{
  last_action_result_ = SkillTask::ActionResult{};
  action_elapsed_seconds_ = 0.0;
  transition_start_pose_ = ReadLegPositions();
  desired_leg_pose_ = transition_start_pose_;
  right_wheel_velocity_ = joint_state_frame_.joints[
    actuator::ToIndex(actuator::JointId::kFrontRightWheel)].velocity;
  left_wheel_velocity_ = joint_state_frame_.joints[
    actuator::ToIndex(actuator::JointId::kFrontLeftWheel)].velocity;
  if (action != SkillTask::Action::kHoldStand) {
    desired_linear_velocity_ = 0.0;
    desired_yaw_velocity_ = 0.0;
    has_velocity_command_ = false;
  }
  RCLCPP_INFO(get_logger(), "Selected action: %s", SkillTask::ActionName(action));
}

void SkillTaskNode::ResetExecutionState() noexcept
{
  last_action_result_ = SkillTask::ActionResult{};
  transition_start_pose_.fill(0.0);
  desired_leg_pose_.fill(0.0);
  action_elapsed_seconds_ = 0.0;
  desired_linear_velocity_ = 0.0;
  desired_yaw_velocity_ = 0.0;
  right_wheel_velocity_ = 0.0;
  left_wheel_velocity_ = 0.0;
  has_executed_action_ = false;
  has_velocity_command_ = false;
  command_sequence_ = 0U;
  last_velocity_command_at_ = {};
}

void SkillTaskNode::LogTaskOutput(const SkillTask::Output & output)
{
  if (!output.state_changed && !output.action_changed) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "SkillTask state: %s, action: %s",
    SkillTask::StateName(output.state), SkillTask::ActionName(output.action));
  PublishTaskState();
}

void SkillTaskNode::LogActuatorResult(
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

void SkillTaskNode::PublishTaskState()
{
  std_msgs::msg::String message;
  message.data = SkillTask::StateName(skill_task_->GetState());
  skill_state_publisher_->publish(message);
}

SkillTaskNode::LegJointPositions SkillTaskNode::Interpolate(
  const LegJointPositions & from, const LegJointPositions & to,
  const double ratio) noexcept
{
  LegJointPositions result{};
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index] = from[index] + bounded_ratio * (to[index] - from[index]);
  }
  return result;
}

double SkillTaskNode::SmoothStep(const double ratio) noexcept
{
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  return bounded_ratio * bounded_ratio * (3.0 - 2.0 * bounded_ratio);
}

}  // namespace wheel_dog_mujoco::skill
