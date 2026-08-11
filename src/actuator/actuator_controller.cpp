#include "wheel_dog_mujoco/actuator/actuator_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::actuator
{

ActuatorController::ActuatorController(const ActuatorConfig & config)
: config_(config)
{
  if (ToIndex(config_.joint_id) >= kActuatorCount) {
    throw std::invalid_argument("ActuatorController received an invalid JointId");
  }
  if (!std::isfinite(config_.controller.mode_transition_duration) ||
    config_.controller.mode_transition_duration < 0.0 ||
    !std::isfinite(config_.controller.max_time_step) || config_.controller.max_time_step <= 0.0)
  {
    throw std::invalid_argument("ActuatorController timing parameters are invalid");
  }
}

ControllerOutput ActuatorController::Update(
  const JointCommand & requested_command, const JointState & state,
  const double elapsed_seconds)
{
  if (!IsCommandValid(requested_command)) {
    return MakeRejectedOutput(Fault::kInvalidCommand);
  }
  if (!IsStateValid(state)) {
    return MakeRejectedOutput(state.online ? Fault::kInvalidFeedback : Fault::kMotorOffline);
  }
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return MakeRejectedOutput(Fault::kInvalidCommand);
  }

  ControllerOutput output;
  output.command = requested_command;
  output.gains = SelectGains(requested_command.gain_profile);
  output.accepted = true;

  const double bounded_dt = std::min(elapsed_seconds, config_.controller.max_time_step);
  if (bounded_dt < elapsed_seconds) {
    output.status.command_limited = true;
  }

  if (!initialized_ || requested_command.control_mode != active_mode_ ||
    requested_command.gain_profile != active_gain_profile_)
  {
    StartTransition(requested_command, state);
  }

  output.status.active_mode = active_mode_;
  if (active_mode_ == ControlMode::kDisabled) {
    output.command.control_mode = ControlMode::kDisabled;
    output.command.position = state.position;
    output.command.velocity = 0.0;
    output.command.torque_feedforward = 0.0;
    output.gains = ControlGains{};
    integral_error_ = 0.0;
    return output;
  }

  transition_elapsed_ += bounded_dt;
  const double duration = config_.controller.mode_transition_duration;
  const double transition_ratio = duration > 0.0 ?
    SmoothStep(transition_elapsed_ / duration) : 1.0;
  if (transition_ratio < 1.0) {
    output.status.command_limited = true;
  }

  switch (active_mode_) {
    case ControlMode::kDisabled:
      break;
    case ControlMode::kDamping:
      output.command.position = state.position;
      output.command.velocity = 0.0;
      output.command.torque_feedforward = 0.0;
      integral_error_ = 0.0;
      break;
    case ControlMode::kTorque:
      output.command.position = state.position;
      output.command.velocity = 0.0;
      output.command.torque_feedforward =
        transition_ratio * requested_command.torque_feedforward;
      integral_error_ = 0.0;
      break;
    case ControlMode::kVelocity:
      output.command.position = state.position;
      output.command.velocity = Interpolate(
        transition_start_velocity_, requested_command.velocity, transition_ratio);
      output.command.torque_feedforward =
        transition_ratio * requested_command.torque_feedforward;
      break;
    case ControlMode::kPosition:
      output.command.position = Interpolate(
        transition_start_position_, requested_command.position, transition_ratio);
      output.command.velocity = 0.0;
      output.command.torque_feedforward =
        transition_ratio * requested_command.torque_feedforward;
      break;
    case ControlMode::kHybrid:
      output.command.position = Interpolate(
        transition_start_position_, requested_command.position, transition_ratio);
      output.command.velocity = Interpolate(
        transition_start_velocity_, requested_command.velocity, transition_ratio);
      output.command.torque_feedforward =
        transition_ratio * requested_command.torque_feedforward;
      break;
  }

  double control_error = 0.0;
  if (active_mode_ == ControlMode::kPosition || active_mode_ == ControlMode::kHybrid) {
    control_error = output.command.position - state.position;
  } else if (active_mode_ == ControlMode::kVelocity) {
    control_error = output.command.velocity - state.velocity;
  } else {
    integral_error_ = 0.0;
  }

  if (output.gains.ki > 0.0 && output.gains.integral_limit > 0.0 &&
    (active_mode_ == ControlMode::kPosition || active_mode_ == ControlMode::kVelocity ||
    active_mode_ == ControlMode::kHybrid))
  {
    const double candidate_integral = std::clamp(
      integral_error_ + control_error * bounded_dt,
      -output.gains.integral_limit, output.gains.integral_limit);
    const double candidate_torque =
      output.command.torque_feedforward + output.gains.ki * candidate_integral;
    if (std::abs(candidate_torque) <= config_.limits.max_torque) {
      integral_error_ = candidate_integral;
    } else {
      output.status.command_limited = true;
    }
  }

  const double requested_torque =
    output.command.torque_feedforward + output.gains.ki * integral_error_;
  output.command.torque_feedforward = std::clamp(
    requested_torque, -config_.limits.max_torque, config_.limits.max_torque);
  if (output.command.torque_feedforward != requested_torque) {
    output.status.command_limited = true;
  }
  return output;
}

void ActuatorController::Reset() noexcept
{
  active_mode_ = ControlMode::kDisabled;
  active_gain_profile_ = GainProfile::kNormal;
  integral_error_ = 0.0;
  transition_elapsed_ = 0.0;
  transition_start_position_ = 0.0;
  transition_start_velocity_ = 0.0;
  initialized_ = false;
}

JointId ActuatorController::GetJointId() const noexcept
{
  return config_.joint_id;
}

ControlMode ActuatorController::GetActiveMode() const noexcept
{
  return active_mode_;
}

double ActuatorController::GetIntegralError() const noexcept
{
  return integral_error_;
}

const ControlGains & ActuatorController::SelectGains(const GainProfile profile) const
{
  switch (profile) {
    case GainProfile::kSoft:
      return config_.gains.soft;
    case GainProfile::kNormal:
      return config_.gains.normal;
    case GainProfile::kStiff:
      return config_.gains.stiff;
  }
  throw std::invalid_argument("Unknown GainProfile");
}

void ActuatorController::StartTransition(
  const JointCommand & command, const JointState & state) noexcept
{
  active_mode_ = command.control_mode;
  active_gain_profile_ = command.gain_profile;
  integral_error_ = 0.0;
  transition_elapsed_ = 0.0;
  transition_start_position_ = state.position;
  transition_start_velocity_ = state.velocity;
  initialized_ = true;
}

ControllerOutput ActuatorController::MakeRejectedOutput(const Fault fault) noexcept
{
  Reset();
  ControllerOutput output;
  output.command.control_mode = ControlMode::kDisabled;
  output.status.active_mode = ControlMode::kDisabled;
  output.status.faults = ToMask(fault);
  output.accepted = false;
  return output;
}

bool ActuatorController::IsCommandValid(const JointCommand & command) noexcept
{
  const bool values_are_finite =
    std::isfinite(command.position) && std::isfinite(command.velocity) &&
    std::isfinite(command.torque_feedforward);
  if (!values_are_finite) {
    return false;
  }
  bool mode_is_valid = true;
  switch (command.control_mode) {
    case ControlMode::kDisabled:
    case ControlMode::kDamping:
    case ControlMode::kTorque:
    case ControlMode::kVelocity:
    case ControlMode::kPosition:
    case ControlMode::kHybrid:
      break;
    default:
      mode_is_valid = false;
      break;
  }
  if (!mode_is_valid) {
    return false;
  }
  switch (command.gain_profile) {
    case GainProfile::kSoft:
    case GainProfile::kNormal:
    case GainProfile::kStiff:
      return true;
  }
  return false;
}

bool ActuatorController::IsStateValid(const JointState & state) noexcept
{
  return state.online && std::isfinite(state.position) && std::isfinite(state.velocity) &&
         std::isfinite(state.acceleration) && std::isfinite(state.estimated_torque) &&
         std::isfinite(state.temperature);
}

double ActuatorController::SmoothStep(const double ratio) noexcept
{
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  return bounded_ratio * bounded_ratio * (3.0 - 2.0 * bounded_ratio);
}

double ActuatorController::Interpolate(
  const double from, const double to, const double ratio) noexcept
{
  return from + ratio * (to - from);
}

}  // namespace wheel_dog_mujoco::actuator
