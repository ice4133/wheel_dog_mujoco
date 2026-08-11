#include "wheel_dog_mujoco/actuator/actuator_safety.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::actuator
{
namespace
{

bool HasAnyFault(const FaultFlags flags, const FaultFlags mask) noexcept
{
  return (flags & mask) != 0U;
}

double LimitValue(
  const double requested, const double minimum, const double maximum,
  FaultFlags & faults, const Fault fault, bool & limited) noexcept
{
  const double result = std::clamp(requested, minimum, maximum);
  if (result != requested) {
    faults |= ToMask(fault);
    limited = true;
  }
  return result;
}

}  // namespace

ActuatorSafety::ActuatorSafety(const ActuatorConfig & config)
: config_(config)
{
  if (ToIndex(config_.joint_id) >= kActuatorCount) {
    throw std::invalid_argument("ActuatorSafety received an invalid JointId");
  }
  if (!std::isfinite(config_.safety.command_timeout) ||
    config_.safety.command_timeout <= 0.0 ||
    !std::isfinite(config_.safety.feedback_timeout) ||
    config_.safety.feedback_timeout <= 0.0 ||
    !std::isfinite(config_.safety.position_limit_tolerance) ||
    config_.safety.position_limit_tolerance < 0.0 ||
    !std::isfinite(config_.safety.velocity_limit_tolerance) ||
    config_.safety.velocity_limit_tolerance < 0.0 ||
    !std::isfinite(config_.limits.max_velocity) || config_.limits.max_velocity <= 0.0 ||
    !std::isfinite(config_.limits.max_acceleration) || config_.limits.max_acceleration <= 0.0 ||
    !std::isfinite(config_.limits.max_torque) || config_.limits.max_torque <= 0.0 ||
    !std::isfinite(config_.controller.max_time_step) || config_.controller.max_time_step <= 0.0)
  {
    throw std::invalid_argument("ActuatorSafety configuration is invalid");
  }
}

SafetyOutput ActuatorSafety::Update(
  const JointCommand & requested_command, const JointState & state,
  const SafetyTiming & timing)
{
  faults_ = ToMask(Fault::kNone);

  if (!IsTimingValid(timing)) {
    faults_ |= ToMask(Fault::kInvalidCommand);
    return MakeFallback(ControlMode::kDamping, faults_, state);
  }

  faults_ |= CheckFeedback(state, timing);
  if (IsCriticalFeedbackFault(faults_)) {
    return MakeFallback(ControlMode::kDisabled, faults_, state);
  }
  // A position-mode command is able to recover a joint that has crossed a hard
  // stop: its target is clamped below and then rate-limited back into range.
  // Falling back to damping here would leave the joint outside the range forever.
  const FaultFlags non_position_faults = faults_ & ~ToMask(Fault::kPositionLimit);
  const bool can_recover_position =
    requested_command.control_mode == ControlMode::kPosition ||
    requested_command.control_mode == ControlMode::kHybrid;
  if (non_position_faults != ToMask(Fault::kNone) ||
    (HasFault(faults_, Fault::kPositionLimit) && !can_recover_position))
  {
    return MakeFallback(ControlMode::kDamping, faults_, state);
  }

  if (timing.command_age > config_.safety.command_timeout) {
    faults_ |= ToMask(Fault::kCommandTimeout);
    return MakeFallback(ControlMode::kDamping, faults_, state);
  }
  if (!IsCommandValid(requested_command)) {
    faults_ |= ToMask(Fault::kInvalidCommand);
    return MakeFallback(ControlMode::kDamping, faults_, state);
  }

  SafetyOutput output;
  output.command = requested_command;
  output.request_accepted = true;
  bool limited = false;
  const double bounded_dt = std::min(timing.elapsed_seconds, config_.controller.max_time_step);
  if (bounded_dt < timing.elapsed_seconds) {
    limited = true;
  }

  if (!initialized_ || requested_command.control_mode != previous_command_.control_mode) {
    previous_command_ = requested_command;
    previous_command_.position = state.position;
    previous_command_.velocity = state.velocity;
    previous_command_.torque_feedforward = 0.0;
    previous_position_rate_ = 0.0;
    initialized_ = true;
  }

  if (config_.limits.position_limited &&
    (requested_command.control_mode == ControlMode::kPosition ||
    requested_command.control_mode == ControlMode::kHybrid))
  {
    output.command.position = LimitValue(
      output.command.position, config_.limits.min_position, config_.limits.max_position,
      faults_, Fault::kPositionLimit, limited);
  }

  if (requested_command.control_mode == ControlMode::kPosition ||
    requested_command.control_mode == ControlMode::kHybrid)
  {
    const double requested_position = output.command.position;
    const double requested_rate =
      (requested_position - previous_command_.position) / bounded_dt;
    const double velocity_limited_rate = LimitValue(
      requested_rate, -config_.limits.max_velocity, config_.limits.max_velocity,
      faults_, Fault::kVelocityLimit, limited);
    const double max_rate_change = config_.limits.max_acceleration * bounded_dt;
    const double safe_rate = LimitValue(
      velocity_limited_rate,
      previous_position_rate_ - max_rate_change,
      previous_position_rate_ + max_rate_change,
      faults_, Fault::kVelocityLimit, limited);
    const double next_position = previous_command_.position + safe_rate * bounded_dt;
    const bool would_cross_target =
      (requested_position - previous_command_.position) * (requested_position - next_position) < 0.0;
    output.command.position = would_cross_target ? requested_position : next_position;
    previous_position_rate_ = would_cross_target ? 0.0 : safe_rate;
  } else {
    previous_position_rate_ = 0.0;
  }

  if (requested_command.control_mode == ControlMode::kVelocity ||
    requested_command.control_mode == ControlMode::kHybrid)
  {
    output.command.velocity = LimitValue(
      output.command.velocity, -config_.limits.max_velocity, config_.limits.max_velocity,
      faults_, Fault::kVelocityLimit, limited);
    const double max_velocity_change = config_.limits.max_acceleration * bounded_dt;
    output.command.velocity = LimitValue(
      output.command.velocity,
      previous_command_.velocity - max_velocity_change,
      previous_command_.velocity + max_velocity_change,
      faults_, Fault::kVelocityLimit, limited);
  }

  output.command.torque_feedforward = LimitValue(
    output.command.torque_feedforward,
    -config_.limits.max_torque, config_.limits.max_torque,
    faults_, Fault::kTorqueLimit, limited);

  if (requested_command.control_mode == ControlMode::kDisabled) {
    output.command.position = state.position;
    output.command.velocity = 0.0;
    output.command.torque_feedforward = 0.0;
  } else if (requested_command.control_mode == ControlMode::kDamping) {
    output.command.gain_profile = GainProfile::kSoft;
    output.command.position = state.position;
    output.command.velocity = 0.0;
    output.command.torque_feedforward = 0.0;
  }

  output.status.active_mode = output.command.control_mode;
  output.status.faults = faults_;
  output.status.command_limited = limited;
  previous_command_ = output.command;
  return output;
}

void ActuatorSafety::Reset() noexcept
{
  previous_command_ = JointCommand{};
  previous_position_rate_ = 0.0;
  faults_ = ToMask(Fault::kNone);
  initialized_ = false;
}

JointId ActuatorSafety::GetJointId() const noexcept
{
  return config_.joint_id;
}

FaultFlags ActuatorSafety::GetFaults() const noexcept
{
  return faults_;
}

SafetyOutput ActuatorSafety::MakeFallback(
  const ControlMode fallback_mode, const FaultFlags faults, const JointState & state) noexcept
{
  SafetyOutput output;
  output.command.control_mode = fallback_mode;
  output.command.gain_profile = GainProfile::kSoft;
  output.command.position = std::isfinite(state.position) ? state.position : 0.0;
  output.command.velocity = 0.0;
  output.command.torque_feedforward = 0.0;
  output.status.active_mode = fallback_mode;
  output.status.faults = faults;
  output.status.command_limited = true;
  output.request_accepted = false;
  previous_command_ = output.command;
  previous_position_rate_ = 0.0;
  initialized_ = false;
  return output;
}

FaultFlags ActuatorSafety::CheckFeedback(
  const JointState & state, const SafetyTiming & timing) const noexcept
{
  FaultFlags faults = ToMask(Fault::kNone);
  if (timing.feedback_age > config_.safety.feedback_timeout) {
    faults |= ToMask(Fault::kFeedbackTimeout);
  }
  if (!state.online) {
    faults |= ToMask(Fault::kMotorOffline);
  }
  if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
    !std::isfinite(state.acceleration) || !std::isfinite(state.estimated_torque) ||
    !std::isfinite(state.temperature))
  {
    faults |= ToMask(Fault::kInvalidFeedback);
    return faults;
  }
  if (config_.limits.position_limited &&
    (state.position <
    config_.limits.min_position - config_.safety.position_limit_tolerance ||
    state.position > config_.limits.max_position + config_.safety.position_limit_tolerance))
  {
    faults |= ToMask(Fault::kPositionLimit);
  }
  if (std::abs(state.velocity) >
    config_.limits.max_velocity + config_.safety.velocity_limit_tolerance)
  {
    faults |= ToMask(Fault::kVelocityLimit);
  }
  if (std::abs(state.estimated_torque) > config_.limits.max_torque) {
    faults |= ToMask(Fault::kTorqueLimit);
  }
  if (state.temperature > config_.limits.max_temperature) {
    faults |= ToMask(Fault::kOverTemperature);
  }
  return faults;
}

bool ActuatorSafety::IsCommandValid(const JointCommand & command) noexcept
{
  if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
    !std::isfinite(command.torque_feedforward))
  {
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

bool ActuatorSafety::IsTimingValid(const SafetyTiming & timing) noexcept
{
  return std::isfinite(timing.elapsed_seconds) && timing.elapsed_seconds > 0.0 &&
         std::isfinite(timing.command_age) && timing.command_age >= 0.0 &&
         std::isfinite(timing.feedback_age) && timing.feedback_age >= 0.0;
}

bool ActuatorSafety::IsCriticalFeedbackFault(const FaultFlags faults) noexcept
{
  constexpr FaultFlags critical_faults =
    ToMask(Fault::kFeedbackTimeout) | ToMask(Fault::kMotorOffline) |
    ToMask(Fault::kInvalidFeedback) | ToMask(Fault::kOverTemperature);
  return HasAnyFault(faults, critical_faults);
}

}  // namespace wheel_dog_mujoco::actuator
