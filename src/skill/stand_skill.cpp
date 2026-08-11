#include "wheel_dog_mujoco/skill/stand_skill.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

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

bool IsFinite(const double value) noexcept
{
  return std::isfinite(value);
}

double Approach(const double current, const double target, const double max_change) noexcept
{
  return current + std::clamp(target - current, -max_change, max_change);
}

}  // namespace

StandSkill::StandSkill(const Config & config)
: config_(config)
{
  const bool poses_are_finite = std::all_of(
    config_.crouch_pose.begin(), config_.crouch_pose.end(), IsFinite) &&
    std::all_of(config_.stand_pose.begin(), config_.stand_pose.end(), IsFinite);
  if (!poses_are_finite ||
    !IsFinite(config_.crouch_duration) || config_.crouch_duration <= 0.0 ||
    !IsFinite(config_.rise_duration) || config_.rise_duration <= 0.0 ||
    !IsFinite(config_.lie_down_duration) || config_.lie_down_duration <= 0.0 ||
    !IsFinite(config_.velocity_timeout) || config_.velocity_timeout <= 0.0 ||
    !IsFinite(config_.wheel_radius) || config_.wheel_radius <= 0.0 ||
    !IsFinite(config_.track_width) || config_.track_width <= 0.0 ||
    !IsFinite(config_.max_wheel_speed) || config_.max_wheel_speed <= 0.0 ||
    !IsFinite(config_.wheel_acceleration) || config_.wheel_acceleration <= 0.0)
  {
    throw std::invalid_argument("StandSkill configuration is invalid");
  }
}

actuator::JointCommandFrame StandSkill::Update(
  const actuator::JointStateFrame & state,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  if (!IsStateValid(state) || !IsFinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return MakeDampingCommand(now);
  }

  const LegJointPositions measured_pose = ReadLegPositions(state);
  if (!initialized_) {
    transition_start_pose_ = measured_pose;
    desired_leg_pose_ = measured_pose;
    right_wheel_velocity_ = state.joints[
      actuator::ToIndex(actuator::JointId::kFrontRightWheel)].velocity;
    left_wheel_velocity_ = state.joints[
      actuator::ToIndex(actuator::JointId::kFrontLeftWheel)].velocity;
    initialized_ = true;
    StartPhase(
      lie_down_requested_ ? Phase::kLyingDown : Phase::kMovingToCrouch,
      measured_pose);
  }

  if (lie_down_requested_ && phase_ != Phase::kLyingDown && phase_ != Phase::kLying) {
    StartPhase(Phase::kLyingDown, measured_pose);
  }

  phase_elapsed_seconds_ += elapsed_seconds;
  switch (phase_) {
    case Phase::kWaitingForState:
      return MakeDampingCommand(now);
    case Phase::kMovingToCrouch: {
        const double ratio = phase_elapsed_seconds_ / config_.crouch_duration;
        desired_leg_pose_ = Interpolate(
          transition_start_pose_, config_.crouch_pose, SmoothStep(ratio));
        if (ratio >= 1.0) {
          desired_leg_pose_ = config_.crouch_pose;
          StartPhase(Phase::kRising, config_.crouch_pose);
        }
        break;
      }
    case Phase::kRising: {
        const double ratio = phase_elapsed_seconds_ / config_.rise_duration;
        desired_leg_pose_ = Interpolate(
          transition_start_pose_, config_.stand_pose, SmoothStep(ratio));
        if (ratio >= 1.0) {
          desired_leg_pose_ = config_.stand_pose;
          StartPhase(Phase::kHolding, config_.stand_pose);
        }
        break;
      }
    case Phase::kHolding:
      desired_leg_pose_ = config_.stand_pose;
      break;
    case Phase::kLyingDown: {
        const double ratio = phase_elapsed_seconds_ / config_.lie_down_duration;
        desired_leg_pose_ = Interpolate(
          transition_start_pose_, config_.crouch_pose, SmoothStep(ratio));
        if (ratio >= 1.0) {
          desired_leg_pose_ = config_.crouch_pose;
          StartPhase(Phase::kLying, config_.crouch_pose);
        }
        break;
      }
    case Phase::kLying:
      desired_leg_pose_ = config_.crouch_pose;
      break;
  }
  return MakeJointCommand(desired_leg_pose_, now, elapsed_seconds);
}

void StandSkill::SetVelocityTarget(
  const double linear_velocity, const double yaw_velocity,
  const std::chrono::steady_clock::time_point received_at) noexcept
{
  if (!IsFinite(linear_velocity) || !IsFinite(yaw_velocity)) {
    return;
  }
  desired_linear_velocity_ = linear_velocity;
  desired_yaw_velocity_ = yaw_velocity;
  last_velocity_command_at_ = received_at;
  has_velocity_command_ = true;
}

void StandSkill::RequestLieDown() noexcept
{
  lie_down_requested_ = true;
  desired_linear_velocity_ = 0.0;
  desired_yaw_velocity_ = 0.0;
  has_velocity_command_ = false;
}

void StandSkill::Reset() noexcept
{
  transition_start_pose_.fill(0.0);
  desired_leg_pose_.fill(0.0);
  phase_ = Phase::kWaitingForState;
  desired_linear_velocity_ = 0.0;
  desired_yaw_velocity_ = 0.0;
  right_wheel_velocity_ = 0.0;
  left_wheel_velocity_ = 0.0;
  phase_elapsed_seconds_ = 0.0;
  last_velocity_command_at_ = {};
  sequence_ = 0U;
  has_velocity_command_ = false;
  lie_down_requested_ = false;
  initialized_ = false;
}

StandSkill::Phase StandSkill::GetPhase() const noexcept
{
  return phase_;
}

bool StandSkill::IsStanding() const noexcept
{
  return phase_ == Phase::kHolding;
}

bool StandSkill::IsLying() const noexcept
{
  return phase_ == Phase::kLying;
}

const char * StandSkill::PhaseName(const Phase phase) noexcept
{
  switch (phase) {
    case Phase::kWaitingForState:
      return "waiting for state";
    case Phase::kMovingToCrouch:
      return "moving to crouch";
    case Phase::kRising:
      return "rising";
    case Phase::kHolding:
      return "standing";
    case Phase::kLyingDown:
      return "lying down";
    case Phase::kLying:
      return "lying";
  }
  return "unknown";
}

bool StandSkill::IsStateValid(const actuator::JointStateFrame & state) const noexcept
{
  return std::all_of(
    state.joints.begin(), state.joints.end(),
    [](const actuator::JointState & joint) {
      return joint.online && IsFinite(joint.position) && IsFinite(joint.velocity);
    });
}

StandSkill::LegJointPositions StandSkill::ReadLegPositions(
  const actuator::JointStateFrame & state) const noexcept
{
  LegJointPositions positions{};
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    positions[index] = state.joints[index].position;
  }
  return positions;
}

actuator::JointCommandFrame StandSkill::MakeDampingCommand(
  const std::chrono::steady_clock::time_point now)
{
  actuator::JointCommandFrame command;
  for (auto & joint : command.joints) {
    joint.control_mode = actuator::ControlMode::kDamping;
    joint.gain_profile = actuator::GainProfile::kSoft;
  }
  command.created_at = now;
  command.sequence = ++sequence_;
  return command;
}

actuator::JointCommandFrame StandSkill::MakeJointCommand(
  const LegJointPositions & leg_positions,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  actuator::JointCommandFrame command;
  for (std::size_t index = 0U; index < leg_positions.size(); ++index) {
    command.joints[index].control_mode = actuator::ControlMode::kPosition;
    command.joints[index].gain_profile = actuator::GainProfile::kNormal;
    command.joints[index].position = leg_positions[index];
  }

  double target_right = 0.0;
  double target_left = 0.0;
  if (phase_ == Phase::kHolding && has_velocity_command_) {
    const double command_age =
      std::chrono::duration<double>(now - last_velocity_command_at_).count();
    if (IsFinite(command_age) && command_age >= 0.0 &&
      command_age <= config_.velocity_timeout)
    {
      target_right =
        (desired_linear_velocity_ + 0.5 * config_.track_width * desired_yaw_velocity_) /
        config_.wheel_radius;
      target_left =
        (desired_linear_velocity_ - 0.5 * config_.track_width * desired_yaw_velocity_) /
        config_.wheel_radius;
      const double largest = std::max(std::abs(target_right), std::abs(target_left));
      if (largest > config_.max_wheel_speed) {
        const double scale = config_.max_wheel_speed / largest;
        target_right *= scale;
        target_left *= scale;
      }
    } else {
      desired_linear_velocity_ = 0.0;
      desired_yaw_velocity_ = 0.0;
      has_velocity_command_ = false;
    }
  }

  const double max_change = config_.wheel_acceleration * elapsed_seconds;
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
  command.sequence = ++sequence_;
  return command;
}

void StandSkill::StartPhase(
  const Phase phase, const LegJointPositions & transition_start) noexcept
{
  phase_ = phase;
  transition_start_pose_ = transition_start;
  phase_elapsed_seconds_ = 0.0;
}

StandSkill::LegJointPositions StandSkill::Interpolate(
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

double StandSkill::SmoothStep(const double ratio) noexcept
{
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  return bounded_ratio * bounded_ratio * (3.0 - 2.0 * bounded_ratio);
}

}  // namespace wheel_dog_mujoco::skill
