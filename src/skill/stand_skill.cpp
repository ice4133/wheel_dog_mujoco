#include "wheel_dog_mujoco/skill/stand_skill.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::skill
{
namespace
{

bool IsFinite(const double value) noexcept
{
  return std::isfinite(value);
}

body::Vector3 QuaternionToRpy(const body::Quaternion & q) noexcept
{
  return body::Vector3{
    std::atan2(
      2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
    std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)),
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

}  // namespace

StandSkill::StandSkill(const Config & config)
: config_(config)
{
  if (!IsFinite(config_.recovery_ground_clearance) ||
    config_.recovery_ground_clearance <= 0.0 ||
    !IsFinite(config_.stand_ground_clearance) ||
    config_.stand_ground_clearance < config_.recovery_ground_clearance ||
    !IsFinite(config_.lie_down_ground_clearance) ||
    config_.lie_down_ground_clearance <= 0.0 ||
    config_.lie_down_ground_clearance > config_.stand_ground_clearance ||
    !IsFinite(config_.height_tolerance) || config_.height_tolerance <= 0.0 ||
    !IsFinite(config_.attitude_tolerance) || config_.attitude_tolerance <= 0.0 ||
    !IsFinite(config_.angular_velocity_tolerance) ||
    config_.angular_velocity_tolerance <= 0.0 ||
    !IsFinite(config_.settle_duration) || config_.settle_duration < 0.0 ||
    !IsFinite(config_.velocity_timeout) || config_.velocity_timeout <= 0.0)
  {
    throw std::invalid_argument("StandSkill configuration is invalid");
  }
}

body::BodyCommandFrame StandSkill::Update(
  const body::BodyStateFrame & body_state,
  const std::chrono::steady_clock::time_point now)
{
  if (!body_state.state.valid || !body_state.status.state_estimate_valid) {
    SetPhase(Phase::kWaitingForState);
    return MakeDisabledCommand(now);
  }

  if (lie_down_requested_ && phase_ != Phase::kLyingDown && phase_ != Phase::kLying) {
    SetPhase(Phase::kLyingDown);
  } else if (phase_ == Phase::kWaitingForState) {
    SetPhase(lie_down_requested_ ? Phase::kLyingDown : Phase::kRecovering);
  }

  switch (phase_) {
    case Phase::kWaitingForState:
      return MakeDisabledCommand(now);
    case Phase::kRecovering:
      if (IsPostureSettled(
          body_state.state, config_.recovery_ground_clearance, now))
      {
        SetPhase(Phase::kRising);
      }
      return MakeHeightCommand(config_.recovery_ground_clearance, false, now);
    case Phase::kRising:
      if (IsPostureSettled(body_state.state, config_.stand_ground_clearance, now)) {
        SetPhase(Phase::kHolding);
      }
      return MakeHeightCommand(config_.stand_ground_clearance, false, now);
    case Phase::kHolding:
      return MakeHeightCommand(config_.stand_ground_clearance, true, now);
    case Phase::kLyingDown:
      if (IsPostureSettled(body_state.state, config_.lie_down_ground_clearance, now)) {
        SetPhase(Phase::kLying);
      }
      return MakeHeightCommand(config_.lie_down_ground_clearance, false, now);
    case Phase::kLying:
      return MakeHeightCommand(config_.lie_down_ground_clearance, false, now);
  }
  return MakeDisabledCommand(now);
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
  if (phase_ != Phase::kWaitingForState && phase_ != Phase::kLying) {
    SetPhase(Phase::kLyingDown);
  }
}

void StandSkill::Reset() noexcept
{
  phase_ = Phase::kWaitingForState;
  desired_linear_velocity_ = 0.0;
  desired_yaw_velocity_ = 0.0;
  last_velocity_command_at_ = {};
  settled_since_ = {};
  sequence_ = 0U;
  has_velocity_command_ = false;
  lie_down_requested_ = false;
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
    case Phase::kRecovering:
      return "recovering to crouch";
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

body::BodyCommandFrame StandSkill::MakeDisabledCommand(
  const std::chrono::steady_clock::time_point now)
{
  body::BodyCommandFrame command;
  command.command.control_mode = body::ControlMode::kDisabled;
  command.created_at = now;
  command.sequence = ++sequence_;
  return command;
}

body::BodyCommandFrame StandSkill::MakeHeightCommand(
  const double ground_clearance, const bool allow_driving,
  const std::chrono::steady_clock::time_point now)
{
  body::BodyCommandFrame command;
  command.command.control_mode = body::ControlMode::kHybrid;
  command.command.pose_frame = body::ReferenceFrame::kWorld;
  command.command.twist_frame = body::ReferenceFrame::kBody;
  command.command.desired_pose.position.z = ground_clearance;
  command.command.desired_pose.orientation = body::Quaternion{};

  if (allow_driving && has_velocity_command_) {
    const double command_age =
      std::chrono::duration<double>(now - last_velocity_command_at_).count();
    if (IsFinite(command_age) && command_age >= 0.0 &&
      command_age <= config_.velocity_timeout)
    {
      command.command.desired_twist.linear.x = desired_linear_velocity_;
      command.command.desired_twist.angular.z = desired_yaw_velocity_;
    } else {
      desired_linear_velocity_ = 0.0;
      desired_yaw_velocity_ = 0.0;
      has_velocity_command_ = false;
    }
  }

  command.created_at = now;
  command.sequence = ++sequence_;
  return command;
}

bool StandSkill::IsPostureSettled(
  const body::BodyState & state, const double target_ground_clearance,
  const std::chrono::steady_clock::time_point now) noexcept
{
  const body::Vector3 rpy = QuaternionToRpy(state.pose_world.orientation);
  const bool settled =
    std::abs(state.ground_clearance - target_ground_clearance) <=
    config_.height_tolerance &&
    std::abs(rpy.x) <= config_.attitude_tolerance &&
    std::abs(rpy.y) <= config_.attitude_tolerance &&
    std::abs(state.twist_body.angular.x) <= config_.angular_velocity_tolerance &&
    std::abs(state.twist_body.angular.y) <= config_.angular_velocity_tolerance;
  if (!settled) {
    settled_since_ = {};
    return false;
  }
  if (settled_since_ == std::chrono::steady_clock::time_point{}) {
    settled_since_ = now;
  }
  return std::chrono::duration<double>(now - settled_since_).count() >=
         config_.settle_duration;
}

void StandSkill::SetPhase(const Phase phase) noexcept
{
  if (phase_ == phase) {
    return;
  }
  phase_ = phase;
  settled_since_ = {};
}

}  // namespace wheel_dog_mujoco::skill
