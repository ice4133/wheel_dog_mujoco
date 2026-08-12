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
    !IsFinite(config_.position_tolerance) || config_.position_tolerance <= 0.0 ||
    !IsFinite(config_.velocity_tolerance) || config_.velocity_tolerance < 0.0 ||
    !IsFinite(config_.velocity_timeout) || config_.velocity_timeout <= 0.0 ||
    !IsFinite(config_.wheel_radius) || config_.wheel_radius <= 0.0 ||
    !IsFinite(config_.track_width) || config_.track_width <= 0.0 ||
    !IsFinite(config_.max_wheel_speed) || config_.max_wheel_speed <= 0.0 ||
    !IsFinite(config_.wheel_acceleration) || config_.wheel_acceleration <= 0.0)
  {
    throw std::invalid_argument("StandSkill configuration is invalid");
  }
}

const StandSkill::Config & StandSkill::GetConfig() const noexcept
{
  return config_;
}

bool StandSkill::IsFeedbackValid(
  const actuator::JointStateFrame & state) const noexcept
{
  return std::all_of(
    state.joints.begin(), state.joints.end(),
    [](const actuator::JointState & joint) {
      return joint.online && IsFinite(joint.position) && IsFinite(joint.velocity);
    });
}

bool StandSkill::IsCrouchReached(
  const actuator::JointStateFrame & state) const noexcept
{
  return IsPoseReached(state, config_.crouch_pose);
}

bool StandSkill::IsStandReached(
  const actuator::JointStateFrame & state) const noexcept
{
  return IsPoseReached(state, config_.stand_pose);
}

bool StandSkill::IsPoseReached(
  const actuator::JointStateFrame & state,
  const LegJointPositions & target) const noexcept
{
  if (!IsFeedbackValid(state)) {
    return false;
  }
  for (std::size_t index = 0U; index < target.size(); ++index) {
    const auto & joint = state.joints[index];
    if (std::abs(joint.position - target[index]) > config_.position_tolerance ||
      std::abs(joint.velocity) > config_.velocity_tolerance)
    {
      return false;
    }
  }
  return true;
}

}  // namespace wheel_dog_mujoco::skill
