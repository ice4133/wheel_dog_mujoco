#include "wheel_dog_mujoco/stand_controller.h"

#include <algorithm>
#include <stdexcept>

namespace wheel_dog_mujoco
{

StandController::StandController()
{
  ValidateConfig(config_);
}

StandController::StandController(const Config & config)
: config_(config)
{
  ValidateConfig(config_);
}

void StandController::SetConfig(const Config & config)
{
  ValidateConfig(config);
  config_ = config;
  Reset();
}

const StandController::Config & StandController::GetConfig() const noexcept
{
  return config_;
}

void StandController::Start(const JointPositions & current_pose)
{
  initial_pose_ = current_pose;
  desired_pose_ = current_pose;
  elapsed_seconds_ = 0.0;
  state_ = State::kMovingToCrouch;
}

void StandController::Reset() noexcept
{
  initial_pose_.fill(0.0);
  desired_pose_.fill(0.0);
  elapsed_seconds_ = 0.0;
  state_ = State::kIdle;
}

void StandController::Update(const double elapsed_seconds)
{
  if (state_ == State::kIdle || state_ == State::kHolding) {
    return;
  }

  elapsed_seconds_ += std::max(0.0, elapsed_seconds);

  if (elapsed_seconds_ < config_.crouch_duration) {
    state_ = State::kMovingToCrouch;
    const double ratio = elapsed_seconds_ / config_.crouch_duration;
    desired_pose_ = Interpolate(initial_pose_, config_.crouch_pose, SmoothStep(ratio));
    return;
  }

  const double stand_elapsed = elapsed_seconds_ - config_.crouch_duration;
  if (stand_elapsed < config_.stand_duration) {
    state_ = State::kStandingUp;
    const double ratio = stand_elapsed / config_.stand_duration;
    desired_pose_ = Interpolate(config_.crouch_pose, config_.stand_pose, SmoothStep(ratio));
    return;
  }

  desired_pose_ = config_.stand_pose;
  state_ = State::kHolding;
}

const StandController::JointPositions & StandController::GetDesiredPose() const noexcept
{
  return desired_pose_;
}

StandController::State StandController::GetState() const noexcept
{
  return state_;
}

bool StandController::IsHolding() const noexcept
{
  return state_ == State::kHolding;
}

void StandController::ValidateConfig(const Config & config)
{
  if (config.crouch_duration <= 0.0 || config.stand_duration <= 0.0) {
    throw std::invalid_argument("Stand transition durations must be greater than zero");
  }
  if (config.leg_kp < 0.0 || config.leg_kd < 0.0 || config.wheel_kd < 0.0) {
    throw std::invalid_argument("Stand controller gains must not be negative");
  }
}

StandController::JointPositions StandController::Interpolate(
  const JointPositions & from, const JointPositions & to, const double ratio)
{
  JointPositions result{};
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = from[index] + bounded_ratio * (to[index] - from[index]);
  }
  return result;
}

double StandController::SmoothStep(const double ratio) noexcept
{
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  return bounded_ratio * bounded_ratio * (3.0 - 2.0 * bounded_ratio);
}

}  // namespace wheel_dog_mujoco
