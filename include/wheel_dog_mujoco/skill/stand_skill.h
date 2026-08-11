#pragma once

#include <chrono>
#include <cstdint>

#include "wheel_dog_mujoco/body/body_type.h"

namespace wheel_dog_mujoco::skill
{

// StandSkill is an L4 finite-state motion skill. It never talks to DDS or
// actuators; its only output is the L3 BodyCommandFrame boundary.
class StandSkill
{
public:
  enum class Phase : std::uint8_t
  {
    kWaitingForState,
    kRecovering,
    kRising,
    kHolding,
    kLyingDown,
    kLying,
  };

  struct Config
  {
    double recovery_ground_clearance{0.20};
    double stand_ground_clearance{0.42};
    double lie_down_ground_clearance{0.20};
    double height_tolerance{0.03};
    double attitude_tolerance{0.15};
    double angular_velocity_tolerance{0.40};
    double settle_duration{0.25};
    double velocity_timeout{0.25};
  };

  explicit StandSkill(const Config & config);

  body::BodyCommandFrame Update(
    const body::BodyStateFrame & body_state,
    std::chrono::steady_clock::time_point now);
  void SetVelocityTarget(
    double linear_velocity, double yaw_velocity,
    std::chrono::steady_clock::time_point received_at) noexcept;
  void RequestLieDown() noexcept;
  void Reset() noexcept;

  Phase GetPhase() const noexcept;
  bool IsStanding() const noexcept;
  bool IsLying() const noexcept;
  static const char * PhaseName(Phase phase) noexcept;

private:
  body::BodyCommandFrame MakeDisabledCommand(
    std::chrono::steady_clock::time_point now);
  body::BodyCommandFrame MakeHeightCommand(
    double ground_clearance, bool allow_driving,
    std::chrono::steady_clock::time_point now);
  bool IsPostureSettled(
    const body::BodyState & state, double target_ground_clearance,
    std::chrono::steady_clock::time_point now) noexcept;
  void SetPhase(Phase phase) noexcept;

  Config config_{};
  Phase phase_{Phase::kWaitingForState};
  double desired_linear_velocity_{0.0};
  double desired_yaw_velocity_{0.0};
  std::chrono::steady_clock::time_point last_velocity_command_at_{};
  std::chrono::steady_clock::time_point settled_since_{};
  std::uint64_t sequence_{0};
  bool has_velocity_command_{false};
  bool lie_down_requested_{false};
};

}  // namespace wheel_dog_mujoco::skill
