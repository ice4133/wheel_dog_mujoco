#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "wheel_dog_mujoco/actuator/actuator_type.h"

namespace wheel_dog_mujoco::skill
{

// Joint-space L4 skill for deterministic stand-up and lie-down. Fixed posture
// motions intentionally bypass the body-space controller and do not use IK.
class StandSkill
{
public:
  static constexpr std::size_t kLegJointCount = 12U;
  using LegJointPositions = std::array<double, kLegJointCount>;

  enum class Phase : std::uint8_t
  {
    kWaitingForState,
    kMovingToCrouch,
    kRising,
    kHolding,
    kLyingDown,
    kLying,
  };

  struct Config
  {
    LegJointPositions crouch_pose{
      0.0, 1.36, -2.65,
      0.0, 1.36, -2.65,
      -0.2, 1.36, -2.65,
      0.2, 1.36, -2.65};
    LegJointPositions stand_pose{
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30};
    double crouch_duration{1.0};
    double rise_duration{1.6};
    double lie_down_duration{1.5};
    double velocity_timeout{0.25};
    double wheel_radius{0.086};
    double track_width{0.284};
    double max_wheel_speed{6.0};
    double wheel_acceleration{12.0};
  };

  explicit StandSkill(const Config & config);

  actuator::JointCommandFrame Update(
    const actuator::JointStateFrame & state,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
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
  bool IsStateValid(const actuator::JointStateFrame & state) const noexcept;
  LegJointPositions ReadLegPositions(
    const actuator::JointStateFrame & state) const noexcept;
  actuator::JointCommandFrame MakeDampingCommand(
    std::chrono::steady_clock::time_point now);
  actuator::JointCommandFrame MakeJointCommand(
    const LegJointPositions & leg_positions,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void StartPhase(
    Phase phase, const LegJointPositions & transition_start) noexcept;
  static LegJointPositions Interpolate(
    const LegJointPositions & from, const LegJointPositions & to,
    double ratio) noexcept;
  static double SmoothStep(double ratio) noexcept;

  Config config_{};
  LegJointPositions transition_start_pose_{};
  LegJointPositions desired_leg_pose_{};
  Phase phase_{Phase::kWaitingForState};
  double desired_linear_velocity_{0.0};
  double desired_yaw_velocity_{0.0};
  double right_wheel_velocity_{0.0};
  double left_wheel_velocity_{0.0};
  double phase_elapsed_seconds_{0.0};
  std::chrono::steady_clock::time_point last_velocity_command_at_{};
  std::uint64_t sequence_{0};
  bool has_velocity_command_{false};
  bool lie_down_requested_{false};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::skill
