#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include "wheel_dog_mujoco/body/body_trajectory.h"
#include "wheel_dog_mujoco/body/body_type.h"

namespace wheel_dog_mujoco::body
{

struct LegPostureAdjustment
{
  // Applied by WheelLegCoordinator on top of the geometric target derived from
  // stabilized_pose_world. Positive moves the wheel center farther down.
  double extension_offset{0.0};
  bool limited{false};
};

struct ControllerOutput
{
  std::array<LegPostureAdjustment, kLegCount> legs{};
  Pose stabilized_pose_world{};
  Twist body_twist_command{};
  SpatialAcceleration acceleration_feedforward{};
  ReferenceFrame twist_frame{ReferenceFrame::kBody};
  ControlMode active_mode{ControlMode::kDisabled};
  FaultFlags faults{ToMask(Fault::kNone)};
  std::chrono::steady_clock::time_point evaluated_at{};
  std::uint64_t source_sequence{0};
  std::uint64_t sequence{0};
  double filtered_lateral_acceleration{0.0};
  double turning_roll_compensation{0.0};
  bool accepted{false};
  bool command_limited{false};
  bool time_step_limited{false};
};

// Closes the body pose/velocity feedback loop. It calculates virtual body
// motion and per-leg extension corrections, but never creates joint or motor commands.
class BodyController
{
public:
  explicit BodyController(const BodyConfig & config);

  ControllerOutput Update(
    const TrajectoryOutput & reference,
    const BodyState & state,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void Reset() noexcept;

  ControlMode GetActiveMode() const noexcept;
  const ControllerParameters & GetParameters() const noexcept;

private:
  ControllerOutput MakeRejectedOutput(
    FaultFlags faults, std::chrono::steady_clock::time_point now,
    std::uint64_t source_sequence) const noexcept;

  BodyConfig config_{};
  std::array<double, 3> linear_integral_{};
  std::array<double, 3> angular_integral_{};
  ControlMode active_mode_{ControlMode::kDisabled};
  ReferenceFrame active_twist_frame_{ReferenceFrame::kBody};
  double filtered_lateral_acceleration_{0.0};
  std::uint64_t sequence_{0};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::body
