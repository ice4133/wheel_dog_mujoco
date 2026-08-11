#pragma once

#include <chrono>
#include <cstdint>

#include "wheel_dog_mujoco/body/body_type.h"

namespace wheel_dog_mujoco::body
{

struct TrajectoryOutput
{
  BodyCommandFrame command_frame{};

  // Pose-trajectory derivatives are kept separate from desired_twist because
  // hybrid control can use a world-frame pose and a body-frame velocity target.
  Vector3 pose_position_rate_world{};
  Vector3 pose_position_acceleration_world{};
  Vector3 pose_rpy_rate{};
  Vector3 pose_rpy_acceleration{};

  FaultFlags faults{ToMask(Fault::kNone)};
  std::uint64_t source_sequence{0};
  bool accepted{false};
  bool target_reached{false};
  bool command_limited{false};
  bool time_step_limited{false};
};

// Converts discontinuous body targets into velocity- and acceleration-limited
// references. The output acceleration_feedforward is regenerated from the
// velocity trajectory; pose-path derivatives are reported separately above.
// It performs no state feedback control and emits no actuator command.
class BodyTrajectory
{
public:
  explicit BodyTrajectory(const BodyConfig & config);

  TrajectoryOutput Update(
    const BodyCommandFrame & target,
    const BodyState & current_state,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void Reset() noexcept;

  const BodyCommandFrame & GetCommandFrame() const noexcept;
  const TrajectoryParameters & GetParameters() const noexcept;

private:
  void InitializeFromState(
    const BodyCommandFrame & target, const BodyState & current_state);
  TrajectoryOutput MakeRejectedOutput(
    FaultFlags faults, std::uint64_t source_sequence) const noexcept;

  BodyConfig config_{};
  BodyCommandFrame command_frame_{};
  Vector3 pose_position_rate_world_{};
  Vector3 pose_rpy_{};
  Vector3 pose_rpy_rate_{};
  std::uint64_t output_sequence_{0};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::body
