#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include "wheel_dog_mujoco/actuator/actuator_type.h"
#include "wheel_dog_mujoco/body/body_controller.h"
#include "wheel_dog_mujoco/body/body_model.h"

namespace wheel_dog_mujoco::body
{

struct CoordinatorOutput
{
  actuator::JointCommandFrame joint_command_frame{};
  std::array<Vector3, kLegCount> wheel_center_targets_body{};
  std::array<double, kLegCount> wheel_velocity_targets{};
  FaultFlags faults{ToMask(Fault::kNone)};
  std::chrono::steady_clock::time_point coordinated_at{};
  std::uint64_t source_sequence{0};
  std::uint64_t sequence{0};
  bool accepted{false};
  bool command_limited{false};
  bool time_step_limited{false};
};

// The wheel-legged specialization of the body layer. It converts the generic
// body controller result into leg IK targets and differential wheel velocities.
class WheelLegCoordinator
{
public:
  explicit WheelLegCoordinator(const BodyModel & model);

  CoordinatorOutput Update(
    const ControllerOutput & controller_output,
    const BodyKinematicState & kinematic_state,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void Reset() noexcept;

  const CoordinatorParameters & GetParameters() const noexcept;

private:
  CoordinatorOutput MakeDampingOutput(
    FaultFlags faults, std::chrono::steady_clock::time_point now,
    std::uint64_t source_sequence, bool accepted);

  const BodyModel & model_;
  BodyConfig config_{};
  std::array<double, kLegCount> wheel_center_z_targets_{};
  std::array<double, kLegCount> wheel_center_z_rates_{};
  std::array<double, kLegCount> wheel_velocity_targets_{};
  std::uint64_t sequence_{0};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::body
