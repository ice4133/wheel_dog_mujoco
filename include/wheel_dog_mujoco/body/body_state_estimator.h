#pragma once

#include <chrono>
#include <cstdint>

#include "wheel_dog_mujoco/body/body_model.h"
#include "wheel_dog_mujoco/body/body_type.h"

namespace wheel_dog_mujoco::body
{

struct EstimatorOutput
{
  BodyState state{};
  FaultFlags faults{ToMask(Fault::kNone)};
  std::chrono::steady_clock::time_point estimated_at{};
  std::uint64_t sequence{0};
  bool time_step_limited{false};
};

// Fuses transport-independent IMU/contact data with BodyModel kinematics.
// It owns estimation state only: it performs no I/O and emits no actuator command.
class BodyStateEstimator
{
public:
  explicit BodyStateEstimator(const BodyConfig & config);

  EstimatorOutput Update(
    const BodyKinematicState & kinematic_state,
    const BodySensorFrame & sensor_frame,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void Reset() noexcept;

  const BodyState & GetState() const noexcept;
  const EstimatorParameters & GetParameters() const noexcept;

private:
  EstimatorOutput MakeInvalidOutput(
    FaultFlags faults, std::chrono::steady_clock::time_point now) noexcept;

  BodyConfig config_{};
  BodyState state_{};
  std::uint64_t sequence_{0};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::body
