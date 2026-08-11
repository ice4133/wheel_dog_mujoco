#pragma once

#include "wheel_dog_mujoco/actuator/actuator_type.h"

namespace wheel_dog_mujoco::actuator
{

struct SafetyTiming
{
  double elapsed_seconds{0.0};
  double command_age{0.0};
  double feedback_age{0.0};
};

struct SafetyOutput
{
  JointCommand command{};
  ActuatorStatus status{};
  bool request_accepted{false};
};

class ActuatorSafety
{
public:
  explicit ActuatorSafety(const ActuatorConfig & config);

  SafetyOutput Update(
    const JointCommand & requested_command, const JointState & state,
    const SafetyTiming & timing);
  void Reset() noexcept;

  JointId GetJointId() const noexcept;
  FaultFlags GetFaults() const noexcept;

private:
  SafetyOutput MakeFallback(
    ControlMode fallback_mode, FaultFlags faults, const JointState & state) noexcept;
  FaultFlags CheckFeedback(const JointState & state, const SafetyTiming & timing) const noexcept;
  static bool IsCommandValid(const JointCommand & command) noexcept;
  static bool IsTimingValid(const SafetyTiming & timing) noexcept;
  static bool IsCriticalFeedbackFault(FaultFlags faults) noexcept;

  ActuatorConfig config_{};
  JointCommand previous_command_{};
  double previous_position_rate_{0.0};
  FaultFlags faults_{ToMask(Fault::kNone)};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::actuator
