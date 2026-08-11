#pragma once

#include "wheel_dog_mujoco/actuator/actuator_type.h"

namespace wheel_dog_mujoco::actuator
{

struct ControllerOutput
{
  JointCommand command{};
  ControlGains gains{};
  ActuatorStatus status{};
  bool accepted{false};
};

class ActuatorController
{
public:
  explicit ActuatorController(const ActuatorConfig & config);

  ControllerOutput Update(
    const JointCommand & requested_command, const JointState & state, double elapsed_seconds);
  void Reset() noexcept;

  JointId GetJointId() const noexcept;
  ControlMode GetActiveMode() const noexcept;
  double GetIntegralError() const noexcept;

private:
  const ControlGains & SelectGains(GainProfile profile) const;
  void StartTransition(const JointCommand & command, const JointState & state) noexcept;
  ControllerOutput MakeRejectedOutput(Fault fault) noexcept;
  static bool IsCommandValid(const JointCommand & command) noexcept;
  static bool IsStateValid(const JointState & state) noexcept;
  static double SmoothStep(double ratio) noexcept;
  static double Interpolate(double from, double to, double ratio) noexcept;

  ActuatorConfig config_{};
  ControlMode active_mode_{ControlMode::kDisabled};
  GainProfile active_gain_profile_{GainProfile::kNormal};
  double integral_error_{0.0};
  double transition_elapsed_{0.0};
  double transition_start_position_{0.0};
  double transition_start_velocity_{0.0};
  bool initialized_{false};
};

}  // namespace wheel_dog_mujoco::actuator
