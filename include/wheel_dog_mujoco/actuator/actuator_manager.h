#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "wheel_dog_mujoco/actuator/actuator_controller.h"
#include "wheel_dog_mujoco/actuator/actuator_model.h"
#include "wheel_dog_mujoco/actuator/actuator_safety.h"

namespace wheel_dog_mujoco::actuator
{

struct ManagerOutput
{
  driver::RobotCommand robot_command{};
  std::array<ActuatorStatus, kActuatorCount> status{};
  FaultFlags combined_faults{ToMask(Fault::kNone)};
  bool all_requests_accepted{false};
  bool any_command_limited{false};
};

class ActuatorManager
{
public:
  explicit ActuatorManager(const std::string & config_path);

  JointStateFrame DecodeFeedback(const driver::RobotFeedback & feedback) const;
  ManagerOutput Update(
    const JointCommandFrame & command_frame, const JointStateFrame & state_frame,
    std::chrono::steady_clock::time_point now, double elapsed_seconds);
  void Reset() noexcept;

  const ActuatorModel & GetModel() const noexcept;

private:
  ActuatorModel model_;
  std::vector<ActuatorController> controllers_;
  std::vector<ActuatorSafety> safety_modules_;
};

}  // namespace wheel_dog_mujoco::actuator
