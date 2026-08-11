#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "wheel_dog_mujoco/actuator/actuator_type.h"
#include "wheel_dog_mujoco/body/body_controller.h"
#include "wheel_dog_mujoco/body/body_model.h"
#include "wheel_dog_mujoco/body/body_state_estimator.h"
#include "wheel_dog_mujoco/body/body_trajectory.h"
#include "wheel_dog_mujoco/body/body_type.h"
#include "wheel_dog_mujoco/body/wheel_leg_coordinator.h"

namespace wheel_dog_mujoco::body
{

struct ManagerOutput
{
  actuator::JointCommandFrame joint_command_frame{};
  BodyStateFrame body_state_frame{};
  BodyKinematicState kinematic_state{};
  EstimatorOutput estimator{};
  TrajectoryOutput trajectory{};
  ControllerOutput controller{};
  CoordinatorOutput coordinator{};
  FaultFlags combined_faults{ToMask(Fault::kNone)};
  std::chrono::steady_clock::time_point updated_at{};
  std::uint64_t sequence{0};
  bool accepted{false};
  bool command_limited{false};
};

// The only public entry point of the body-motion layer. All transport and
// actuator details remain outside; callers provide normalized state and receive
// a semantic JointCommandFrame for ActuatorManager.
class BodyManager
{
public:
  explicit BodyManager(const std::string & config_path);

  ManagerOutput Update(
    const BodyCommandFrame & command,
    const actuator::JointStateFrame & actuator_state,
    const BodySensorFrame & sensor_state,
    std::chrono::steady_clock::time_point now,
    double elapsed_seconds);
  void Reset() noexcept;

  const BodyModel & GetModel() const noexcept;
  const BodyConfig & GetConfig() const noexcept;

private:
  BodyModel model_;
  BodyStateEstimator estimator_;
  BodyTrajectory trajectory_;
  BodyController controller_;
  WheelLegCoordinator coordinator_;
  std::uint64_t sequence_{0};
};

}  // namespace wheel_dog_mujoco::body
