#include "wheel_dog_mujoco/body/body_manager.h"

#include <cmath>

namespace wheel_dog_mujoco::body
{

BodyManager::BodyManager(const std::string & config_path)
: model_(config_path),
  estimator_(model_.GetConfig()),
  trajectory_(model_.GetConfig()),
  controller_(model_.GetConfig()),
  coordinator_(model_)
{
}

ManagerOutput BodyManager::Update(
  const BodyCommandFrame & command,
  const actuator::JointStateFrame & actuator_state,
  const BodySensorFrame & sensor_state,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  ManagerOutput output;
  output.kinematic_state = model_.ComputeKinematicState(actuator_state);
  output.estimator = estimator_.Update(
    output.kinematic_state, sensor_state, now, elapsed_seconds);

  FaultFlags command_faults = ToMask(Fault::kNone);
  if (command.command.control_mode != ControlMode::kDisabled) {
    const double command_age =
      std::chrono::duration<double>(now - command.created_at).count();
    if (!std::isfinite(command_age) || command_age < 0.0) {
      command_faults |= ToMask(Fault::kInvalidCommand);
    } else if (command_age > model_.GetConfig().safety.command_timeout) {
      command_faults |= ToMask(Fault::kCommandTimeout);
    }
  }

  if (command_faults != ToMask(Fault::kNone)) {
    trajectory_.Reset();
    controller_.Reset();
    coordinator_.Reset();
    output.trajectory.command_frame = command;
    output.trajectory.faults = command_faults;
    output.trajectory.source_sequence = command.sequence;
    output.controller.active_mode = ControlMode::kDisabled;
    output.controller.faults = command_faults;
    output.controller.evaluated_at = now;
    output.controller.source_sequence = command.sequence;
    output.controller.accepted = true;
    output.coordinator = coordinator_.Update(
      output.controller, output.kinematic_state, now, elapsed_seconds);
    output.joint_command_frame = output.coordinator.joint_command_frame;
    output.combined_faults = output.estimator.faults | command_faults |
      output.coordinator.faults;
    output.command_limited = true;
  } else {
    output.trajectory = trajectory_.Update(
      command, output.estimator.state, now, elapsed_seconds);
    output.controller = controller_.Update(
      output.trajectory, output.estimator.state, now, elapsed_seconds);
    output.coordinator = coordinator_.Update(
      output.controller, output.kinematic_state, now, elapsed_seconds);
    output.joint_command_frame = output.coordinator.joint_command_frame;
    output.combined_faults = output.estimator.faults | output.trajectory.faults |
      output.controller.faults | output.coordinator.faults;
    output.command_limited = output.trajectory.command_limited ||
      output.controller.command_limited || output.coordinator.command_limited;
    output.accepted = output.estimator.state.valid && output.trajectory.accepted &&
      output.controller.accepted && output.coordinator.accepted;
  }

  output.updated_at = now;
  output.sequence = ++sequence_;
  output.body_state_frame.state = output.estimator.state;
  output.body_state_frame.status.active_mode = output.controller.active_mode;
  output.body_state_frame.status.faults = output.combined_faults;
  output.body_state_frame.status.command_limited = output.command_limited;
  output.body_state_frame.status.state_estimate_valid = output.estimator.state.valid;
  output.body_state_frame.received_at = now;
  output.body_state_frame.sequence = output.sequence;
  return output;
}

void BodyManager::Reset() noexcept
{
  estimator_.Reset();
  trajectory_.Reset();
  controller_.Reset();
  coordinator_.Reset();
  sequence_ = 0U;
}

const BodyModel & BodyManager::GetModel() const noexcept
{
  return model_;
}

const BodyConfig & BodyManager::GetConfig() const noexcept
{
  return model_.GetConfig();
}

}  // namespace wheel_dog_mujoco::body
