#include "wheel_dog_mujoco/actuator/actuator_manager.h"

#include <stdexcept>

namespace wheel_dog_mujoco::actuator
{

ActuatorManager::ActuatorManager(const std::string & config_path)
: model_(config_path)
{
  controllers_.reserve(kActuatorCount);
  safety_modules_.reserve(kActuatorCount);
  for (const auto & config : model_.GetConfigurations()) {
    controllers_.emplace_back(config);
    safety_modules_.emplace_back(config);
  }
  if (controllers_.size() != kActuatorCount || safety_modules_.size() != kActuatorCount) {
    throw std::runtime_error("ActuatorManager did not create exactly 16 actuator channels");
  }
}

JointStateFrame ActuatorManager::DecodeFeedback(const driver::RobotFeedback & feedback) const
{
  return model_.DecodeFeedback(feedback);
}

ManagerOutput ActuatorManager::Update(
  const JointCommandFrame & command_frame, const JointStateFrame & state_frame,
  const std::chrono::steady_clock::time_point now, const double elapsed_seconds)
{
  ManagerOutput output;
  output.all_requests_accepted = true;

  const double command_age =
    std::chrono::duration<double>(now - command_frame.created_at).count();
  const double feedback_age =
    std::chrono::duration<double>(now - state_frame.received_at).count();
  const SafetyTiming timing{elapsed_seconds, command_age, feedback_age};

  for (std::size_t index = 0; index < kActuatorCount; ++index) {
    const JointId joint_id = static_cast<JointId>(index);
    const auto safety_output = safety_modules_[index].Update(
      command_frame.joints[index], state_frame.joints[index], timing);
    const auto controller_output = controllers_[index].Update(
      safety_output.command, state_frame.joints[index], elapsed_seconds);

    auto & status = output.status[index];
    status.active_mode = controller_output.status.active_mode;
    status.faults = safety_output.status.faults | controller_output.status.faults;
    status.command_limited =
      safety_output.status.command_limited || controller_output.status.command_limited;

    const auto & config = model_.GetConfiguration(joint_id);
    output.robot_command.motors[config.motor_index] = model_.EncodeCommand(
      joint_id, controller_output.command, controller_output.gains);
    output.combined_faults |= status.faults;
    output.any_command_limited = output.any_command_limited || status.command_limited;
    output.all_requests_accepted = output.all_requests_accepted &&
      safety_output.request_accepted && controller_output.accepted;
  }
  return output;
}

void ActuatorManager::Reset() noexcept
{
  for (auto & controller : controllers_) {
    controller.Reset();
  }
  for (auto & safety : safety_modules_) {
    safety.Reset();
  }
}

const ActuatorModel & ActuatorManager::GetModel() const noexcept
{
  return model_;
}

}  // namespace wheel_dog_mujoco::actuator
