#pragma once

#include <array>
#include <string>

#include "wheel_dog_mujoco/actuator/actuator_type.h"
#include "wheel_dog_mujoco/driver/drv_dds.h"

namespace wheel_dog_mujoco::actuator
{

class ActuatorModel
{
public:
  using Configurations = std::array<ActuatorConfig, kActuatorCount>;

  explicit ActuatorModel(const std::string & config_path);

  const Configurations & GetConfigurations() const noexcept;
  const ActuatorConfig & GetConfiguration(JointId joint_id) const;
  const ControlGains & GetGains(JointId joint_id, GainProfile profile) const;

  JointStateFrame DecodeFeedback(const driver::RobotFeedback & feedback) const;
  driver::MotorCommand EncodeCommand(
    JointId joint_id, const JointCommand & command, const ControlGains & gains) const;

  double MotorToJointPosition(JointId joint_id, double motor_position) const;
  double JointToMotorPosition(JointId joint_id, double joint_position) const;
  double MotorToJointVelocity(JointId joint_id, double motor_velocity) const;
  double JointToMotorVelocity(JointId joint_id, double joint_velocity) const;
  double MotorToJointTorque(JointId joint_id, double motor_torque) const;
  double JointToMotorTorque(JointId joint_id, double joint_torque) const;

private:
  static Configurations LoadConfigurations(const std::string & config_path);

  Configurations configurations_{};
};

}  // namespace wheel_dog_mujoco::actuator
