#include <chrono>
#include <cstddef>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/actuator/actuator_manager.h"

namespace wheel_dog_mujoco::actuator
{
namespace
{

using Clock = std::chrono::steady_clock;

double SafeInitialPosition(const JointId joint_id)
{
  switch (joint_id) {
    case JointId::kFrontRightCalf:
    case JointId::kFrontLeftCalf:
    case JointId::kRearRightCalf:
    case JointId::kRearLeftCalf:
      return -2.0;
    case JointId::kFrontRightThigh:
    case JointId::kFrontLeftThigh:
    case JointId::kRearRightThigh:
    case JointId::kRearLeftThigh:
      return 1.0;
    default:
      return 0.0;
  }
}

driver::RobotFeedback MakeFeedback(
  const ActuatorManager & manager, const Clock::time_point received_at)
{
  driver::RobotFeedback feedback;
  feedback.received_at = received_at;
  feedback.tick = 42U;
  for (const auto & config : manager.GetModel().GetConfigurations()) {
    auto & motor = feedback.motors[config.motor_index];
    motor.position = static_cast<float>(manager.GetModel().JointToMotorPosition(
        config.joint_id, SafeInitialPosition(config.joint_id)));
    motor.velocity = 0.0F;
    motor.acceleration = 0.0F;
    motor.estimated_torque = 0.0F;
    motor.temperature = 25;
    motor.lost = 0U;
  }
  return feedback;
}

JointCommandFrame MakeStandCommand(const Clock::time_point created_at)
{
  JointCommandFrame command;
  command.created_at = created_at;
  command.sequence = 7U;
  for (std::size_t index = 0; index < kActuatorCount; ++index) {
    auto & joint = command.joints[index];
    joint.gain_profile = GainProfile::kNormal;
    if (index < ToIndex(JointId::kFrontRightWheel)) {
      joint.control_mode = ControlMode::kPosition;
      joint.position = SafeInitialPosition(static_cast<JointId>(index));
    } else {
      joint.control_mode = ControlMode::kVelocity;
      joint.velocity = 0.0;
    }
  }
  return command;
}

TEST(ActuatorManagerTest, BuildsCompleteStandCommand)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto feedback_time = Clock::now();
  const auto state = manager.DecodeFeedback(MakeFeedback(manager, feedback_time));
  const auto command = MakeStandCommand(feedback_time);

  const auto output = manager.Update(
    command, state, feedback_time + std::chrono::milliseconds(1), 0.002);

  EXPECT_TRUE(output.all_requests_accepted);
  EXPECT_EQ(output.combined_faults, ToMask(Fault::kNone));
  for (const auto & config : manager.GetModel().GetConfigurations()) {
    const auto expected_mode = config.actuator_kind == ActuatorKind::kWheel ?
      driver::MotorControlMode::kVelocity : driver::MotorControlMode::kPosition;
    EXPECT_EQ(output.robot_command.motors[config.motor_index].control_mode, expected_mode);
  }
}

TEST(ActuatorManagerTest, DisablesMotorsWhenFeedbackIsStale)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto now = Clock::now();
  const auto state = manager.DecodeFeedback(MakeFeedback(manager, now - std::chrono::seconds(1)));
  const auto output = manager.Update(MakeStandCommand(now), state, now, 0.002);

  EXPECT_FALSE(output.all_requests_accepted);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kFeedbackTimeout));
  for (const auto & config : manager.GetModel().GetConfigurations()) {
    EXPECT_EQ(
      output.robot_command.motors[config.motor_index].control_mode,
      driver::MotorControlMode::kDisabled);
  }
}

TEST(ActuatorManagerTest, UsesDampingWhenCommandTimesOut)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto now = Clock::now();
  const auto state = manager.DecodeFeedback(MakeFeedback(manager, now));
  const auto output = manager.Update(
    MakeStandCommand(now - std::chrono::seconds(1)), state, now, 0.002);

  EXPECT_FALSE(output.all_requests_accepted);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kCommandTimeout));
  for (const auto & config : manager.GetModel().GetConfigurations()) {
    EXPECT_EQ(
      output.robot_command.motors[config.motor_index].control_mode,
      driver::MotorControlMode::kVelocity);
    EXPECT_EQ(output.robot_command.motors[config.motor_index].velocity, 0.0F);
  }
}

TEST(ActuatorManagerTest, IgnoresSmallFeedbackPositionOvershoot)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto now = Clock::now();
  auto feedback = MakeFeedback(manager, now);
  const auto & config = manager.GetModel().GetConfiguration(JointId::kFrontRightHip);
  feedback.motors[config.motor_index].position = static_cast<float>(
    manager.GetModel().JointToMotorPosition(
      config.joint_id, config.limits.max_position + 0.5 * config.safety.position_limit_tolerance));
  const auto state = manager.DecodeFeedback(feedback);

  const auto output = manager.Update(MakeStandCommand(now), state, now, 0.002);

  EXPECT_TRUE(output.all_requests_accepted);
  EXPECT_FALSE(HasFault(output.combined_faults, Fault::kPositionLimit));
}

TEST(ActuatorManagerTest, RecoversFeedbackPositionBeyondTolerance)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto now = Clock::now();
  auto feedback = MakeFeedback(manager, now);
  const auto & config = manager.GetModel().GetConfiguration(JointId::kFrontRightHip);
  feedback.motors[config.motor_index].position = static_cast<float>(
    manager.GetModel().JointToMotorPosition(
      config.joint_id, config.limits.max_position +
      2.0 * config.safety.position_limit_tolerance));
  const auto state = manager.DecodeFeedback(feedback);

  const auto output = manager.Update(MakeStandCommand(now), state, now, 0.002);

  EXPECT_TRUE(output.all_requests_accepted);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kPositionLimit));
  EXPECT_EQ(
    output.robot_command.motors[config.motor_index].control_mode,
    driver::MotorControlMode::kPosition);
  EXPECT_LE(
    manager.GetModel().MotorToJointPosition(
      config.joint_id, output.robot_command.motors[config.motor_index].position),
    state.joints[ToIndex(config.joint_id)].position);
}

TEST(ActuatorManagerTest, UsesDampingForFeedbackVelocityViolation)
{
  ActuatorManager manager(ACTUATOR_CONFIG_PATH);
  const auto now = Clock::now();
  auto feedback = MakeFeedback(manager, now);
  const auto & config = manager.GetModel().GetConfiguration(JointId::kFrontRightHip);
  feedback.motors[config.motor_index].velocity = static_cast<float>(
    manager.GetModel().JointToMotorVelocity(
      config.joint_id, config.limits.max_velocity +
      2.0 * config.safety.velocity_limit_tolerance));
  const auto state = manager.DecodeFeedback(feedback);

  const auto output = manager.Update(MakeStandCommand(now), state, now, 0.002);

  EXPECT_FALSE(output.all_requests_accepted);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kVelocityLimit));
  EXPECT_EQ(
    output.robot_command.motors[config.motor_index].control_mode,
    driver::MotorControlMode::kVelocity);
  EXPECT_EQ(output.robot_command.motors[config.motor_index].velocity, 0.0F);
}

}  // namespace
}  // namespace wheel_dog_mujoco::actuator
