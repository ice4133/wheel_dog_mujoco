#include <array>
#include <chrono>
#include <cstddef>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_manager.h"

namespace wheel_dog_mujoco::body
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::array<std::array<actuator::JointId, 3>, kLegCount> kLegJointIds{{
  {actuator::JointId::kFrontRightHip, actuator::JointId::kFrontRightThigh,
    actuator::JointId::kFrontRightCalf},
  {actuator::JointId::kFrontLeftHip, actuator::JointId::kFrontLeftThigh,
    actuator::JointId::kFrontLeftCalf},
  {actuator::JointId::kRearRightHip, actuator::JointId::kRearRightThigh,
    actuator::JointId::kRearRightCalf},
  {actuator::JointId::kRearLeftHip, actuator::JointId::kRearLeftThigh,
    actuator::JointId::kRearLeftCalf},
}};

actuator::JointStateFrame MakeJointState(const Clock::time_point now)
{
  actuator::JointStateFrame frame;
  frame.received_at = now;
  frame.sequence = 5U;
  for (auto & joint : frame.joints) {
    joint.online = true;
    joint.temperature = 25.0;
  }
  for (const auto & ids : kLegJointIds) {
    frame.joints[actuator::ToIndex(ids[0])].position = 0.0;
    frame.joints[actuator::ToIndex(ids[1])].position = 0.67;
    frame.joints[actuator::ToIndex(ids[2])].position = -1.30;
  }
  return frame;
}

BodySensorFrame MakeSensorState(const Clock::time_point now)
{
  BodySensorFrame frame;
  frame.received_at = now;
  frame.sequence = 6U;
  frame.imu.orientation_world_from_body = Quaternion{};
  frame.imu.linear_acceleration_body.z = 9.81;
  frame.imu.temperature = 25.0;
  frame.imu.orientation_valid = true;
  frame.imu.angular_velocity_valid = true;
  frame.imu.linear_acceleration_valid = true;
  for (auto & contact : frame.contacts) {
    contact.normal_force = 20.0;
    contact.in_contact = true;
    contact.valid = true;
  }
  return frame;
}

BodyCommandFrame MakeCommand(
  const BodyManager & manager, const actuator::JointStateFrame & state,
  const Clock::time_point now)
{
  const auto kinematics = manager.GetModel().ComputeKinematicState(state);
  double height = 0.0;
  for (const auto & leg : kinematics.legs) {
    height += manager.GetConfig().geometry.wheel_radius -
      leg.wheel_center_position_body.z;
  }
  height /= static_cast<double>(kLegCount);

  BodyCommandFrame command;
  command.command.control_mode = ControlMode::kHybrid;
  command.command.pose_frame = ReferenceFrame::kWorld;
  command.command.twist_frame = ReferenceFrame::kBody;
  command.command.desired_pose.position.z = height;
  command.command.desired_pose.orientation = Quaternion{};
  command.created_at = now;
  command.sequence = 7U;
  return command;
}

TEST(BodyManagerTest, RunsCompleteBodyPipelineThroughSingleEntryPoint)
{
  BodyManager manager(BODY_CONFIG_PATH);
  const auto now = Clock::now();
  const auto actuator_state = MakeJointState(now);
  const auto sensor_state = MakeSensorState(now);
  const auto command = MakeCommand(manager, actuator_state, now);

  const auto output = manager.Update(
    command, actuator_state, sensor_state, now, 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_TRUE(output.estimator.state.valid);
  EXPECT_TRUE(output.trajectory.accepted);
  EXPECT_TRUE(output.controller.accepted);
  EXPECT_TRUE(output.coordinator.accepted);
  EXPECT_EQ(output.sequence, 1U);
  EXPECT_EQ(output.body_state_frame.status.active_mode, ControlMode::kHybrid);
  for (std::size_t index = 0U; index < actuator::kActuatorCount; ++index) {
    const auto expected_mode = index < 12U ?
      actuator::ControlMode::kHybrid : actuator::ControlMode::kVelocity;
    EXPECT_EQ(output.joint_command_frame.joints[index].control_mode, expected_mode);
  }
}

TEST(BodyManagerTest, SmoothlyPropagatesVelocityCommandToDifferentialWheels)
{
  BodyManager manager(BODY_CONFIG_PATH);
  const auto start = Clock::now();
  auto actuator_state = MakeJointState(start);
  auto sensor_state = MakeSensorState(start);
  auto command = MakeCommand(manager, actuator_state, start);
  command.command.desired_twist.linear.x = 0.30;
  command.command.desired_twist.angular.z = 0.80;

  ManagerOutput output;
  for (std::size_t step = 0U; step < 40U; ++step) {
    const auto now = start + std::chrono::milliseconds(10 * step);
    actuator_state.received_at = now;
    sensor_state.received_at = now;
    command.created_at = now;
    output = manager.Update(command, actuator_state, sensor_state, now, 0.01);
  }

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(
    output.coordinator.wheel_velocity_targets[ToIndex(LegId::kFrontRight)],
    output.coordinator.wheel_velocity_targets[ToIndex(LegId::kFrontLeft)]);
  EXPECT_GT(
    output.coordinator.wheel_velocity_targets[ToIndex(LegId::kFrontLeft)], 0.0);
}

TEST(BodyManagerTest, StaleCommandProducesDampingFallback)
{
  BodyManager manager(BODY_CONFIG_PATH);
  const auto now = Clock::now();
  const auto actuator_state = MakeJointState(now);
  const auto sensor_state = MakeSensorState(now);
  auto command = MakeCommand(manager, actuator_state, now);
  command.created_at = now - std::chrono::seconds(1);

  const auto output = manager.Update(
    command, actuator_state, sensor_state, now, 0.01);

  EXPECT_FALSE(output.accepted);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kCommandTimeout));
  for (const auto & joint : output.joint_command_frame.joints) {
    EXPECT_EQ(joint.control_mode, actuator::ControlMode::kDamping);
  }
}

TEST(BodyManagerTest, InvalidSensorStateCannotReachCoordinatorAsActiveCommand)
{
  BodyManager manager(BODY_CONFIG_PATH);
  const auto now = Clock::now();
  const auto actuator_state = MakeJointState(now);
  auto sensor_state = MakeSensorState(now);
  sensor_state.imu.orientation_valid = false;
  const auto command = MakeCommand(manager, actuator_state, now);

  const auto output = manager.Update(
    command, actuator_state, sensor_state, now, 0.01);

  EXPECT_FALSE(output.accepted);
  EXPECT_FALSE(output.estimator.state.valid);
  EXPECT_TRUE(HasFault(output.combined_faults, Fault::kSensorStateInvalid));
  for (const auto & joint : output.joint_command_frame.joints) {
    EXPECT_EQ(joint.control_mode, actuator::ControlMode::kDamping);
  }
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
