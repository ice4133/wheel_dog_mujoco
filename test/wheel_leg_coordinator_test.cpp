#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_model.h"
#include "wheel_dog_mujoco/body/wheel_leg_coordinator.h"

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

constexpr std::array<actuator::JointId, kLegCount> kWheelJointIds{{
  actuator::JointId::kFrontRightWheel,
  actuator::JointId::kFrontLeftWheel,
  actuator::JointId::kRearRightWheel,
  actuator::JointId::kRearLeftWheel,
}};

Quaternion QuaternionFromRpy(const double roll, const double pitch, const double yaw)
{
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  return Quaternion{
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy};
}

actuator::JointStateFrame MakeJointState(const Clock::time_point now)
{
  actuator::JointStateFrame frame;
  frame.received_at = now;
  for (auto & joint : frame.joints) {
    joint.online = true;
  }
  for (const auto & ids : kLegJointIds) {
    frame.joints[actuator::ToIndex(ids[0])].position = 0.0;
    frame.joints[actuator::ToIndex(ids[1])].position = 0.67;
    frame.joints[actuator::ToIndex(ids[2])].position = -1.30;
  }
  return frame;
}

ControllerOutput MakeControllerOutput(
  const BodyModel & model, const BodyKinematicState & kinematics)
{
  ControllerOutput output;
  output.stabilized_pose_world.position.z =
    model.GetGeometry().wheel_radius - kinematics.legs[0].wheel_center_position_body.z;
  output.stabilized_pose_world.orientation = Quaternion{};
  output.twist_frame = ReferenceFrame::kBody;
  output.active_mode = ControlMode::kHybrid;
  output.sequence = 13U;
  output.accepted = true;
  return output;
}

TEST(WheelLegCoordinatorTest, ConvertsHeightTargetIntoSmoothLegJointCommands)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  const auto now = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  auto controller = MakeControllerOutput(model, kinematics);
  controller.stabilized_pose_world.position.z += 0.05;

  const auto output = coordinator.Update(controller, kinematics, now, 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_TRUE(output.command_limited);
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    EXPECT_LT(
      output.wheel_center_targets_body[index].z,
      kinematics.legs[index].wheel_center_position_body.z);
    for (const auto joint_id : kLegJointIds[index]) {
      EXPECT_EQ(
        output.joint_command_frame.joints[actuator::ToIndex(joint_id)].control_mode,
        actuator::ControlMode::kHybrid);
    }
  }
}

TEST(WheelLegCoordinatorTest, RollTargetExtendsLeftAndRetractsRightSide)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  const auto now = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  auto controller = MakeControllerOutput(model, kinematics);
  controller.stabilized_pose_world.orientation = QuaternionFromRpy(0.10, 0.0, 0.0);

  const auto output = coordinator.Update(controller, kinematics, now, 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_LT(
    output.wheel_center_targets_body[ToIndex(LegId::kFrontLeft)].z,
    kinematics.legs[ToIndex(LegId::kFrontLeft)].wheel_center_position_body.z);
  EXPECT_GT(
    output.wheel_center_targets_body[ToIndex(LegId::kFrontRight)].z,
    kinematics.legs[ToIndex(LegId::kFrontRight)].wheel_center_position_body.z);
}

TEST(WheelLegCoordinatorTest, SplitsForwardAndYawVelocityAcrossWheels)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  const auto start = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(start));
  auto controller = MakeControllerOutput(model, kinematics);
  controller.body_twist_command.linear.x = 0.30;
  controller.body_twist_command.angular.z = 1.0;

  CoordinatorOutput output;
  for (std::size_t step = 0U; step < 30U; ++step) {
    output = coordinator.Update(
      controller, kinematics, start + std::chrono::milliseconds(20 * step), 0.02);
  }

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(
    output.wheel_velocity_targets[ToIndex(LegId::kFrontRight)],
    output.wheel_velocity_targets[ToIndex(LegId::kFrontLeft)]);
  EXPECT_NEAR(
    output.wheel_velocity_targets[ToIndex(LegId::kFrontRight)],
    output.wheel_velocity_targets[ToIndex(LegId::kRearRight)], 1.0E-12);
  EXPECT_NEAR(
    output.wheel_velocity_targets[ToIndex(LegId::kFrontLeft)],
    output.wheel_velocity_targets[ToIndex(LegId::kRearLeft)], 1.0E-12);
}

TEST(WheelLegCoordinatorTest, PreservesCurvatureWhenWheelSpeedIsLimited)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  const auto start = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(start));
  auto controller = MakeControllerOutput(model, kinematics);
  controller.body_twist_command.linear.x = 0.60;
  controller.body_twist_command.angular.z = 1.50;

  CoordinatorOutput output;
  for (std::size_t step = 0U; step < 40U; ++step) {
    output = coordinator.Update(
      controller, kinematics, start + std::chrono::milliseconds(20 * step), 0.02);
  }

  ASSERT_TRUE(output.accepted);
  const double right = output.wheel_velocity_targets[ToIndex(LegId::kFrontRight)];
  const double left = output.wheel_velocity_targets[ToIndex(LegId::kFrontLeft)];
  EXPECT_NEAR(right, model.GetConfig().coordinator.max_wheel_speed, 1.0E-12);
  EXPECT_GT(left, 0.0);
  EXPECT_LT(left, right);
  EXPECT_TRUE(output.command_limited);
}

TEST(WheelLegCoordinatorTest, ReportsUnsupportedLateralVelocity)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  const auto now = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  auto controller = MakeControllerOutput(model, kinematics);
  controller.body_twist_command.linear.y = 0.20;

  const auto output = coordinator.Update(controller, kinematics, now, 0.01);

  EXPECT_TRUE(output.accepted);
  EXPECT_TRUE(output.command_limited);
}

TEST(WheelLegCoordinatorTest, DisabledControllerProducesDampingCommand)
{
  const BodyModel model(BODY_CONFIG_PATH);
  WheelLegCoordinator coordinator(model);
  ControllerOutput controller;
  controller.active_mode = ControlMode::kDisabled;
  controller.accepted = true;

  const auto output = coordinator.Update(
    controller, BodyKinematicState{}, Clock::now(), -1.0);

  EXPECT_TRUE(output.accepted);
  for (const auto & command : output.joint_command_frame.joints) {
    EXPECT_EQ(command.control_mode, actuator::ControlMode::kDamping);
  }
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
