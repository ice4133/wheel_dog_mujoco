#include <chrono>
#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_controller.h"
#include "wheel_dog_mujoco/body/body_model.h"

namespace wheel_dog_mujoco::body
{
namespace
{

using Clock = std::chrono::steady_clock;

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

BodyState MakeState()
{
  BodyState state;
  state.pose_world.position.z = 0.30;
  state.pose_world.orientation = Quaternion{};
  state.ground_clearance = 0.30;
  state.valid = true;
  return state;
}

TrajectoryOutput MakeReference(const ControlMode mode)
{
  TrajectoryOutput reference;
  reference.command_frame.command.control_mode = mode;
  reference.command_frame.command.pose_frame = ReferenceFrame::kWorld;
  reference.command_frame.command.twist_frame = ReferenceFrame::kBody;
  reference.command_frame.command.desired_pose.position.z = 0.30;
  reference.command_frame.command.desired_pose.orientation = Quaternion{};
  reference.command_frame.sequence = 31U;
  reference.accepted = true;
  reference.target_reached = true;
  return reference;
}

TEST(BodyControllerTest, ExtendsAllLegsWhenBodyIsTooLow)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  const BodyState state = MakeState();
  auto reference = MakeReference(ControlMode::kHybrid);
  reference.command_frame.command.desired_pose.position.z = 0.35;

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  const double first_extension = output.legs[0].extension_offset;
  EXPECT_GT(first_extension, 0.0);
  for (const auto & leg : output.legs) {
    EXPECT_NEAR(leg.extension_offset, first_extension, 1.0E-12);
  }
}

TEST(BodyControllerTest, UsesFrontAndRearLegsToCorrectForwardPitch)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState state = MakeState();
  state.pose_world.orientation = QuaternionFromRpy(0.0, 0.10, 0.0);
  const auto reference = MakeReference(ControlMode::kHybrid);

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(output.legs[ToIndex(LegId::kFrontRight)].extension_offset, 0.0);
  EXPECT_GT(output.legs[ToIndex(LegId::kFrontLeft)].extension_offset, 0.0);
  EXPECT_LT(output.legs[ToIndex(LegId::kRearRight)].extension_offset, 0.0);
  EXPECT_LT(output.legs[ToIndex(LegId::kRearLeft)].extension_offset, 0.0);
}

TEST(BodyControllerTest, UsesLeftAndRightLegsToCorrectLeftLean)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState state = MakeState();
  state.pose_world.orientation = QuaternionFromRpy(-0.10, 0.0, 0.0);
  const auto reference = MakeReference(ControlMode::kHybrid);

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(output.legs[ToIndex(LegId::kFrontLeft)].extension_offset, 0.0);
  EXPECT_GT(output.legs[ToIndex(LegId::kRearLeft)].extension_offset, 0.0);
  EXPECT_LT(output.legs[ToIndex(LegId::kFrontRight)].extension_offset, 0.0);
  EXPECT_LT(output.legs[ToIndex(LegId::kRearRight)].extension_offset, 0.0);
}

TEST(BodyControllerTest, PitchRateFeedbackProducesDampingCompensation)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState state = MakeState();
  state.twist_body.angular.y = 0.50;
  const auto reference = MakeReference(ControlMode::kHybrid);

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(output.legs[ToIndex(LegId::kFrontRight)].extension_offset, 0.0);
  EXPECT_LT(output.legs[ToIndex(LegId::kRearRight)].extension_offset, 0.0);
}

TEST(BodyControllerTest, LateralAccelerationLeansBodyIntoTurn)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState state = MakeState();
  state.acceleration_body.linear.y = 2.0;
  const auto reference = MakeReference(ControlMode::kHybrid);

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(output.filtered_lateral_acceleration, 0.0);
  EXPECT_LT(output.turning_roll_compensation, 0.0);
  EXPECT_LT(
    output.legs[ToIndex(LegId::kFrontLeft)].extension_offset,
    output.legs[ToIndex(LegId::kFrontRight)].extension_offset);
}

TEST(BodyControllerTest, CorrectsPlanarVelocityWithoutExceedingBodyLimits)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState state = MakeState();
  state.twist_body.linear.x = 0.10;
  state.twist_body.angular.z = 0.10;
  auto reference = MakeReference(ControlMode::kHybrid);
  reference.command_frame.command.desired_twist.linear.x = 0.30;
  reference.command_frame.command.desired_twist.angular.z = 0.40;

  const auto output = controller.Update(reference, state, Clock::now(), 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_GT(output.body_twist_command.linear.x, 0.30);
  EXPECT_LE(
    output.body_twist_command.linear.x, model.GetConfig().limits.max_linear_velocity.x);
  EXPECT_GT(output.body_twist_command.angular.z, 0.40);
  EXPECT_LE(
    output.body_twist_command.angular.z, model.GetConfig().limits.max_angular_velocity.z);
}

TEST(BodyControllerTest, DisabledIsImmediateAndInvalidStateIsRejected)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());
  BodyState invalid_state;
  const auto now = Clock::now();

  const auto disabled = controller.Update(
    MakeReference(ControlMode::kDisabled), invalid_state, now, -1.0);
  ASSERT_TRUE(disabled.accepted);
  EXPECT_EQ(disabled.active_mode, ControlMode::kDisabled);

  const auto rejected = controller.Update(
    MakeReference(ControlMode::kHybrid), invalid_state, now, 0.01);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_TRUE(HasFault(rejected.faults, Fault::kStateEstimateInvalid));
  EXPECT_TRUE(HasFault(rejected.faults, Fault::kControllerFailure));
}

TEST(BodyControllerTest, ReportsWhenLargeTimeStepIsCapped)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyController controller(model.GetConfig());

  const auto output = controller.Update(
    MakeReference(ControlMode::kHybrid), MakeState(), Clock::now(), 0.10);

  EXPECT_TRUE(output.accepted);
  EXPECT_TRUE(output.time_step_limited);
  EXPECT_TRUE(output.command_limited);
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
