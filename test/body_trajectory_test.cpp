#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_model.h"
#include "wheel_dog_mujoco/body/body_trajectory.h"

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

Vector3 RpyFromQuaternion(const Quaternion & q)
{
  return Vector3{
    std::atan2(
      2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
    std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)),
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

BodyState MakeState()
{
  BodyState state;
  state.pose_world.position = Vector3{1.0, 2.0, 0.30};
  state.pose_world.orientation = Quaternion{};
  state.valid = true;
  return state;
}

BodyCommandFrame MakeTarget(
  const ControlMode mode, const Clock::time_point timestamp, const std::uint64_t sequence = 7U)
{
  BodyCommandFrame target;
  target.command.control_mode = mode;
  target.command.pose_frame = ReferenceFrame::kWorld;
  target.command.twist_frame = ReferenceFrame::kBody;
  target.command.desired_pose = MakeState().pose_world;
  target.created_at = timestamp;
  target.sequence = sequence;
  return target;
}

TEST(BodyTrajectoryTest, SmoothsPoseAndOrientationTargetsToConvergence)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  const BodyState state = MakeState();
  const auto start = Clock::now();
  auto target = MakeTarget(ControlMode::kPose, start);
  target.command.desired_pose.position.z = 0.45;
  target.command.desired_pose.orientation = QuaternionFromRpy(0.20, -0.10, 0.30);

  auto output = trajectory.Update(target, state, start, 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_TRUE(output.command_limited);
  EXPECT_FALSE(output.target_reached);
  EXPECT_NEAR(output.command_frame.command.desired_pose.position.z, 0.3001, 1.0E-12);
  EXPECT_NEAR(output.pose_position_rate_world.z, 0.01, 1.0E-12);
  EXPECT_NEAR(output.pose_position_acceleration_world.z, 1.0, 1.0E-12);
  EXPECT_NEAR(output.pose_rpy_rate.x, 0.03, 1.0E-12);

  for (std::size_t step = 1U; step <= 1000U && !output.target_reached; ++step) {
    output = trajectory.Update(
      target, state, start + std::chrono::milliseconds(10 * step), 0.01);
  }

  ASSERT_TRUE(output.target_reached);
  EXPECT_NEAR(output.command_frame.command.desired_pose.position.z, 0.45, 1.0E-4);
  const Vector3 final_rpy = RpyFromQuaternion(
    output.command_frame.command.desired_pose.orientation);
  EXPECT_NEAR(final_rpy.x, 0.20, 1.0E-4);
  EXPECT_NEAR(final_rpy.y, -0.10, 1.0E-4);
  EXPECT_NEAR(final_rpy.z, 0.30, 1.0E-4);
}

TEST(BodyTrajectoryTest, LimitsVelocityTargetAccelerationAndReversal)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  const BodyState state = MakeState();
  const auto start = Clock::now();
  auto target = MakeTarget(ControlMode::kVelocity, start);
  target.command.desired_twist.linear.x = 0.60;
  target.command.desired_twist.angular.z = 1.0;

  const auto forward = trajectory.Update(target, state, start, 0.01);

  ASSERT_TRUE(forward.accepted);
  EXPECT_NEAR(forward.command_frame.command.desired_twist.linear.x, 0.015, 1.0E-12);
  EXPECT_NEAR(forward.command_frame.command.desired_twist.angular.z, 0.04, 1.0E-12);
  EXPECT_NEAR(
    forward.command_frame.command.acceleration_feedforward.linear.x, 1.50, 1.0E-12);
  EXPECT_NEAR(
    forward.command_frame.command.acceleration_feedforward.angular.z, 4.0, 1.0E-12);

  target.command.desired_twist.linear.x = -0.60;
  const auto reversing = trajectory.Update(
    target, state, start + std::chrono::milliseconds(10), 0.01);

  ASSERT_TRUE(reversing.accepted);
  EXPECT_NEAR(reversing.command_frame.command.desired_twist.linear.x, 0.0, 1.0E-12);
  EXPECT_NEAR(
    reversing.command_frame.command.acceleration_feedforward.linear.x, -1.50, 1.0E-12);
}

TEST(BodyTrajectoryTest, HybridModeSeparatesPoseAndVelocityAxes)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  BodyState state = MakeState();
  state.pose_world.orientation = QuaternionFromRpy(0.0, 0.0, 0.50);
  const auto now = Clock::now();
  auto target = MakeTarget(ControlMode::kHybrid, now);
  target.command.desired_pose.position = Vector3{10.0, 20.0, 0.40};
  target.command.desired_pose.orientation = QuaternionFromRpy(0.20, -0.10, 1.50);
  target.command.desired_twist.linear = Vector3{0.40, 0.20, 9.0};
  target.command.desired_twist.angular = Vector3{9.0, 9.0, 1.0};

  const auto output = trajectory.Update(target, state, now, 0.01);

  ASSERT_TRUE(output.accepted);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_pose.position.y, 2.0);
  EXPECT_NEAR(output.command_frame.command.desired_pose.position.z, 0.3001, 1.0E-12);
  const Vector3 output_rpy = RpyFromQuaternion(
    output.command_frame.command.desired_pose.orientation);
  EXPECT_NEAR(output_rpy.z, 0.50, 1.0E-12);
  EXPECT_NEAR(output.command_frame.command.desired_twist.linear.x, 0.015, 1.0E-12);
  EXPECT_NEAR(output.command_frame.command.desired_twist.linear.y, 0.01, 1.0E-12);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_twist.angular.y, 0.0);
  EXPECT_NEAR(output.command_frame.command.desired_twist.angular.z, 0.04, 1.0E-12);
}

TEST(BodyTrajectoryTest, DisabledModeBypassesSmoothingAndStateRequirements)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  BodyState invalid_state;
  const auto now = Clock::now();
  const auto target = MakeTarget(ControlMode::kDisabled, now, 91U);

  const auto output = trajectory.Update(target, invalid_state, now, -1.0);

  EXPECT_TRUE(output.accepted);
  EXPECT_TRUE(output.target_reached);
  EXPECT_EQ(output.source_sequence, 91U);
  EXPECT_EQ(output.command_frame.command.control_mode, ControlMode::kDisabled);
  EXPECT_DOUBLE_EQ(output.command_frame.command.desired_twist.linear.x, 0.0);
}

TEST(BodyTrajectoryTest, RejectsAmbiguousBodyFramePoseAndInvalidInitialState)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  const auto now = Clock::now();
  auto target = MakeTarget(ControlMode::kPose, now);
  target.command.pose_frame = ReferenceFrame::kBody;

  const auto ambiguous = trajectory.Update(target, MakeState(), now, 0.01);

  EXPECT_FALSE(ambiguous.accepted);
  EXPECT_TRUE(HasFault(ambiguous.faults, Fault::kInvalidCommand));

  target = MakeTarget(ControlMode::kVelocity, now);
  const BodyState invalid_state;
  const auto invalid = trajectory.Update(target, invalid_state, now, 0.01);
  EXPECT_FALSE(invalid.accepted);
  EXPECT_TRUE(HasFault(invalid.faults, Fault::kStateEstimateInvalid));
}

TEST(BodyTrajectoryTest, CapsLargeTimeStepAndUnreachableVelocityRequest)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyTrajectory trajectory(model.GetConfig());
  const BodyState state = MakeState();
  const auto now = Clock::now();
  auto target = MakeTarget(ControlMode::kVelocity, now);
  target.command.desired_twist.linear.x = 2.0;

  auto output = trajectory.Update(target, state, now, 0.10);

  ASSERT_TRUE(output.accepted);
  EXPECT_TRUE(output.time_step_limited);
  EXPECT_TRUE(output.command_limited);
  EXPECT_FALSE(output.target_reached);
  EXPECT_NEAR(output.command_frame.command.desired_twist.linear.x, 0.03, 1.0E-12);

  for (std::size_t step = 1U; step <= 100U; ++step) {
    output = trajectory.Update(
      target, state, now + std::chrono::milliseconds(20 * step), 0.02);
  }
  EXPECT_NEAR(output.command_frame.command.desired_twist.linear.x, 0.60, 1.0E-12);
  EXPECT_TRUE(output.command_limited);
  EXPECT_FALSE(output.target_reached);
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
