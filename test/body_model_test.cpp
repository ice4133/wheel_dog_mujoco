#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_model.h"

namespace wheel_dog_mujoco::body
{
namespace
{

double Component(const Vector3 & vector, const std::size_t index)
{
  switch (index) {
    case 0:
      return vector.x;
    case 1:
      return vector.y;
    default:
      return vector.z;
  }
}

LegJointVector OffsetJoint(
  LegJointVector joints, const std::size_t index, const double offset)
{
  switch (index) {
    case 0:
      joints.hip += offset;
      break;
    case 1:
      joints.thigh += offset;
      break;
    default:
      joints.calf += offset;
      break;
  }
  return joints;
}

TEST(BodyModelTest, SolvesForwardInverseKinematicsRoundTripForEveryLeg)
{
  const BodyModel model(BODY_CONFIG_PATH);
  constexpr std::array<LegJointVector, 2> poses{{
    {0.0, 0.67, -1.30},
    {0.15, 1.20, -2.20},
  }};

  for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index) {
    const auto leg_id = static_cast<LegId>(leg_index);
    for (const auto & pose : poses) {
      const Vector3 wheel_center = model.ForwardKinematics(leg_id, pose);
      const auto solution = model.SolveInverseKinematics(leg_id, wheel_center);

      ASSERT_TRUE(solution.reachable);
      EXPECT_NEAR(solution.position_error, 0.0, 1.0E-8);
      EXPECT_NEAR(solution.joint_positions.hip, pose.hip, 1.0E-8);
      EXPECT_NEAR(solution.joint_positions.thigh, pose.thigh, 1.0E-8);
      EXPECT_NEAR(solution.joint_positions.calf, pose.calf, 1.0E-8);
    }
  }
}

TEST(BodyModelTest, AnalyticJacobianMatchesFiniteDifference)
{
  const BodyModel model(BODY_CONFIG_PATH);
  constexpr double epsilon = 1.0E-6;
  const LegJointVector pose{0.12, 0.75, -1.45};

  for (std::size_t leg_index = 0; leg_index < kLegCount; ++leg_index) {
    const auto leg_id = static_cast<LegId>(leg_index);
    const auto jacobian = model.ComputeJacobian(leg_id, pose);
    for (std::size_t column = 0; column < 3; ++column) {
      const Vector3 positive = model.ForwardKinematics(
        leg_id, OffsetJoint(pose, column, epsilon));
      const Vector3 negative = model.ForwardKinematics(
        leg_id, OffsetJoint(pose, column, -epsilon));
      const Vector3 numerical{
        (positive.x - negative.x) / (2.0 * epsilon),
        (positive.y - negative.y) / (2.0 * epsilon),
        (positive.z - negative.z) / (2.0 * epsilon)};
      for (std::size_t row = 0; row < 3; ++row) {
        EXPECT_NEAR(jacobian[row][column], Component(numerical, row), 1.0E-8);
      }
    }
  }
}

TEST(BodyModelTest, RejectsGeometricallyUnreachableTarget)
{
  const BodyModel model(BODY_CONFIG_PATH);
  for (std::size_t index = 0; index < kLegCount; ++index) {
    const auto leg_id = static_cast<LegId>(index);
    const Vector3 hip_position = model.GetGeometry().hip_positions_body[index];
    EXPECT_FALSE(model.SolveInverseKinematics(leg_id, hip_position).reachable);
  }
}

TEST(BodyModelTest, MapsSemanticActuatorStateIntoLegState)
{
  const BodyModel model(BODY_CONFIG_PATH);
  actuator::JointStateFrame frame;
  frame.sequence = 123U;
  frame.received_at = std::chrono::steady_clock::now();
  for (auto & joint : frame.joints) {
    joint.online = true;
  }

  constexpr std::array<std::array<actuator::JointId, 4>, kLegCount> ids{{
    {actuator::JointId::kFrontRightHip, actuator::JointId::kFrontRightThigh,
      actuator::JointId::kFrontRightCalf, actuator::JointId::kFrontRightWheel},
    {actuator::JointId::kFrontLeftHip, actuator::JointId::kFrontLeftThigh,
      actuator::JointId::kFrontLeftCalf, actuator::JointId::kFrontLeftWheel},
    {actuator::JointId::kRearRightHip, actuator::JointId::kRearRightThigh,
      actuator::JointId::kRearRightCalf, actuator::JointId::kRearRightWheel},
    {actuator::JointId::kRearLeftHip, actuator::JointId::kRearLeftThigh,
      actuator::JointId::kRearLeftCalf, actuator::JointId::kRearLeftWheel},
  }};

  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    frame.joints[actuator::ToIndex(ids[leg][0])].position = 0.01 * leg;
    frame.joints[actuator::ToIndex(ids[leg][1])].position = 0.70 + 0.01 * leg;
    frame.joints[actuator::ToIndex(ids[leg][2])].position = -1.30 - 0.01 * leg;
    frame.joints[actuator::ToIndex(ids[leg][3])].velocity = 0.50 + leg;
  }

  const auto state = model.ComputeKinematicState(frame);

  ASSERT_TRUE(state.valid);
  EXPECT_EQ(state.sequence, frame.sequence);
  EXPECT_EQ(state.received_at, frame.received_at);
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    EXPECT_NEAR(state.legs[leg].joint_positions.hip, 0.01 * leg, 1.0E-12);
    EXPECT_NEAR(state.legs[leg].joint_positions.thigh, 0.70 + 0.01 * leg, 1.0E-12);
    EXPECT_NEAR(state.legs[leg].joint_positions.calf, -1.30 - 0.01 * leg, 1.0E-12);
    EXPECT_NEAR(state.legs[leg].wheel_velocity, 0.50 + leg, 1.0E-12);
  }
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
