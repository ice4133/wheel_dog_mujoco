#include <array>
#include <chrono>
#include <cstddef>
#include <limits>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/body/body_model.h"
#include "wheel_dog_mujoco/body/body_state_estimator.h"

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

actuator::JointStateFrame MakeJointState(const Clock::time_point timestamp)
{
  actuator::JointStateFrame frame;
  frame.received_at = timestamp;
  frame.sequence = 17U;
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

BodySensorFrame MakeSensorFrame(const Clock::time_point timestamp)
{
  BodySensorFrame frame;
  frame.received_at = timestamp;
  frame.sequence = 23U;
  frame.imu.orientation_world_from_body = Quaternion{};
  frame.imu.angular_velocity_body = Vector3{};
  frame.imu.linear_acceleration_body = Vector3{0.0, 0.0, 9.81};
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

TEST(BodyStateEstimatorTest, EstimatesStationaryStateAndRemovesGravity)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto now = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  const auto sensor = MakeSensorFrame(now);

  const EstimatorOutput output = estimator.Update(kinematics, sensor, now, 0.002);

  ASSERT_TRUE(output.state.valid);
  EXPECT_EQ(output.faults, ToMask(Fault::kNone));
  EXPECT_EQ(output.sequence, 1U);
  EXPECT_DOUBLE_EQ(output.state.pose_world.position.x, 0.0);
  EXPECT_DOUBLE_EQ(output.state.pose_world.position.y, 0.0);
  EXPECT_NEAR(output.state.twist_body.linear.x, 0.0, 1.0E-12);
  EXPECT_NEAR(output.state.acceleration_body.linear.z, 0.0, 1.0E-12);

  double expected_height = 0.0;
  for (const auto & leg : kinematics.legs) {
    expected_height += model.GetGeometry().wheel_radius - leg.wheel_center_position_body.z;
  }
  expected_height /= static_cast<double>(kLegCount);
  EXPECT_NEAR(output.state.ground_clearance, expected_height, 1.0E-12);
  EXPECT_NEAR(output.state.pose_world.position.z, expected_height, 1.0E-12);
}

TEST(BodyStateEstimatorTest, EstimatesRollingVelocityAndIntegratesAfterInitialization)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto first_time = Clock::now();
  auto joint_state = MakeJointState(first_time);
  for (const auto wheel_id : kWheelJointIds) {
    joint_state.joints[actuator::ToIndex(wheel_id)].velocity = 2.0;
  }
  auto kinematics = model.ComputeKinematicState(joint_state);
  auto sensor = MakeSensorFrame(first_time);

  const auto first = estimator.Update(kinematics, sensor, first_time, 0.01);
  ASSERT_TRUE(first.state.valid);
  const double expected_velocity = 2.0 * model.GetGeometry().wheel_radius;
  EXPECT_NEAR(first.state.twist_body.linear.x, expected_velocity, 1.0E-12);
  EXPECT_DOUBLE_EQ(first.state.pose_world.position.x, 0.0);

  const auto second_time = first_time + std::chrono::milliseconds(10);
  kinematics.received_at = second_time;
  sensor.received_at = second_time;
  const auto second = estimator.Update(kinematics, sensor, second_time, 0.01);
  ASSERT_TRUE(second.state.valid);
  EXPECT_NEAR(second.state.twist_body.linear.x, expected_velocity, 1.0E-12);
  EXPECT_NEAR(second.state.pose_world.position.x, expected_velocity * 0.01, 1.0E-12);
}

TEST(BodyStateEstimatorTest, AccountsForBodyYawWhenUsingWheelContactConstraints)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto now = Clock::now();
  auto joint_state = MakeJointState(now);
  const auto stationary_kinematics = model.ComputeKinematicState(joint_state);
  constexpr double yaw_rate = 0.4;
  double mean_longitudinal_position = 0.0;
  for (std::size_t index = 0; index < kLegCount; ++index) {
    const double lateral_position =
      stationary_kinematics.legs[index].wheel_center_position_body.y;
    mean_longitudinal_position +=
      stationary_kinematics.legs[index].wheel_center_position_body.x;
    joint_state.joints[actuator::ToIndex(kWheelJointIds[index])].velocity =
      -yaw_rate * lateral_position / model.GetGeometry().wheel_radius;
  }
  mean_longitudinal_position /= static_cast<double>(kLegCount);
  const auto kinematics = model.ComputeKinematicState(joint_state);
  auto sensor = MakeSensorFrame(now);
  sensor.imu.angular_velocity_body.z = yaw_rate;

  const auto output = estimator.Update(kinematics, sensor, now, 0.002);

  ASSERT_TRUE(output.state.valid);
  EXPECT_NEAR(output.state.twist_body.linear.x, 0.0, 1.0E-12);
  EXPECT_NEAR(
    output.state.twist_body.linear.y,
    -yaw_rate * mean_longitudinal_position, 1.0E-12);
  EXPECT_NEAR(output.state.twist_body.angular.z, yaw_rate, 1.0E-12);
  EXPECT_FALSE(HasFault(output.faults, Fault::kBodyVelocityLimit));
}

TEST(BodyStateEstimatorTest, RejectsStaleActuatorStateWithoutCorruptingStoredState)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto now = Clock::now();
  auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  auto sensor = MakeSensorFrame(now);
  ASSERT_TRUE(estimator.Update(kinematics, sensor, now, 0.002).state.valid);

  const auto later = now + std::chrono::milliseconds(500);
  sensor.received_at = later;
  const auto output = estimator.Update(kinematics, sensor, later, 0.002);

  EXPECT_FALSE(output.state.valid);
  EXPECT_TRUE(HasFault(output.faults, Fault::kActuatorStateTimeout));
  EXPECT_TRUE(HasFault(output.faults, Fault::kStateEstimateInvalid));
  EXPECT_TRUE(estimator.GetState().valid);
}

TEST(BodyStateEstimatorTest, RejectsInvalidImuSample)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto now = Clock::now();
  const auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  auto sensor = MakeSensorFrame(now);
  sensor.imu.orientation_valid = false;

  const auto output = estimator.Update(kinematics, sensor, now, 0.002);

  EXPECT_FALSE(output.state.valid);
  EXPECT_TRUE(HasFault(output.faults, Fault::kSensorStateInvalid));
  EXPECT_TRUE(HasFault(output.faults, Fault::kStateEstimateInvalid));
}

TEST(BodyStateEstimatorTest, RejectsNonFiniteKinematicSample)
{
  const BodyModel model(BODY_CONFIG_PATH);
  BodyStateEstimator estimator(model.GetConfig());
  const auto now = Clock::now();
  auto kinematics = model.ComputeKinematicState(MakeJointState(now));
  kinematics.legs[0].wheel_velocity = std::numeric_limits<double>::quiet_NaN();
  const auto sensor = MakeSensorFrame(now);

  const auto output = estimator.Update(kinematics, sensor, now, 0.002);

  EXPECT_FALSE(output.state.valid);
  EXPECT_TRUE(HasFault(output.faults, Fault::kActuatorStateInvalid));
  EXPECT_TRUE(HasFault(output.faults, Fault::kStateEstimateInvalid));
}

}  // namespace
}  // namespace wheel_dog_mujoco::body
