#pragma once

#include <array>
#include <string>

#include "wheel_dog_mujoco/actuator/actuator_type.h"
#include "wheel_dog_mujoco/body/body_type.h"

namespace wheel_dog_mujoco::body
{

struct LegJointVector
{
  double hip{0.0};
  double thigh{0.0};
  double calf{0.0};
};

using LegJacobian = std::array<std::array<double, 3>, 3>;

struct InverseKinematicsResult
{
  LegJointVector joint_positions{};
  double position_error{0.0};
  bool reachable{false};
};

struct JointVelocityResult
{
  LegJointVector joint_velocities{};
  bool valid{false};
};

struct LegKinematicState
{
  LegJointVector joint_positions{};
  LegJointVector joint_velocities{};
  Vector3 wheel_center_position_body{};
  Vector3 wheel_center_velocity_body{};
  double wheel_position{0.0};
  double wheel_velocity{0.0};
  bool valid{false};
};

struct BodyKinematicState
{
  std::array<LegKinematicState, kLegCount> legs{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0};
  bool valid{false};
};

// BodyModel is the semantic bridge between actuator joint state and body-level
// wheel-leg kinematics. It contains no controller state and performs no I/O.
class BodyModel
{
public:
  explicit BodyModel(const std::string & config_path);

  const BodyConfig & GetConfig() const noexcept;
  const BodyGeometry & GetGeometry() const noexcept;

  Vector3 ForwardKinematics(
    LegId leg_id, const LegJointVector & joint_positions) const;
  LegJacobian ComputeJacobian(
    LegId leg_id, const LegJointVector & joint_positions) const;
  InverseKinematicsResult SolveInverseKinematics(
    LegId leg_id, const Vector3 & wheel_center_position_body) const;
  JointVelocityResult SolveJointVelocity(
    LegId leg_id, const LegJointVector & joint_positions,
    const Vector3 & wheel_center_velocity_body) const;

  LegKinematicState ComputeLegState(
    LegId leg_id, const actuator::JointStateFrame & joint_state) const;
  BodyKinematicState ComputeKinematicState(
    const actuator::JointStateFrame & joint_state) const;

private:
  static BodyConfig LoadConfig(const std::string & config_path);
  static void ValidateConfig(const BodyConfig & config);

  BodyConfig config_{};
};

}  // namespace wheel_dog_mujoco::body
