#include "wheel_dog_mujoco/body/wheel_leg_coordinator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::body
{
namespace
{

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

bool IsFinite(const double value) noexcept
{
  return std::isfinite(value);
}

bool IsFinite(const Vector3 & value) noexcept
{
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const Quaternion & value) noexcept
{
  return IsFinite(value.w) && IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const LegKinematicState & state) noexcept
{
  return state.valid && IsFinite(state.joint_positions.hip) &&
         IsFinite(state.joint_positions.thigh) && IsFinite(state.joint_positions.calf) &&
         IsFinite(state.joint_velocities.hip) && IsFinite(state.joint_velocities.thigh) &&
         IsFinite(state.joint_velocities.calf) && IsFinite(state.wheel_center_position_body) &&
         IsFinite(state.wheel_center_velocity_body) && IsFinite(state.wheel_velocity);
}

bool IsControllerOutputValid(const ControllerOutput & output) noexcept
{
  return output.accepted && IsFinite(output.stabilized_pose_world.position) &&
         IsFinite(output.stabilized_pose_world.orientation) &&
         IsFinite(output.body_twist_command.linear) &&
         IsFinite(output.body_twist_command.angular) &&
         (output.twist_frame == ReferenceFrame::kBody ||
         output.twist_frame == ReferenceFrame::kWorld) &&
         std::all_of(
    output.legs.begin(), output.legs.end(),
    [](const LegPostureAdjustment & leg) {return IsFinite(leg.extension_offset);});
}

bool IsKinematicStateValid(const BodyKinematicState & state) noexcept
{
  return state.valid && std::all_of(
    state.legs.begin(), state.legs.end(),
    [](const LegKinematicState & leg) {return IsFinite(leg);});
}

bool IsLeftLeg(const std::size_t index) noexcept
{
  return index == ToIndex(LegId::kFrontLeft) || index == ToIndex(LegId::kRearLeft);
}

double Approach(const double current, const double target, const double max_change) noexcept
{
  return current + std::clamp(target - current, -max_change, max_change);
}

void StepPosition(
  const double desired_position, const double max_velocity, const double max_acceleration,
  const double time_step, double & position, double & velocity, bool & limited) noexcept
{
  const double error = desired_position - position;
  if (std::abs(error) <= 1.0E-8 && std::abs(velocity) <= max_acceleration * time_step) {
    position = desired_position;
    velocity = 0.0;
    return;
  }
  const double stopping_velocity = std::sqrt(2.0 * max_acceleration * std::abs(error));
  const double desired_velocity = std::copysign(
    std::min(max_velocity, stopping_velocity), error);
  velocity = Approach(velocity, desired_velocity, max_acceleration * time_step);
  const double next_position = position + velocity * time_step;
  if (error * (desired_position - next_position) <= 0.0) {
    position = desired_position;
    velocity = 0.0;
  } else {
    position = next_position;
  }
  limited = true;
}

Vector3 QuaternionToRpy(const Quaternion & q) noexcept
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

Vector3 RotateWorldToBody(const Quaternion & q, const Vector3 & value) noexcept
{
  const Quaternion conjugate{q.w, -q.x, -q.y, -q.z};
  const double xx = conjugate.x * conjugate.x;
  const double yy = conjugate.y * conjugate.y;
  const double zz = conjugate.z * conjugate.z;
  const double xy = conjugate.x * conjugate.y;
  const double xz = conjugate.x * conjugate.z;
  const double yz = conjugate.y * conjugate.z;
  const double wx = conjugate.w * conjugate.x;
  const double wy = conjugate.w * conjugate.y;
  const double wz = conjugate.w * conjugate.z;
  return Vector3{
    (1.0 - 2.0 * (yy + zz)) * value.x + 2.0 * (xy - wz) * value.y +
    2.0 * (xz + wy) * value.z,
    2.0 * (xy + wz) * value.x + (1.0 - 2.0 * (xx + zz)) * value.y +
    2.0 * (yz - wx) * value.z,
    2.0 * (xz - wy) * value.x + 2.0 * (yz + wx) * value.y +
    (1.0 - 2.0 * (xx + yy)) * value.z};
}

void SetLegCommand(
  actuator::JointCommandFrame & frame, const std::size_t leg_index,
  const LegJointVector & position, const LegJointVector & velocity)
{
  const auto & ids = kLegJointIds[leg_index];
  const std::array<double, 3> positions{{position.hip, position.thigh, position.calf}};
  const std::array<double, 3> velocities{{velocity.hip, velocity.thigh, velocity.calf}};
  for (std::size_t joint = 0U; joint < ids.size(); ++joint) {
    auto & command = frame.joints[actuator::ToIndex(ids[joint])];
    command.control_mode = actuator::ControlMode::kHybrid;
    command.gain_profile = actuator::GainProfile::kNormal;
    command.position = positions[joint];
    command.velocity = velocities[joint];
  }
}

}  // namespace

WheelLegCoordinator::WheelLegCoordinator(const BodyModel & model)
: model_(model), config_(model.GetConfig())
{
  const auto & parameters = config_.coordinator;
  if (!IsFinite(parameters.max_time_step) || parameters.max_time_step <= 0.0 ||
    !IsFinite(parameters.max_wheel_speed) || parameters.max_wheel_speed <= 0.0 ||
    !IsFinite(parameters.max_wheel_acceleration) ||
    parameters.max_wheel_acceleration <= 0.0 ||
    !IsFinite(parameters.lateral_velocity_tolerance) ||
    parameters.lateral_velocity_tolerance < 0.0 ||
    !IsFinite(config_.limits.max_linear_velocity.z) ||
    config_.limits.max_linear_velocity.z <= 0.0 ||
    !IsFinite(config_.limits.max_linear_acceleration.z) ||
    config_.limits.max_linear_acceleration.z <= 0.0)
  {
    throw std::invalid_argument("WheelLegCoordinator configuration is invalid");
  }
}

CoordinatorOutput WheelLegCoordinator::Update(
  const ControllerOutput & controller_output,
  const BodyKinematicState & kinematic_state,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  if (controller_output.active_mode == ControlMode::kDisabled && controller_output.accepted) {
    initialized_ = false;
    return MakeDampingOutput(
      controller_output.faults, now, controller_output.sequence, true);
  }
  if (!IsControllerOutputValid(controller_output)) {
    initialized_ = false;
    return MakeDampingOutput(
      controller_output.faults | ToMask(Fault::kControllerFailure),
      now, controller_output.sequence, false);
  }
  if (!IsKinematicStateValid(kinematic_state)) {
    initialized_ = false;
    return MakeDampingOutput(
      ToMask(Fault::kActuatorStateInvalid), now, controller_output.sequence, false);
  }
  if (!IsFinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return MakeDampingOutput(
      ToMask(Fault::kInvalidCommand), now, controller_output.sequence, false);
  }

  if (!initialized_) {
    for (std::size_t index = 0U; index < kLegCount; ++index) {
      wheel_center_z_targets_[index] =
        kinematic_state.legs[index].wheel_center_position_body.z;
      wheel_center_z_rates_[index] =
        kinematic_state.legs[index].wheel_center_velocity_body.z;
      wheel_velocity_targets_[index] = kinematic_state.legs[index].wheel_velocity;
    }
    initialized_ = true;
  }

  const double time_step = std::min(elapsed_seconds, config_.coordinator.max_time_step);
  const bool time_step_limited = time_step < elapsed_seconds;
  bool command_limited = controller_output.command_limited || time_step_limited;
  FaultFlags faults = controller_output.faults;
  CoordinatorOutput output;

  const Vector3 desired_rpy = QuaternionToRpy(
    controller_output.stabilized_pose_world.orientation);
  const double desired_height = controller_output.stabilized_pose_world.position.z;
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    const auto & hip = config_.geometry.hip_positions_body[index];
    const double support_y = hip.y +
      (IsLeftLeg(index) ? config_.geometry.hip_link_length :
      -config_.geometry.hip_link_length);
    const double geometric_extension =
      support_y * desired_rpy.x - hip.x * desired_rpy.y;
    const double desired_center_z = config_.geometry.wheel_radius - desired_height -
      geometric_extension - controller_output.legs[index].extension_offset;
    StepPosition(
      desired_center_z, config_.limits.max_linear_velocity.z,
      config_.limits.max_linear_acceleration.z, time_step,
      wheel_center_z_targets_[index], wheel_center_z_rates_[index], command_limited);

    Vector3 target = kinematic_state.legs[index].wheel_center_position_body;
    target.z = wheel_center_z_targets_[index];
    auto inverse_kinematics = model_.SolveInverseKinematics(
      static_cast<LegId>(index), target);
    if (!inverse_kinematics.reachable) {
      Vector3 reachable_target = kinematic_state.legs[index].wheel_center_position_body;
      auto reachable_solution = model_.SolveInverseKinematics(
        static_cast<LegId>(index), reachable_target);
      if (!reachable_solution.reachable) {
        initialized_ = false;
        return MakeDampingOutput(
          faults | ToMask(Fault::kKinematicsInfeasible),
          now, controller_output.sequence, false);
      }

      // Keep the closest reachable point on this cycle's vertical segment.
      // This lets a folded leg move away from a workspace boundary instead of
      // turning one infeasible sample into a permanent damping deadlock.
      double unreachable_z = target.z;
      for (std::size_t iteration = 0U; iteration < 24U; ++iteration) {
        Vector3 candidate = target;
        candidate.z = 0.5 * (reachable_target.z + unreachable_z);
        const auto candidate_solution = model_.SolveInverseKinematics(
          static_cast<LegId>(index), candidate);
        if (candidate_solution.reachable) {
          reachable_target = candidate;
          reachable_solution = candidate_solution;
        } else {
          unreachable_z = candidate.z;
        }
      }
      target = reachable_target;
      inverse_kinematics = reachable_solution;
      wheel_center_z_targets_[index] = target.z;
      wheel_center_z_rates_[index] = 0.0;
      command_limited = true;
    }
    output.wheel_center_targets_body[index] = target;

    const auto joint_velocity = model_.SolveJointVelocity(
      static_cast<LegId>(index), inverse_kinematics.joint_positions,
      Vector3{0.0, 0.0, wheel_center_z_rates_[index]});
    LegJointVector joint_velocity_command{};
    if (joint_velocity.valid) {
      joint_velocity_command = joint_velocity.joint_velocities;
    } else {
      // Position control remains well-defined at a Jacobian singularity. The
      // actuator layer will rate-limit this target, so only omit feedforward.
      wheel_center_z_rates_[index] = 0.0;
      command_limited = true;
    }
    SetLegCommand(
      output.joint_command_frame, index,
      inverse_kinematics.joint_positions, joint_velocity_command);
  }

  Vector3 desired_linear = controller_output.body_twist_command.linear;
  Vector3 desired_angular = controller_output.body_twist_command.angular;
  if (controller_output.twist_frame == ReferenceFrame::kWorld) {
    desired_linear = RotateWorldToBody(
      controller_output.stabilized_pose_world.orientation, desired_linear);
    desired_angular = RotateWorldToBody(
      controller_output.stabilized_pose_world.orientation, desired_angular);
  }
  if (std::abs(desired_linear.y) > config_.coordinator.lateral_velocity_tolerance) {
    command_limited = true;
  }

  double half_track = 0.0;
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    const auto & hip = config_.geometry.hip_positions_body[index];
    half_track += std::abs(
      hip.y + (IsLeftLeg(index) ? config_.geometry.hip_link_length :
      -config_.geometry.hip_link_length));
  }
  half_track /= static_cast<double>(kLegCount);
  double target_right =
    (desired_linear.x + half_track * desired_angular.z) / config_.geometry.wheel_radius;
  double target_left =
    (desired_linear.x - half_track * desired_angular.z) / config_.geometry.wheel_radius;
  const double largest_speed = std::max(std::abs(target_right), std::abs(target_left));
  if (largest_speed > config_.coordinator.max_wheel_speed) {
    const double scale = config_.coordinator.max_wheel_speed / largest_speed;
    target_right *= scale;
    target_left *= scale;
    command_limited = true;
  }

  const std::array<double, kLegCount> wheel_targets{{
    target_right, target_left, target_right, target_left}};
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    wheel_velocity_targets_[index] = Approach(
      wheel_velocity_targets_[index], wheel_targets[index],
      config_.coordinator.max_wheel_acceleration * time_step);
    if (wheel_velocity_targets_[index] != wheel_targets[index]) {
      command_limited = true;
    }
    output.wheel_velocity_targets[index] = wheel_velocity_targets_[index];
    auto & command = output.joint_command_frame.joints[
      actuator::ToIndex(kWheelJointIds[index])];
    command.control_mode = actuator::ControlMode::kVelocity;
    command.gain_profile = actuator::GainProfile::kNormal;
    command.velocity = wheel_velocity_targets_[index];
  }

  const std::uint64_t output_sequence = ++sequence_;
  output.joint_command_frame.created_at = now;
  output.joint_command_frame.sequence = output_sequence;
  output.faults = faults;
  output.coordinated_at = now;
  output.source_sequence = controller_output.sequence;
  output.sequence = output_sequence;
  output.accepted = true;
  output.command_limited = command_limited;
  output.time_step_limited = time_step_limited;
  return output;
}

void WheelLegCoordinator::Reset() noexcept
{
  wheel_center_z_targets_.fill(0.0);
  wheel_center_z_rates_.fill(0.0);
  wheel_velocity_targets_.fill(0.0);
  sequence_ = 0U;
  initialized_ = false;
}

const CoordinatorParameters & WheelLegCoordinator::GetParameters() const noexcept
{
  return config_.coordinator;
}

CoordinatorOutput WheelLegCoordinator::MakeDampingOutput(
  const FaultFlags faults, const std::chrono::steady_clock::time_point now,
  const std::uint64_t source_sequence, const bool accepted)
{
  CoordinatorOutput output;
  for (auto & command : output.joint_command_frame.joints) {
    command.control_mode = actuator::ControlMode::kDamping;
    command.gain_profile = actuator::GainProfile::kSoft;
  }
  const std::uint64_t output_sequence = ++sequence_;
  output.joint_command_frame.created_at = now;
  output.joint_command_frame.sequence = output_sequence;
  output.faults = faults;
  output.coordinated_at = now;
  output.source_sequence = source_sequence;
  output.sequence = output_sequence;
  output.accepted = accepted;
  return output;
}

}  // namespace wheel_dog_mujoco::body
