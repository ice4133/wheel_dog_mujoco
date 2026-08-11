#include "wheel_dog_mujoco/body/body_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::body
{
namespace
{

constexpr double kQuaternionNormTolerance = 1.0E-12;
constexpr double kPi = 3.14159265358979323846;

struct PidResult
{
  double output{0.0};
  bool limited{false};
};

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

bool IsFinite(const Pose & value) noexcept
{
  return IsFinite(value.position) && IsFinite(value.orientation);
}

bool IsFinite(const Twist & value) noexcept
{
  return IsFinite(value.linear) && IsFinite(value.angular);
}

bool IsFinite(const SpatialAcceleration & value) noexcept
{
  return IsFinite(value.linear) && IsFinite(value.angular);
}

bool IsValid(const AxisControlGains & gains) noexcept
{
  return IsFinite(gains.kp) && gains.kp >= 0.0 && IsFinite(gains.ki) && gains.ki >= 0.0 &&
         IsFinite(gains.kd) && gains.kd >= 0.0 && IsFinite(gains.integral_limit) &&
         gains.integral_limit >= 0.0;
}

bool IsStateValid(const BodyState & state) noexcept
{
  const Quaternion & q = state.pose_world.orientation;
  const double orientation_norm_squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return state.valid && IsFinite(state.pose_world) && IsFinite(state.twist_body) &&
         IsFinite(state.acceleration_body) && IsFinite(state.ground_clearance) &&
         orientation_norm_squared > kQuaternionNormTolerance;
}

bool IsReferenceValid(const TrajectoryOutput & reference) noexcept
{
  const BodyCommand & command = reference.command_frame.command;
  const bool mode_is_valid = command.control_mode == ControlMode::kDisabled ||
    command.control_mode == ControlMode::kPose ||
    command.control_mode == ControlMode::kVelocity ||
    command.control_mode == ControlMode::kHybrid;
  return reference.accepted && mode_is_valid && IsFinite(command.desired_pose) &&
         IsFinite(command.desired_twist) && IsFinite(command.acceleration_feedforward) &&
         IsFinite(reference.pose_position_rate_world) &&
         IsFinite(reference.pose_position_acceleration_world) &&
         IsFinite(reference.pose_rpy_rate) && IsFinite(reference.pose_rpy_acceleration) &&
         (command.twist_frame == ReferenceFrame::kBody ||
         command.twist_frame == ReferenceFrame::kWorld);
}

double NormalizeAngle(double angle) noexcept
{
  constexpr double two_pi = 2.0 * kPi;
  while (angle > kPi) {
    angle -= two_pi;
  }
  while (angle < -kPi) {
    angle += two_pi;
  }
  return angle;
}

Quaternion Normalize(const Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!IsFinite(norm) || norm <= kQuaternionNormTolerance) {
    throw std::invalid_argument("Cannot normalize an invalid quaternion");
  }
  return Quaternion{
    quaternion.w / norm, quaternion.x / norm,
    quaternion.y / norm, quaternion.z / norm};
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

Quaternion RpyToQuaternion(const Vector3 & rpy) noexcept
{
  const double cr = std::cos(0.5 * rpy.x);
  const double sr = std::sin(0.5 * rpy.x);
  const double cp = std::cos(0.5 * rpy.y);
  const double sp = std::sin(0.5 * rpy.y);
  const double cy = std::cos(0.5 * rpy.z);
  const double sy = std::sin(0.5 * rpy.z);
  return Quaternion{
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy};
}

Vector3 RotateBodyToWorld(const Quaternion & q, const Vector3 & value) noexcept
{
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  return Vector3{
    (1.0 - 2.0 * (yy + zz)) * value.x + 2.0 * (xy - wz) * value.y +
    2.0 * (xz + wy) * value.z,
    2.0 * (xy + wz) * value.x + (1.0 - 2.0 * (xx + zz)) * value.y +
    2.0 * (yz - wx) * value.z,
    2.0 * (xz - wy) * value.x + 2.0 * (yz + wx) * value.y +
    (1.0 - 2.0 * (xx + yy)) * value.z};
}

Vector3 RotateWorldToBody(const Quaternion & q, const Vector3 & value) noexcept
{
  return RotateBodyToWorld(Quaternion{q.w, -q.x, -q.y, -q.z}, value);
}

Vector3 BodyAngularVelocityToRpyRate(
  const Vector3 & angular_velocity_body, const Vector3 & rpy) noexcept
{
  const double sine_roll = std::sin(rpy.x);
  const double cosine_roll = std::cos(rpy.x);
  const double cosine_pitch = std::cos(rpy.y);
  const double safe_cosine_pitch = std::copysign(
    std::max(std::abs(cosine_pitch), 1.0E-6), cosine_pitch);
  const double tangent_pitch = std::sin(rpy.y) / safe_cosine_pitch;
  return Vector3{
    angular_velocity_body.x + sine_roll * tangent_pitch * angular_velocity_body.y +
    cosine_roll * tangent_pitch * angular_velocity_body.z,
    cosine_roll * angular_velocity_body.y - sine_roll * angular_velocity_body.z,
    sine_roll / safe_cosine_pitch * angular_velocity_body.y +
    cosine_roll / safe_cosine_pitch * angular_velocity_body.z};
}

double Component(const Vector3 & vector, const std::size_t index) noexcept
{
  if (index == 0U) {
    return vector.x;
  }
  if (index == 1U) {
    return vector.y;
  }
  return vector.z;
}

void SetComponent(Vector3 & vector, const std::size_t index, const double value) noexcept
{
  if (index == 0U) {
    vector.x = value;
  } else if (index == 1U) {
    vector.y = value;
  } else {
    vector.z = value;
  }
}

PidResult ComputePid(
  const AxisControlGains & gains, const double error, const double error_rate,
  const double feedforward, double & integral, const double time_step,
  const double output_limit) noexcept
{
  double candidate_integral = integral;
  if (gains.ki > 0.0 && gains.integral_limit > 0.0) {
    candidate_integral = std::clamp(
      integral + error * time_step, -gains.integral_limit, gains.integral_limit);
  } else {
    candidate_integral = 0.0;
  }

  const double candidate_output = feedforward + gains.kp * error +
    gains.ki * candidate_integral + gains.kd * error_rate;
  const double bounded_output = std::clamp(candidate_output, -output_limit, output_limit);
  if (bounded_output == candidate_output) {
    integral = candidate_integral;
  }

  const double output = std::clamp(
    feedforward + gains.kp * error + gains.ki * integral + gains.kd * error_rate,
    -output_limit, output_limit);
  return PidResult{output, output != candidate_output};
}

bool IsLeftLeg(const std::size_t index) noexcept
{
  return index == ToIndex(LegId::kFrontLeft) || index == ToIndex(LegId::kRearLeft);
}

}  // namespace

BodyController::BodyController(const BodyConfig & config)
: config_(config)
{
  const auto & parameters = config_.controller;
  const bool gains_valid = std::all_of(
    parameters.gains.linear.begin(), parameters.gains.linear.end(),
    [](const AxisControlGains & gains) {return IsValid(gains);}) &&
    std::all_of(
    parameters.gains.angular.begin(), parameters.gains.angular.end(),
    [](const AxisControlGains & gains) {return IsValid(gains);});
  if (!gains_valid || !IsFinite(parameters.max_time_step) ||
    parameters.max_time_step <= 0.0 ||
    !IsFinite(parameters.max_leg_extension_adjustment) ||
    parameters.max_leg_extension_adjustment <= 0.0 ||
    !IsFinite(parameters.velocity_posture_time_horizon) ||
    parameters.velocity_posture_time_horizon <= 0.0 ||
    !IsFinite(parameters.turning_roll_gain) || parameters.turning_roll_gain < 0.0 ||
    !IsFinite(parameters.max_turning_roll) || parameters.max_turning_roll < 0.0 ||
    !IsFinite(parameters.lateral_acceleration_filter_coefficient) ||
    parameters.lateral_acceleration_filter_coefficient < 0.0 ||
    parameters.lateral_acceleration_filter_coefficient > 1.0 ||
    !IsFinite(config_.estimator.gravity_magnitude) || config_.estimator.gravity_magnitude <= 0.0)
  {
    throw std::invalid_argument("BodyController configuration is invalid");
  }
}

ControllerOutput BodyController::Update(
  const TrajectoryOutput & reference,
  const BodyState & state,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  const BodyCommand & command = reference.command_frame.command;
  if (command.control_mode == ControlMode::kDisabled && reference.accepted) {
    linear_integral_.fill(0.0);
    angular_integral_.fill(0.0);
    active_mode_ = ControlMode::kDisabled;
    filtered_lateral_acceleration_ = 0.0;
    initialized_ = false;

    ControllerOutput output;
    output.stabilized_pose_world = command.desired_pose;
    output.twist_frame = command.twist_frame;
    output.active_mode = ControlMode::kDisabled;
    output.evaluated_at = now;
    output.source_sequence = reference.command_frame.sequence;
    output.sequence = ++sequence_;
    output.accepted = true;
    return output;
  }

  if (!IsReferenceValid(reference)) {
    const FaultFlags faults = reference.faults | ToMask(Fault::kInvalidCommand);
    return MakeRejectedOutput(faults, now, reference.command_frame.sequence);
  }
  if (!IsStateValid(state)) {
    return MakeRejectedOutput(
      ToMask(Fault::kStateEstimateInvalid), now, reference.command_frame.sequence);
  }
  if (!IsFinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return MakeRejectedOutput(
      ToMask(Fault::kInvalidCommand), now, reference.command_frame.sequence);
  }

  const double time_step = std::min(elapsed_seconds, config_.controller.max_time_step);
  const bool time_step_limited = time_step < elapsed_seconds;
  if (!initialized_ || active_mode_ != command.control_mode ||
    active_twist_frame_ != command.twist_frame)
  {
    linear_integral_.fill(0.0);
    angular_integral_.fill(0.0);
    active_mode_ = command.control_mode;
    active_twist_frame_ = command.twist_frame;
    initialized_ = true;
  }

  const Quaternion state_orientation = Normalize(state.pose_world.orientation);
  const Vector3 state_rpy = QuaternionToRpy(state_orientation);
  const Vector3 state_world_velocity = RotateBodyToWorld(
    state_orientation, state.twist_body.linear);
  const Vector3 state_world_acceleration = RotateBodyToWorld(
    state_orientation, state.acceleration_body.linear);
  const Vector3 state_rpy_rate = BodyAngularVelocityToRpyRate(
    state.twist_body.angular, state_rpy);

  const double filter_coefficient =
    config_.controller.lateral_acceleration_filter_coefficient;
  filtered_lateral_acceleration_ =
    filter_coefficient * filtered_lateral_acceleration_ +
    (1.0 - filter_coefficient) * state.acceleration_body.linear.y;
  const double turning_roll = std::clamp(
    -config_.controller.turning_roll_gain * std::atan2(
      filtered_lateral_acceleration_, config_.estimator.gravity_magnitude),
    -config_.controller.max_turning_roll, config_.controller.max_turning_roll);

  FaultFlags faults = reference.faults;
  bool command_limited = reference.command_limited || time_step_limited;
  Pose stabilized_pose = command.desired_pose;
  Vector3 desired_rpy = QuaternionToRpy(Normalize(command.desired_pose.orientation));
  const double requested_roll = desired_rpy.x + turning_roll;
  desired_rpy.x = std::clamp(
    requested_roll, -config_.limits.max_roll, config_.limits.max_roll);
  const double requested_pitch = desired_rpy.y;
  desired_rpy.y = std::clamp(
    requested_pitch, -config_.limits.max_pitch, config_.limits.max_pitch);
  const double requested_height = stabilized_pose.position.z;
  stabilized_pose.position.z = std::clamp(
    requested_height, config_.limits.min_ground_clearance,
    config_.limits.max_ground_clearance);
  if (desired_rpy.x != requested_roll || desired_rpy.y != requested_pitch ||
    stabilized_pose.position.z != requested_height)
  {
    faults |= ToMask(Fault::kBodyPoseLimit);
    command_limited = true;
  }
  stabilized_pose.orientation = RpyToQuaternion(desired_rpy);

  const Vector3 actual_twist_linear = command.twist_frame == ReferenceFrame::kBody ?
    state.twist_body.linear : state_world_velocity;
  const Vector3 actual_twist_angular = command.twist_frame == ReferenceFrame::kBody ?
    state.twist_body.angular : RotateBodyToWorld(
    state_orientation, state.twist_body.angular);
  const Vector3 actual_acceleration_linear = command.twist_frame == ReferenceFrame::kBody ?
    state.acceleration_body.linear : state_world_acceleration;
  const Vector3 actual_acceleration_angular = command.twist_frame == ReferenceFrame::kBody ?
    state.acceleration_body.angular : RotateBodyToWorld(
    state_orientation, state.acceleration_body.angular);

  Twist twist_command{};
  SpatialAcceleration acceleration_feedforward{};
  double height_correction = 0.0;
  double roll_correction = 0.0;
  double pitch_correction = 0.0;

  const auto update_velocity_axis = [&](const std::size_t index, const bool angular) {
      const Vector3 & desired = angular ? command.desired_twist.angular :
        command.desired_twist.linear;
      const Vector3 & actual = angular ? actual_twist_angular : actual_twist_linear;
      const Vector3 & desired_acceleration = angular ?
        command.acceleration_feedforward.angular : command.acceleration_feedforward.linear;
      const Vector3 & actual_acceleration = angular ?
        actual_acceleration_angular : actual_acceleration_linear;
      const Vector3 & limit = angular ? config_.limits.max_angular_velocity :
        config_.limits.max_linear_velocity;
      auto & integral = angular ? angular_integral_[index] : linear_integral_[index];
      const AxisControlGains & gains = angular ? config_.controller.gains.angular[index] :
        config_.controller.gains.linear[index];
      const PidResult result = ComputePid(
        gains, Component(desired, index) - Component(actual, index),
        Component(desired_acceleration, index) - Component(actual_acceleration, index),
        Component(desired, index), integral, time_step, Component(limit, index));
      command_limited = command_limited || result.limited;
      return result.output;
    };

  const auto update_pose_translation_axis = [&](const std::size_t index) {
      const double error = Component(stabilized_pose.position, index) -
        Component(state.pose_world.position, index);
      const double rate_error = Component(reference.pose_position_rate_world, index) -
        Component(state_world_velocity, index);
      return ComputePid(
        config_.controller.gains.linear[index], error, rate_error,
        Component(reference.pose_position_rate_world, index), linear_integral_[index],
        time_step, Component(config_.limits.max_linear_velocity, index));
    };

  const auto update_pose_rotation_axis = [&](const std::size_t index) {
      const double error = NormalizeAngle(
        Component(desired_rpy, index) - Component(state_rpy, index));
      const double rate_error = Component(reference.pose_rpy_rate, index) -
        Component(state_rpy_rate, index);
      return ComputePid(
        config_.controller.gains.angular[index], error, rate_error,
        Component(reference.pose_rpy_rate, index), angular_integral_[index],
        time_step, Component(config_.limits.max_angular_velocity, index));
    };

  if (command.control_mode == ControlMode::kPose) {
    Vector3 pose_linear_command_world{};
    for (std::size_t index = 0U; index < 2U; ++index) {
      const PidResult result = update_pose_translation_axis(index);
      command_limited = command_limited || result.limited;
      SetComponent(pose_linear_command_world, index, result.output);
    }
    const Vector3 pose_linear_command = command.twist_frame == ReferenceFrame::kBody ?
      RotateWorldToBody(state_orientation, pose_linear_command_world) :
      pose_linear_command_world;
    twist_command.linear.x = pose_linear_command.x;
    twist_command.linear.y = pose_linear_command.y;
    const PidResult height = ComputePid(
      config_.controller.gains.linear[2],
      stabilized_pose.position.z - state.ground_clearance,
      reference.pose_position_rate_world.z - state_world_velocity.z, 0.0,
      linear_integral_[2], time_step,
      config_.controller.max_leg_extension_adjustment);
    const PidResult roll = update_pose_rotation_axis(0U);
    const PidResult pitch = update_pose_rotation_axis(1U);
    const PidResult yaw = update_pose_rotation_axis(2U);
    height_correction = height.output;
    roll_correction = roll.output;
    pitch_correction = pitch.output;
    twist_command.angular.z = yaw.output;
    command_limited = command_limited || height.limited || roll.limited ||
      pitch.limited || yaw.limited;

    Vector3 pose_acceleration = reference.pose_position_acceleration_world;
    if (command.twist_frame == ReferenceFrame::kBody) {
      pose_acceleration = RotateWorldToBody(state_orientation, pose_acceleration);
    }
    acceleration_feedforward.linear = pose_acceleration;
    acceleration_feedforward.angular.z = reference.pose_rpy_acceleration.z;
  } else {
    twist_command.linear.x = update_velocity_axis(0U, false);
    twist_command.linear.y = update_velocity_axis(1U, false);
    twist_command.angular.z = update_velocity_axis(2U, true);
    acceleration_feedforward = command.acceleration_feedforward;

    if (command.control_mode == ControlMode::kHybrid) {
      const PidResult height = ComputePid(
        config_.controller.gains.linear[2],
        stabilized_pose.position.z - state.ground_clearance,
        reference.pose_position_rate_world.z - state_world_velocity.z, 0.0,
        linear_integral_[2], time_step,
        config_.controller.max_leg_extension_adjustment);
      const PidResult roll = update_pose_rotation_axis(0U);
      const PidResult pitch = update_pose_rotation_axis(1U);
      height_correction = height.output;
      roll_correction = roll.output;
      pitch_correction = pitch.output;
      command_limited = command_limited || height.limited || roll.limited || pitch.limited;
    } else {
      const double vertical_velocity = update_velocity_axis(2U, false);
      const double roll_velocity = update_velocity_axis(0U, true);
      const double pitch_velocity = update_velocity_axis(1U, true);
      const double horizon = config_.controller.velocity_posture_time_horizon;
      height_correction = horizon * vertical_velocity;
      roll_correction = turning_roll + horizon * roll_velocity;
      pitch_correction = horizon * pitch_velocity;
    }
  }

  ControllerOutput output;
  const double extension_limit = config_.controller.max_leg_extension_adjustment;
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    const auto & hip = config_.geometry.hip_positions_body[index];
    const double support_y = hip.y +
      (IsLeftLeg(index) ? config_.geometry.hip_link_length :
      -config_.geometry.hip_link_length);
    const double requested_extension =
      height_correction + support_y * roll_correction - hip.x * pitch_correction;
    output.legs[index].extension_offset = std::clamp(
      requested_extension, -extension_limit, extension_limit);
    output.legs[index].limited = output.legs[index].extension_offset != requested_extension;
    command_limited = command_limited || output.legs[index].limited;
  }

  output.stabilized_pose_world = stabilized_pose;
  output.body_twist_command = twist_command;
  output.acceleration_feedforward = acceleration_feedforward;
  output.twist_frame = command.twist_frame;
  output.active_mode = active_mode_;
  output.faults = faults;
  output.evaluated_at = now;
  output.source_sequence = reference.command_frame.sequence;
  output.sequence = ++sequence_;
  output.filtered_lateral_acceleration = filtered_lateral_acceleration_;
  output.turning_roll_compensation = turning_roll;
  output.accepted = true;
  output.command_limited = command_limited;
  output.time_step_limited = time_step_limited;
  return output;
}

void BodyController::Reset() noexcept
{
  linear_integral_.fill(0.0);
  angular_integral_.fill(0.0);
  active_mode_ = ControlMode::kDisabled;
  active_twist_frame_ = ReferenceFrame::kBody;
  filtered_lateral_acceleration_ = 0.0;
  sequence_ = 0U;
  initialized_ = false;
}

ControlMode BodyController::GetActiveMode() const noexcept
{
  return active_mode_;
}

const ControllerParameters & BodyController::GetParameters() const noexcept
{
  return config_.controller;
}

ControllerOutput BodyController::MakeRejectedOutput(
  const FaultFlags faults, const std::chrono::steady_clock::time_point now,
  const std::uint64_t source_sequence) const noexcept
{
  ControllerOutput output;
  output.faults = faults | ToMask(Fault::kControllerFailure);
  output.evaluated_at = now;
  output.source_sequence = source_sequence;
  return output;
}

}  // namespace wheel_dog_mujoco::body
