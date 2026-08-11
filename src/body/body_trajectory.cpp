#include "wheel_dog_mujoco/body/body_trajectory.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::body
{
namespace
{

constexpr double kQuaternionNormTolerance = 1.0E-12;
constexpr double kPi = 3.14159265358979323846;

struct ScalarStep
{
  double value{0.0};
  double rate{0.0};
  double acceleration{0.0};
  bool reached{false};
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

bool IsNonnegative(const Vector3 & value) noexcept
{
  return IsFinite(value) && value.x >= 0.0 && value.y >= 0.0 && value.z >= 0.0;
}

bool IsControlModeValid(const ControlMode mode) noexcept
{
  switch (mode) {
    case ControlMode::kDisabled:
    case ControlMode::kPose:
    case ControlMode::kVelocity:
    case ControlMode::kHybrid:
      return true;
  }
  return false;
}

bool IsReferenceFrameValid(const ReferenceFrame frame) noexcept
{
  return frame == ReferenceFrame::kBody || frame == ReferenceFrame::kWorld;
}

bool UsesPose(const ControlMode mode) noexcept
{
  return mode == ControlMode::kPose || mode == ControlMode::kHybrid;
}

bool IsCommandValid(const BodyCommand & command) noexcept
{
  if (!IsControlModeValid(command.control_mode) ||
    !IsReferenceFrameValid(command.pose_frame) ||
    !IsReferenceFrameValid(command.twist_frame) || !IsFinite(command.desired_pose) ||
    !IsFinite(command.desired_twist) || !IsFinite(command.acceleration_feedforward))
  {
    return false;
  }
  const double orientation_norm_squared =
    command.desired_pose.orientation.w * command.desired_pose.orientation.w +
    command.desired_pose.orientation.x * command.desired_pose.orientation.x +
    command.desired_pose.orientation.y * command.desired_pose.orientation.y +
    command.desired_pose.orientation.z * command.desired_pose.orientation.z;
  return (!UsesPose(command.control_mode) || command.pose_frame == ReferenceFrame::kWorld) &&
         orientation_norm_squared > kQuaternionNormTolerance;
}

bool IsStateValid(const BodyState & state) noexcept
{
  const double orientation_norm_squared =
    state.pose_world.orientation.w * state.pose_world.orientation.w +
    state.pose_world.orientation.x * state.pose_world.orientation.x +
    state.pose_world.orientation.y * state.pose_world.orientation.y +
    state.pose_world.orientation.z * state.pose_world.orientation.z;
  return state.valid && IsFinite(state.pose_world) && IsFinite(state.twist_body) &&
         IsFinite(state.acceleration_body) && IsFinite(state.ground_clearance) &&
         orientation_norm_squared > kQuaternionNormTolerance;
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

double Approach(const double current, const double target, const double max_change) noexcept
{
  return current + std::clamp(target - current, -max_change, max_change);
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

Vector3 QuaternionToRpy(const Quaternion & quaternion) noexcept
{
  const Quaternion & q = quaternion;
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
  const double half_roll = 0.5 * rpy.x;
  const double half_pitch = 0.5 * rpy.y;
  const double half_yaw = 0.5 * rpy.z;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
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

ScalarStep StepVelocity(
  const double current, const double target, const double max_velocity,
  const double max_acceleration, const double time_step, const double tolerance) noexcept
{
  ScalarStep result;
  const double bounded_target = std::clamp(target, -max_velocity, max_velocity);
  result.value = Approach(current, bounded_target, max_acceleration * time_step);
  result.rate = result.value;
  result.acceleration = (result.value - current) / time_step;
  result.reached = std::abs(result.value - target) <= tolerance;
  result.limited = bounded_target != target || !result.reached;
  return result;
}

ScalarStep StepPosition(
  const double current_position, const double current_velocity, const double target_position,
  const double max_velocity, const double max_acceleration, const double time_step,
  const double position_tolerance, const double velocity_tolerance) noexcept
{
  ScalarStep result;
  const double error = target_position - current_position;
  if (std::abs(error) <= position_tolerance &&
    std::abs(current_velocity) <= max_acceleration * time_step)
  {
    result.value = target_position;
    result.rate = 0.0;
    result.acceleration = -current_velocity / time_step;
    result.reached = true;
    result.limited = false;
    return result;
  }

  const double stopping_velocity = std::sqrt(2.0 * max_acceleration * std::abs(error));
  const double desired_velocity = std::copysign(
    std::min(max_velocity, stopping_velocity), error);
  result.rate = Approach(
    current_velocity, desired_velocity, max_acceleration * time_step);
  result.value = current_position + result.rate * time_step;
  result.acceleration = (result.rate - current_velocity) / time_step;
  result.reached = std::abs(target_position - result.value) <= position_tolerance &&
    std::abs(result.rate) <= velocity_tolerance;
  result.limited = !result.reached;
  return result;
}

bool IsZero(const SpatialAcceleration & value) noexcept
{
  return value.linear.x == 0.0 && value.linear.y == 0.0 && value.linear.z == 0.0 &&
         value.angular.x == 0.0 && value.angular.y == 0.0 && value.angular.z == 0.0;
}

}  // namespace

BodyTrajectory::BodyTrajectory(const BodyConfig & config)
: config_(config)
{
  const auto & parameters = config_.trajectory;
  if (!IsFinite(parameters.max_time_step) || parameters.max_time_step <= 0.0 ||
    !IsFinite(parameters.position_tolerance) || parameters.position_tolerance < 0.0 ||
    !IsFinite(parameters.orientation_tolerance) || parameters.orientation_tolerance < 0.0 ||
    !IsFinite(parameters.linear_velocity_tolerance) ||
    parameters.linear_velocity_tolerance < 0.0 ||
    !IsFinite(parameters.angular_velocity_tolerance) ||
    parameters.angular_velocity_tolerance < 0.0 ||
    !IsNonnegative(config_.limits.max_linear_velocity) ||
    !IsNonnegative(config_.limits.max_angular_velocity) ||
    !IsNonnegative(config_.limits.max_linear_acceleration) ||
    !IsNonnegative(config_.limits.max_angular_acceleration))
  {
    throw std::invalid_argument("BodyTrajectory configuration is invalid");
  }
}

TrajectoryOutput BodyTrajectory::Update(
  const BodyCommandFrame & target,
  const BodyState & current_state,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  if (!IsControlModeValid(target.command.control_mode)) {
    return MakeRejectedOutput(ToMask(Fault::kInvalidCommand), target.sequence);
  }

  if (target.command.control_mode == ControlMode::kDisabled) {
    command_frame_.command.control_mode = ControlMode::kDisabled;
    command_frame_.command.desired_twist = Twist{};
    command_frame_.command.acceleration_feedforward = SpatialAcceleration{};
    command_frame_.created_at = now;
    command_frame_.sequence = ++output_sequence_;
    pose_position_rate_world_ = Vector3{};
    pose_rpy_rate_ = Vector3{};
    initialized_ = false;

    TrajectoryOutput output;
    output.command_frame = command_frame_;
    output.source_sequence = target.sequence;
    output.accepted = true;
    output.target_reached = true;
    return output;
  }

  if (!IsCommandValid(target.command) || !IsFinite(elapsed_seconds) || elapsed_seconds <= 0.0) {
    return MakeRejectedOutput(ToMask(Fault::kInvalidCommand), target.sequence);
  }

  const bool mode_or_frame_changed =
    !initialized_ || command_frame_.command.control_mode != target.command.control_mode ||
    command_frame_.command.pose_frame != target.command.pose_frame ||
    command_frame_.command.twist_frame != target.command.twist_frame;
  if (mode_or_frame_changed) {
    if (!IsStateValid(current_state)) {
      return MakeRejectedOutput(ToMask(Fault::kStateEstimateInvalid), target.sequence);
    }
    InitializeFromState(target, current_state);
  }

  const double time_step = std::min(elapsed_seconds, config_.trajectory.max_time_step);
  const bool time_step_limited = time_step < elapsed_seconds;
  bool target_reached = true;
  bool command_limited = time_step_limited || !IsZero(target.command.acceleration_feedforward);
  Vector3 pose_position_acceleration{};
  Vector3 pose_rpy_acceleration{};
  SpatialAcceleration velocity_acceleration{};

  command_frame_.command.control_mode = target.command.control_mode;
  command_frame_.command.pose_frame = target.command.pose_frame;
  command_frame_.command.twist_frame = target.command.twist_frame;

  const Quaternion target_orientation = Normalize(target.command.desired_pose.orientation);
  const Vector3 target_rpy = QuaternionToRpy(target_orientation);

  const auto update_position_axis = [&](const std::size_t index) {
      const ScalarStep step = StepPosition(
        Component(command_frame_.command.desired_pose.position, index),
        Component(pose_position_rate_world_, index),
        Component(target.command.desired_pose.position, index),
        Component(config_.limits.max_linear_velocity, index),
        Component(config_.limits.max_linear_acceleration, index), time_step,
        config_.trajectory.position_tolerance,
        config_.trajectory.linear_velocity_tolerance);
      SetComponent(command_frame_.command.desired_pose.position, index, step.value);
      SetComponent(pose_position_rate_world_, index, step.rate);
      SetComponent(pose_position_acceleration, index, step.acceleration);
      target_reached = target_reached && step.reached;
      command_limited = command_limited || step.limited;
    };

  const auto update_orientation_axis = [&](const std::size_t index) {
      const double angle_target = Component(pose_rpy_, index) + NormalizeAngle(
        Component(target_rpy, index) - Component(pose_rpy_, index));
      const ScalarStep step = StepPosition(
        Component(pose_rpy_, index), Component(pose_rpy_rate_, index), angle_target,
        Component(config_.limits.max_angular_velocity, index),
        Component(config_.limits.max_angular_acceleration, index), time_step,
        config_.trajectory.orientation_tolerance,
        config_.trajectory.angular_velocity_tolerance);
      SetComponent(pose_rpy_, index, step.value);
      SetComponent(pose_rpy_rate_, index, step.rate);
      SetComponent(pose_rpy_acceleration, index, step.acceleration);
      target_reached = target_reached && step.reached;
      command_limited = command_limited || step.limited;
    };

  const auto update_linear_velocity_axis = [&](const std::size_t index) {
      const ScalarStep step = StepVelocity(
        Component(command_frame_.command.desired_twist.linear, index),
        Component(target.command.desired_twist.linear, index),
        Component(config_.limits.max_linear_velocity, index),
        Component(config_.limits.max_linear_acceleration, index), time_step,
        config_.trajectory.linear_velocity_tolerance);
      SetComponent(command_frame_.command.desired_twist.linear, index, step.value);
      SetComponent(velocity_acceleration.linear, index, step.acceleration);
      target_reached = target_reached && step.reached;
      command_limited = command_limited || step.limited;
    };

  const auto update_angular_velocity_axis = [&](const std::size_t index) {
      const ScalarStep step = StepVelocity(
        Component(command_frame_.command.desired_twist.angular, index),
        Component(target.command.desired_twist.angular, index),
        Component(config_.limits.max_angular_velocity, index),
        Component(config_.limits.max_angular_acceleration, index), time_step,
        config_.trajectory.angular_velocity_tolerance);
      SetComponent(command_frame_.command.desired_twist.angular, index, step.value);
      SetComponent(velocity_acceleration.angular, index, step.acceleration);
      target_reached = target_reached && step.reached;
      command_limited = command_limited || step.limited;
    };

  switch (target.command.control_mode) {
    case ControlMode::kDisabled:
      break;
    case ControlMode::kPose:
      for (std::size_t index = 0; index < 3U; ++index) {
        update_position_axis(index);
        update_orientation_axis(index);
      }
      command_frame_.command.desired_twist = Twist{};
      command_frame_.command.acceleration_feedforward = SpatialAcceleration{};
      break;
    case ControlMode::kVelocity:
      pose_position_rate_world_ = Vector3{};
      pose_rpy_rate_ = Vector3{};
      for (std::size_t index = 0; index < 3U; ++index) {
        update_linear_velocity_axis(index);
        update_angular_velocity_axis(index);
      }
      command_frame_.command.acceleration_feedforward = velocity_acceleration;
      break;
    case ControlMode::kHybrid:
      pose_position_rate_world_.x = 0.0;
      pose_position_rate_world_.y = 0.0;
      pose_rpy_rate_.z = 0.0;
      update_position_axis(2U);
      update_orientation_axis(0U);
      update_orientation_axis(1U);
      command_frame_.command.desired_twist.linear.z = 0.0;
      command_frame_.command.desired_twist.angular.x = 0.0;
      command_frame_.command.desired_twist.angular.y = 0.0;
      update_linear_velocity_axis(0U);
      update_linear_velocity_axis(1U);
      update_angular_velocity_axis(2U);
      command_frame_.command.acceleration_feedforward = velocity_acceleration;
      break;
  }

  command_frame_.command.desired_pose.orientation = RpyToQuaternion(pose_rpy_);
  command_frame_.created_at = now;
  command_frame_.sequence = ++output_sequence_;

  TrajectoryOutput output;
  output.command_frame = command_frame_;
  output.pose_position_rate_world = pose_position_rate_world_;
  output.pose_position_acceleration_world = pose_position_acceleration;
  output.pose_rpy_rate = pose_rpy_rate_;
  output.pose_rpy_acceleration = pose_rpy_acceleration;
  output.source_sequence = target.sequence;
  output.accepted = true;
  output.target_reached = target_reached;
  output.command_limited = command_limited;
  output.time_step_limited = time_step_limited;
  return output;
}

void BodyTrajectory::Reset() noexcept
{
  command_frame_ = BodyCommandFrame{};
  pose_position_rate_world_ = Vector3{};
  pose_rpy_ = Vector3{};
  pose_rpy_rate_ = Vector3{};
  output_sequence_ = 0U;
  initialized_ = false;
}

const BodyCommandFrame & BodyTrajectory::GetCommandFrame() const noexcept
{
  return command_frame_;
}

const TrajectoryParameters & BodyTrajectory::GetParameters() const noexcept
{
  return config_.trajectory;
}

void BodyTrajectory::InitializeFromState(
  const BodyCommandFrame & target, const BodyState & current_state)
{
  const Quaternion orientation = Normalize(current_state.pose_world.orientation);
  pose_rpy_ = QuaternionToRpy(orientation);
  pose_position_rate_world_ = RotateBodyToWorld(
    orientation, current_state.twist_body.linear);
  pose_rpy_rate_ = BodyAngularVelocityToRpyRate(
    current_state.twist_body.angular, pose_rpy_);

  command_frame_.command = target.command;
  command_frame_.command.desired_pose = current_state.pose_world;
  command_frame_.command.desired_pose.orientation = orientation;
  command_frame_.command.desired_twist = current_state.twist_body;
  if (target.command.twist_frame == ReferenceFrame::kWorld) {
    command_frame_.command.desired_twist.linear = RotateBodyToWorld(
      orientation, current_state.twist_body.linear);
    command_frame_.command.desired_twist.angular = RotateBodyToWorld(
      orientation, current_state.twist_body.angular);
  }
  command_frame_.command.acceleration_feedforward = SpatialAcceleration{};
  initialized_ = true;
}

TrajectoryOutput BodyTrajectory::MakeRejectedOutput(
  const FaultFlags faults, const std::uint64_t source_sequence) const noexcept
{
  TrajectoryOutput output;
  output.command_frame = command_frame_;
  output.pose_position_rate_world = pose_position_rate_world_;
  output.pose_rpy_rate = pose_rpy_rate_;
  output.faults = faults;
  output.source_sequence = source_sequence;
  return output;
}

}  // namespace wheel_dog_mujoco::body
