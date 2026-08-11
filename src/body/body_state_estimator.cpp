#include "wheel_dog_mujoco/body/body_state_estimator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_dog_mujoco::body
{
namespace
{

constexpr double kQuaternionNormTolerance = 1.0E-12;

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

bool IsFinite(const LegJointVector & value) noexcept
{
  return IsFinite(value.hip) && IsFinite(value.thigh) && IsFinite(value.calf);
}

bool IsValid(const LegKinematicState & state) noexcept
{
  return state.valid && IsFinite(state.joint_positions) && IsFinite(state.joint_velocities) &&
         IsFinite(state.wheel_center_position_body) &&
         IsFinite(state.wheel_center_velocity_body) && IsFinite(state.wheel_position) &&
         IsFinite(state.wheel_velocity);
}

Vector3 Add(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 Subtract(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{left.x - right.x, left.y - right.y, left.z - right.z};
}

Vector3 Scale(const Vector3 & value, const double scale) noexcept
{
  return Vector3{scale * value.x, scale * value.y, scale * value.z};
}

Vector3 Cross(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{
    left.y * right.z - left.z * right.y,
    left.z * right.x - left.x * right.z,
    left.x * right.y - left.y * right.x};
}

Vector3 Blend(
  const Vector3 & previous, const Vector3 & measurement, const double previous_weight) noexcept
{
  return Add(Scale(previous, previous_weight), Scale(measurement, 1.0 - previous_weight));
}

double QuaternionDot(const Quaternion & left, const Quaternion & right) noexcept
{
  return left.w * right.w + left.x * right.x + left.y * right.y + left.z * right.z;
}

Quaternion Normalize(const Quaternion & quaternion)
{
  if (!IsFinite(quaternion)) {
    throw std::invalid_argument("Cannot normalize a non-finite quaternion");
  }
  const double norm = std::sqrt(QuaternionDot(quaternion, quaternion));
  if (norm <= kQuaternionNormTolerance) {
    throw std::invalid_argument("Cannot normalize a zero quaternion");
  }
  return Quaternion{
    quaternion.w / norm, quaternion.x / norm,
    quaternion.y / norm, quaternion.z / norm};
}

Quaternion BlendOrientation(
  const Quaternion & previous, Quaternion measurement, const double previous_weight)
{
  if (QuaternionDot(previous, measurement) < 0.0) {
    measurement.w = -measurement.w;
    measurement.x = -measurement.x;
    measurement.y = -measurement.y;
    measurement.z = -measurement.z;
  }
  return Normalize(Quaternion{
    previous_weight * previous.w + (1.0 - previous_weight) * measurement.w,
    previous_weight * previous.x + (1.0 - previous_weight) * measurement.x,
    previous_weight * previous.y + (1.0 - previous_weight) * measurement.y,
    previous_weight * previous.z + (1.0 - previous_weight) * measurement.z});
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

bool IsImuValid(const ImuState & imu) noexcept
{
  return imu.orientation_valid && imu.angular_velocity_valid &&
         imu.linear_acceleration_valid && IsFinite(imu.orientation_world_from_body) &&
         IsFinite(imu.angular_velocity_body) && IsFinite(imu.linear_acceleration_body) &&
         IsFinite(imu.temperature) &&
         QuaternionDot(imu.orientation_world_from_body, imu.orientation_world_from_body) >
         kQuaternionNormTolerance;
}

double RollFromQuaternion(const Quaternion & q) noexcept
{
  return std::atan2(
    2.0 * (q.w * q.x + q.y * q.z),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

double PitchFromQuaternion(const Quaternion & q) noexcept
{
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

bool ExceedsMagnitude(const Vector3 & value, const Vector3 & limits) noexcept
{
  return std::abs(value.x) > limits.x || std::abs(value.y) > limits.y ||
         std::abs(value.z) > limits.z;
}

bool IsTimingValid(const double elapsed_seconds, const double age) noexcept
{
  return IsFinite(elapsed_seconds) && elapsed_seconds > 0.0 && IsFinite(age) && age >= 0.0;
}

}  // namespace

BodyStateEstimator::BodyStateEstimator(const BodyConfig & config)
: config_(config)
{
  const auto & parameters = config_.estimator;
  if (!IsFinite(parameters.orientation_filter_coefficient) ||
    parameters.orientation_filter_coefficient < 0.0 ||
    parameters.orientation_filter_coefficient > 1.0 ||
    !IsFinite(parameters.velocity_filter_coefficient) ||
    parameters.velocity_filter_coefficient < 0.0 ||
    parameters.velocity_filter_coefficient > 1.0 ||
    !IsFinite(parameters.contact_force_threshold) ||
    parameters.contact_force_threshold < 0.0 ||
    !IsFinite(parameters.gravity_magnitude) || parameters.gravity_magnitude <= 0.0 ||
    !IsFinite(parameters.max_time_step) || parameters.max_time_step <= 0.0 ||
    !IsFinite(config_.safety.actuator_state_timeout) ||
    config_.safety.actuator_state_timeout <= 0.0 ||
    !IsFinite(config_.safety.sensor_state_timeout) || config_.safety.sensor_state_timeout <= 0.0 ||
    !IsFinite(config_.geometry.wheel_radius) || config_.geometry.wheel_radius <= 0.0)
  {
    throw std::invalid_argument("BodyStateEstimator configuration is invalid");
  }
}

EstimatorOutput BodyStateEstimator::Update(
  const BodyKinematicState & kinematic_state,
  const BodySensorFrame & sensor_frame,
  const std::chrono::steady_clock::time_point now,
  const double elapsed_seconds)
{
  FaultFlags faults = ToMask(Fault::kNone);
  const double actuator_age =
    std::chrono::duration<double>(now - kinematic_state.received_at).count();
  const double sensor_age =
    std::chrono::duration<double>(now - sensor_frame.received_at).count();
  if (!IsTimingValid(elapsed_seconds, actuator_age) ||
    !IsTimingValid(elapsed_seconds, sensor_age))
  {
    return MakeInvalidOutput(ToMask(Fault::kStateEstimateInvalid), now);
  }
  if (actuator_age > config_.safety.actuator_state_timeout) {
    faults |= ToMask(Fault::kActuatorStateTimeout);
  }
  if (sensor_age > config_.safety.sensor_state_timeout) {
    faults |= ToMask(Fault::kSensorTimeout);
  }
  if (!kinematic_state.valid ||
    !std::all_of(
      kinematic_state.legs.begin(), kinematic_state.legs.end(),
      [](const LegKinematicState & leg) {return IsValid(leg);}))
  {
    faults |= ToMask(Fault::kActuatorStateInvalid);
  }
  if (!IsImuValid(sensor_frame.imu)) {
    faults |= ToMask(Fault::kSensorStateInvalid);
  }
  constexpr FaultFlags invalid_input_faults =
    ToMask(Fault::kActuatorStateTimeout) | ToMask(Fault::kSensorTimeout) |
    ToMask(Fault::kActuatorStateInvalid) | ToMask(Fault::kSensorStateInvalid);
  if ((faults & invalid_input_faults) != ToMask(Fault::kNone)) {
    return MakeInvalidOutput(faults, now);
  }

  const double bounded_dt = std::min(elapsed_seconds, config_.estimator.max_time_step);
  const Quaternion measured_orientation = Normalize(sensor_frame.imu.orientation_world_from_body);
  const Quaternion orientation = initialized_ ?
    BlendOrientation(
    state_.pose_world.orientation, measured_orientation,
    config_.estimator.orientation_filter_coefficient) : measured_orientation;
  const Vector3 angular_velocity = initialized_ ?
    Blend(
    state_.twist_body.angular, sensor_frame.imu.angular_velocity_body,
    config_.estimator.velocity_filter_coefficient) :
    sensor_frame.imu.angular_velocity_body;

  std::array<ContactState, kLegCount> contacts{};
  std::size_t valid_contact_count = 0U;
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    const auto & measured_contact = sensor_frame.contacts[index];
    auto & contact = contacts[index];
    contact.normal_force = measured_contact.normal_force;
    contact.valid = measured_contact.valid && IsFinite(measured_contact.normal_force);
    contact.in_contact = contact.valid &&
      (measured_contact.in_contact ||
      measured_contact.normal_force >= config_.estimator.contact_force_threshold);
    if (contact.valid) {
      ++valid_contact_count;
    }
  }

  // unitree_mujoco does not publish wheel contact force. When every contact
  // channel is explicitly unavailable, assume all wheels support the body and
  // use their rolling constraints. A valid sensor reporting no contact still
  // means real contact loss and must not take this fallback.
  const bool use_kinematic_support_assumption = valid_contact_count == 0U;
  Vector3 measured_linear_velocity{};
  double ground_clearance_sum = 0.0;
  std::size_t support_count = 0U;
  for (std::size_t index = 0U; index < kLegCount; ++index) {
    if (!contacts[index].in_contact && !use_kinematic_support_assumption) {
      continue;
    }

    const auto & leg = kinematic_state.legs[index];
    const Vector3 contact_position_body = Add(
      leg.wheel_center_position_body,
      Vector3{0.0, 0.0, -config_.geometry.wheel_radius});
    const Vector3 rolling_velocity_body{
      config_.geometry.wheel_radius * leg.wheel_velocity, 0.0, 0.0};
    const Vector3 base_velocity = Subtract(
      Subtract(rolling_velocity_body, leg.wheel_center_velocity_body),
      Cross(angular_velocity, contact_position_body));
    measured_linear_velocity = Add(measured_linear_velocity, base_velocity);
    ground_clearance_sum -= RotateBodyToWorld(orientation, contact_position_body).z;
    ++support_count;
  }

  double ground_clearance = state_.ground_clearance;
  if (support_count > 0U) {
    const double divisor = static_cast<double>(support_count);
    measured_linear_velocity = Scale(measured_linear_velocity, 1.0 / divisor);
    ground_clearance = ground_clearance_sum / divisor;
  } else {
    faults |= ToMask(Fault::kContactLost);
    if (!initialized_) {
      for (const auto & leg : kinematic_state.legs) {
        const Vector3 contact_position_body = Add(
          leg.wheel_center_position_body,
          Vector3{0.0, 0.0, -config_.geometry.wheel_radius});
        ground_clearance -= RotateBodyToWorld(orientation, contact_position_body).z;
      }
      ground_clearance /= static_cast<double>(kLegCount);
    } else {
      measured_linear_velocity = state_.twist_body.linear;
    }
  }

  const Vector3 linear_velocity = initialized_ ?
    Blend(
    state_.twist_body.linear, measured_linear_velocity,
    config_.estimator.velocity_filter_coefficient) : measured_linear_velocity;
  Vector3 angular_acceleration{};
  if (initialized_) {
    angular_acceleration = Scale(
      Subtract(angular_velocity, state_.twist_body.angular), 1.0 / bounded_dt);
  }

  Vector3 linear_acceleration = sensor_frame.imu.linear_acceleration_body;
  if (config_.estimator.accelerometer_includes_gravity) {
    const Vector3 gravity_world{0.0, 0.0, -config_.estimator.gravity_magnitude};
    linear_acceleration = Add(
      linear_acceleration, RotateWorldToBody(orientation, gravity_world));
  }

  Vector3 world_position = state_.pose_world.position;
  const Vector3 world_velocity = RotateBodyToWorld(orientation, linear_velocity);
  if (initialized_) {
    world_position.x += world_velocity.x * bounded_dt;
    world_position.y += world_velocity.y * bounded_dt;
  }
  world_position.z = ground_clearance;

  state_.pose_world.position = world_position;
  state_.pose_world.orientation = orientation;
  state_.twist_body.linear = linear_velocity;
  state_.twist_body.angular = angular_velocity;
  state_.acceleration_body.linear = linear_acceleration;
  state_.acceleration_body.angular = angular_acceleration;
  state_.ground_clearance = ground_clearance;
  state_.contacts = contacts;
  state_.valid = true;
  initialized_ = true;

  if (std::abs(RollFromQuaternion(orientation)) > config_.limits.max_roll ||
    std::abs(PitchFromQuaternion(orientation)) > config_.limits.max_pitch ||
    ground_clearance < config_.limits.min_ground_clearance ||
    ground_clearance > config_.limits.max_ground_clearance)
  {
    faults |= ToMask(Fault::kBodyPoseLimit);
  }
  if (ExceedsMagnitude(linear_velocity, config_.limits.max_linear_velocity) ||
    ExceedsMagnitude(angular_velocity, config_.limits.max_angular_velocity))
  {
    faults |= ToMask(Fault::kBodyVelocityLimit);
  }
  if (ExceedsMagnitude(linear_acceleration, config_.limits.max_linear_acceleration) ||
    ExceedsMagnitude(angular_acceleration, config_.limits.max_angular_acceleration))
  {
    faults |= ToMask(Fault::kBodyAccelerationLimit);
  }

  EstimatorOutput output;
  output.state = state_;
  output.faults = faults;
  output.estimated_at = now;
  output.sequence = ++sequence_;
  output.time_step_limited = bounded_dt < elapsed_seconds;
  return output;
}

void BodyStateEstimator::Reset() noexcept
{
  state_ = BodyState{};
  sequence_ = 0U;
  initialized_ = false;
}

const BodyState & BodyStateEstimator::GetState() const noexcept
{
  return state_;
}

const EstimatorParameters & BodyStateEstimator::GetParameters() const noexcept
{
  return config_.estimator;
}

EstimatorOutput BodyStateEstimator::MakeInvalidOutput(
  const FaultFlags faults, const std::chrono::steady_clock::time_point now) noexcept
{
  EstimatorOutput output;
  output.state = state_;
  output.state.valid = false;
  output.faults = faults | ToMask(Fault::kStateEstimateInvalid);
  output.estimated_at = now;
  output.sequence = ++sequence_;
  return output;
}

}  // namespace wheel_dog_mujoco::body
