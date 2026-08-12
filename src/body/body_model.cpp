#include "wheel_dog_mujoco/body/body_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace wheel_dog_mujoco::body
{
namespace
{

constexpr double kSingularityTolerance = 1.0E-9;

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

bool IsLeftLeg(const LegId leg_id) noexcept
{
  return leg_id == LegId::kFrontLeft || leg_id == LegId::kRearLeft;
}

bool IsFinite(const double value) noexcept
{
  return std::isfinite(value);
}

bool IsFinite(const Vector3 & value) noexcept
{
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const LegJointVector & value) noexcept
{
  return IsFinite(value.hip) && IsFinite(value.thigh) && IsFinite(value.calf);
}

bool IsFiniteAndNonnegative(const double value) noexcept
{
  return IsFinite(value) && value >= 0.0;
}

Vector3 Add(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 Subtract(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{left.x - right.x, left.y - right.y, left.z - right.z};
}

Vector3 Cross(const Vector3 & left, const Vector3 & right) noexcept
{
  return Vector3{
    left.y * right.z - left.z * right.y,
    left.z * right.x - left.x * right.z,
    left.x * right.y - left.y * right.x};
}

double Norm(const Vector3 & value) noexcept
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vector3 RotateX(const Vector3 & value, const double angle) noexcept
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return Vector3{
    value.x,
    cosine * value.y - sine * value.z,
    sine * value.y + cosine * value.z};
}

Vector3 RotateY(const Vector3 & value, const double angle) noexcept
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return Vector3{
    cosine * value.x + sine * value.z,
    value.y,
    -sine * value.x + cosine * value.z};
}

void SetJacobianColumn(
  LegJacobian & jacobian, const std::size_t column, const Vector3 & value) noexcept
{
  jacobian[0][column] = value.x;
  jacobian[1][column] = value.y;
  jacobian[2][column] = value.z;
}

Vector3 Multiply(const LegJacobian & matrix, const LegJointVector & vector) noexcept
{
  const std::array<double, 3> values{{vector.hip, vector.thigh, vector.calf}};
  Vector3 result;
  result.x = matrix[0][0] * values[0] + matrix[0][1] * values[1] +
    matrix[0][2] * values[2];
  result.y = matrix[1][0] * values[0] + matrix[1][1] * values[1] +
    matrix[1][2] * values[2];
  result.z = matrix[2][0] * values[0] + matrix[2][1] * values[1] +
    matrix[2][2] * values[2];
  return result;
}

bool SolveLinearSystem(
  const LegJacobian & matrix, const Vector3 & right_hand_side,
  LegJointVector & solution) noexcept
{
  std::array<std::array<double, 4>, 3> augmented{{
    {matrix[0][0], matrix[0][1], matrix[0][2], right_hand_side.x},
    {matrix[1][0], matrix[1][1], matrix[1][2], right_hand_side.y},
    {matrix[2][0], matrix[2][1], matrix[2][2], right_hand_side.z},
  }};

  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 3; ++row) {
      if (std::abs(augmented[row][column]) > std::abs(augmented[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(augmented[pivot][column]) <= kSingularityTolerance) {
      return false;
    }
    if (pivot != column) {
      std::swap(augmented[pivot], augmented[column]);
    }

    const double divisor = augmented[column][column];
    for (std::size_t entry = column; entry < 4; ++entry) {
      augmented[column][entry] /= divisor;
    }
    for (std::size_t row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = augmented[row][column];
      for (std::size_t entry = column; entry < 4; ++entry) {
        augmented[row][entry] -= factor * augmented[column][entry];
      }
    }
  }

  solution.hip = augmented[0][3];
  solution.thigh = augmented[1][3];
  solution.calf = augmented[2][3];
  return IsFinite(solution);
}

double NormalizeAngle(double angle) noexcept
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;
  while (angle > pi) {
    angle -= two_pi;
  }
  while (angle < -pi) {
    angle += two_pi;
  }
  return angle;
}

Vector3 ParseVector3(const YAML::Node & node, const std::string & context)
{
  if (!node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument(context + " must contain exactly three values");
  }
  return Vector3{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

AxisControlGains ParseAxisGains(const YAML::Node & node)
{
  AxisControlGains gains;
  gains.kp = node["kp"].as<double>();
  gains.ki = node["ki"].as<double>();
  gains.kd = node["kd"].as<double>();
  gains.integral_limit = node["integral_limit"].as<double>();
  return gains;
}

SpatialControlGains ParseSpatialGains(const YAML::Node & node)
{
  SpatialControlGains gains;
  gains.linear[0] = ParseAxisGains(node["linear"]["x"]);
  gains.linear[1] = ParseAxisGains(node["linear"]["y"]);
  gains.linear[2] = ParseAxisGains(node["linear"]["z"]);
  gains.angular[0] = ParseAxisGains(node["angular"]["roll"]);
  gains.angular[1] = ParseAxisGains(node["angular"]["pitch"]);
  gains.angular[2] = ParseAxisGains(node["angular"]["yaw"]);
  return gains;
}

bool IsValidGains(const AxisControlGains & gains) noexcept
{
  return IsFiniteAndNonnegative(gains.kp) && IsFiniteAndNonnegative(gains.ki) &&
         IsFiniteAndNonnegative(gains.kd) && IsFiniteAndNonnegative(gains.integral_limit);
}

bool IsNonnegativeVector(const Vector3 & value) noexcept
{
  return IsFinite(value) && value.x >= 0.0 && value.y >= 0.0 && value.z >= 0.0;
}

}  // namespace

BodyModel::BodyModel(const std::string & config_path)
: config_(LoadConfig(config_path))
{
}

const BodyConfig & BodyModel::GetConfig() const noexcept
{
  return config_;
}

const BodyGeometry & BodyModel::GetGeometry() const noexcept
{
  return config_.geometry;
}

Vector3 BodyModel::ForwardKinematics(
  const LegId leg_id, const LegJointVector & joint_positions) const
{
  const std::size_t leg_index = ToIndex(leg_id);
  if (leg_index >= kLegCount || !IsFinite(joint_positions)) {
    throw std::invalid_argument("ForwardKinematics received an invalid leg or joint position");
  }

  const auto & geometry = config_.geometry;
  const double lateral_offset =
    (IsLeftLeg(leg_id) ? 1.0 : -1.0) * geometry.hip_link_length;
  Vector3 chain{0.0, lateral_offset, 0.0};
  chain = Add(
    chain, RotateY(Vector3{0.0, 0.0, -geometry.thigh_link_length},
      joint_positions.thigh));
  chain = Add(
    chain, RotateY(Vector3{0.0, 0.0, -geometry.calf_link_length},
      joint_positions.thigh + joint_positions.calf));
  return Add(
    geometry.hip_positions_body[leg_index], RotateX(chain, joint_positions.hip));
}

LegJacobian BodyModel::ComputeJacobian(
  const LegId leg_id, const LegJointVector & joint_positions) const
{
  const std::size_t leg_index = ToIndex(leg_id);
  if (leg_index >= kLegCount || !IsFinite(joint_positions)) {
    throw std::invalid_argument("ComputeJacobian received an invalid leg or joint position");
  }

  const auto & geometry = config_.geometry;
  const double lateral_offset =
    (IsLeftLeg(leg_id) ? 1.0 : -1.0) * geometry.hip_link_length;
  const Vector3 hip_origin = geometry.hip_positions_body[leg_index];
  const Vector3 thigh_origin = Add(
    hip_origin, RotateX(Vector3{0.0, lateral_offset, 0.0}, joint_positions.hip));
  const Vector3 thigh_link = RotateX(
    RotateY(Vector3{0.0, 0.0, -geometry.thigh_link_length}, joint_positions.thigh),
    joint_positions.hip);
  const Vector3 calf_origin = Add(thigh_origin, thigh_link);
  const Vector3 wheel_center = ForwardKinematics(leg_id, joint_positions);

  const Vector3 hip_axis{1.0, 0.0, 0.0};
  const Vector3 pitch_axis = RotateX(Vector3{0.0, 1.0, 0.0}, joint_positions.hip);

  LegJacobian jacobian{};
  SetJacobianColumn(jacobian, 0, Cross(hip_axis, Subtract(wheel_center, hip_origin)));
  SetJacobianColumn(jacobian, 1, Cross(pitch_axis, Subtract(wheel_center, thigh_origin)));
  SetJacobianColumn(jacobian, 2, Cross(pitch_axis, Subtract(wheel_center, calf_origin)));
  return jacobian;
}

InverseKinematicsResult BodyModel::SolveInverseKinematics(
  const LegId leg_id, const Vector3 & wheel_center_position_body) const
{
  InverseKinematicsResult result;
  const std::size_t leg_index = ToIndex(leg_id);
  if (leg_index >= kLegCount || !IsFinite(wheel_center_position_body)) {
    return result;
  }

  const auto & geometry = config_.geometry;
  const Vector3 relative = Subtract(
    wheel_center_position_body, geometry.hip_positions_body[leg_index]);
  const double lateral_offset =
    (IsLeftLeg(leg_id) ? 1.0 : -1.0) * geometry.hip_link_length;
  const double radial_squared = relative.y * relative.y + relative.z * relative.z;
  const double lateral_squared = lateral_offset * lateral_offset;
  if (radial_squared + kSingularityTolerance < lateral_squared) {
    return result;
  }

  const double sagittal_z = -std::sqrt(std::max(0.0, radial_squared - lateral_squared));
  const double hip_angle = NormalizeAngle(
    std::atan2(relative.z, relative.y) - std::atan2(sagittal_z, lateral_offset));

  const double thigh_length = geometry.thigh_link_length;
  const double calf_length = geometry.calf_link_length;
  const double cosine_calf =
    (relative.x * relative.x + sagittal_z * sagittal_z -
    thigh_length * thigh_length - calf_length * calf_length) /
    (2.0 * thigh_length * calf_length);
  if (cosine_calf < -1.0 - kSingularityTolerance ||
    cosine_calf > 1.0 + kSingularityTolerance)
  {
    return result;
  }

  const double calf_angle = -std::acos(std::clamp(cosine_calf, -1.0, 1.0));
  const double thigh_angle =
    std::atan2(-relative.x, -sagittal_z) -
    std::atan2(
    calf_length * std::sin(calf_angle),
    thigh_length + calf_length * std::cos(calf_angle));

  result.joint_positions = LegJointVector{hip_angle, thigh_angle, calf_angle};
  if (!IsFinite(result.joint_positions)) {
    return result;
  }
  result.position_error = Norm(Subtract(
      ForwardKinematics(leg_id, result.joint_positions), wheel_center_position_body));
  result.reachable = IsFinite(result.position_error) && result.position_error <= 1.0E-6;
  return result;
}

JointVelocityResult BodyModel::SolveJointVelocity(
  const LegId leg_id, const LegJointVector & joint_positions,
  const Vector3 & wheel_center_velocity_body) const
{
  JointVelocityResult result;
  if (ToIndex(leg_id) >= kLegCount || !IsFinite(joint_positions) ||
    !IsFinite(wheel_center_velocity_body))
  {
    return result;
  }
  result.valid = SolveLinearSystem(
    ComputeJacobian(leg_id, joint_positions), wheel_center_velocity_body,
    result.joint_velocities);
  return result;
}

LegKinematicState BodyModel::ComputeLegState(
  const LegId leg_id, const actuator::JointStateFrame & joint_state) const
{
  LegKinematicState result;
  const std::size_t leg_index = ToIndex(leg_id);
  if (leg_index >= kLegCount) {
    return result;
  }

  const auto & ids = kLegJointIds[leg_index];
  const auto & hip = joint_state.joints[actuator::ToIndex(ids[0])];
  const auto & thigh = joint_state.joints[actuator::ToIndex(ids[1])];
  const auto & calf = joint_state.joints[actuator::ToIndex(ids[2])];
  const auto & wheel = joint_state.joints[actuator::ToIndex(kWheelJointIds[leg_index])];
  if (!hip.online || !thigh.online || !calf.online || !wheel.online) {
    return result;
  }

  result.joint_positions = LegJointVector{hip.position, thigh.position, calf.position};
  result.joint_velocities = LegJointVector{hip.velocity, thigh.velocity, calf.velocity};
  result.wheel_position = wheel.position;
  result.wheel_velocity = wheel.velocity;
  if (!IsFinite(result.joint_positions) || !IsFinite(result.joint_velocities) ||
    !IsFinite(result.wheel_position) || !IsFinite(result.wheel_velocity))
  {
    return result;
  }

  result.wheel_center_position_body = ForwardKinematics(leg_id, result.joint_positions);
  result.wheel_center_velocity_body = Multiply(
    ComputeJacobian(leg_id, result.joint_positions), result.joint_velocities);
  result.valid = IsFinite(result.wheel_center_position_body) &&
    IsFinite(result.wheel_center_velocity_body);
  return result;
}

BodyKinematicState BodyModel::ComputeKinematicState(
  const actuator::JointStateFrame & joint_state) const
{
  BodyKinematicState result;
  result.received_at = joint_state.received_at;
  result.sequence = joint_state.sequence;
  result.valid = true;
  for (std::size_t index = 0; index < kLegCount; ++index) {
    result.legs[index] = ComputeLegState(static_cast<LegId>(index), joint_state);
    result.valid = result.valid && result.legs[index].valid;
  }
  return result;
}

BodyConfig BodyModel::LoadConfig(const std::string & config_path)
{
  if (config_path.empty()) {
    throw std::invalid_argument("Body configuration path must not be empty");
  }

  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node body = root["body"];
    if (!body.IsMap()) {
      throw std::invalid_argument("Body configuration requires a body map");
    }

    BodyConfig config;
    const YAML::Node geometry = body["geometry"];
    const YAML::Node hip_positions = geometry["hip_positions_body"];
    config.geometry.hip_positions_body[ToIndex(LegId::kFrontRight)] =
      ParseVector3(hip_positions["front_right"], "front_right hip position");
    config.geometry.hip_positions_body[ToIndex(LegId::kFrontLeft)] =
      ParseVector3(hip_positions["front_left"], "front_left hip position");
    config.geometry.hip_positions_body[ToIndex(LegId::kRearRight)] =
      ParseVector3(hip_positions["rear_right"], "rear_right hip position");
    config.geometry.hip_positions_body[ToIndex(LegId::kRearLeft)] =
      ParseVector3(hip_positions["rear_left"], "rear_left hip position");
    config.geometry.hip_link_length = geometry["hip_link_length"].as<double>();
    config.geometry.thigh_link_length = geometry["thigh_link_length"].as<double>();
    config.geometry.calf_link_length = geometry["calf_link_length"].as<double>();
    config.geometry.wheel_radius = geometry["wheel_radius"].as<double>();

    const YAML::Node limits = body["limits"];
    config.limits.min_ground_clearance = limits["min_ground_clearance"].as<double>();
    config.limits.max_ground_clearance = limits["max_ground_clearance"].as<double>();
    config.limits.max_roll = limits["max_roll"].as<double>();
    config.limits.max_pitch = limits["max_pitch"].as<double>();
    config.limits.max_linear_velocity = ParseVector3(
      limits["max_linear_velocity"], "max_linear_velocity");
    config.limits.max_angular_velocity = ParseVector3(
      limits["max_angular_velocity"], "max_angular_velocity");
    config.limits.max_linear_acceleration = ParseVector3(
      limits["max_linear_acceleration"], "max_linear_acceleration");
    config.limits.max_angular_acceleration = ParseVector3(
      limits["max_angular_acceleration"], "max_angular_acceleration");

    const YAML::Node controller = body["controller"];
    config.controller.gains = ParseSpatialGains(controller["gains"]);
    config.controller.max_time_step = controller["max_time_step"].as<double>();
    config.controller.max_leg_extension_adjustment =
      controller["max_leg_extension_adjustment"].as<double>();
    config.controller.velocity_posture_time_horizon =
      controller["velocity_posture_time_horizon"].as<double>();
    config.controller.turning_roll_gain = controller["turning_roll_gain"].as<double>();
    config.controller.max_turning_roll = controller["max_turning_roll"].as<double>();
    config.controller.lateral_acceleration_filter_coefficient =
      controller["lateral_acceleration_filter_coefficient"].as<double>();

    const YAML::Node trajectory = body["trajectory"];
    config.trajectory.max_time_step = trajectory["max_time_step"].as<double>();
    config.trajectory.position_tolerance =
      trajectory["position_tolerance"].as<double>();
    config.trajectory.orientation_tolerance =
      trajectory["orientation_tolerance"].as<double>();
    config.trajectory.linear_velocity_tolerance =
      trajectory["linear_velocity_tolerance"].as<double>();
    config.trajectory.angular_velocity_tolerance =
      trajectory["angular_velocity_tolerance"].as<double>();

    const YAML::Node coordinator = body["coordinator"];
    config.coordinator.max_time_step = coordinator["max_time_step"].as<double>();
    config.coordinator.max_wheel_speed = coordinator["max_wheel_speed"].as<double>();
    config.coordinator.max_wheel_acceleration =
      coordinator["max_wheel_acceleration"].as<double>();
    config.coordinator.lateral_velocity_tolerance =
      coordinator["lateral_velocity_tolerance"].as<double>();

    const YAML::Node estimator = body["estimator"];
    config.estimator.orientation_filter_coefficient =
      estimator["orientation_filter_coefficient"].as<double>();
    config.estimator.velocity_filter_coefficient =
      estimator["velocity_filter_coefficient"].as<double>();
    config.estimator.contact_force_threshold =
      estimator["contact_force_threshold"].as<double>();
    config.estimator.gravity_magnitude = estimator["gravity_magnitude"].as<double>();
    config.estimator.max_time_step = estimator["max_time_step"].as<double>();
    config.estimator.accelerometer_includes_gravity =
      estimator["accelerometer_includes_gravity"].as<bool>();

    const YAML::Node safety = body["safety"];
    config.safety.command_timeout = safety["command_timeout"].as<double>();
    config.safety.actuator_state_timeout = safety["actuator_state_timeout"].as<double>();
    config.safety.sensor_state_timeout = safety["sensor_state_timeout"].as<double>();

    ValidateConfig(config);
    return config;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to parse body configuration '" + config_path + "': " + error.what());
  }
}

void BodyModel::ValidateConfig(const BodyConfig & config)
{
  const auto & geometry = config.geometry;
  if (!std::all_of(
      geometry.hip_positions_body.begin(), geometry.hip_positions_body.end(),
      [](const Vector3 & position) {return IsFinite(position);}) ||
    !IsFinite(geometry.hip_link_length) || geometry.hip_link_length <= 0.0 ||
    !IsFinite(geometry.thigh_link_length) || geometry.thigh_link_length <= 0.0 ||
    !IsFinite(geometry.calf_link_length) || geometry.calf_link_length <= 0.0 ||
    !IsFinite(geometry.wheel_radius) || geometry.wheel_radius <= 0.0)
  {
    throw std::invalid_argument("Body geometry is invalid");
  }

  const auto & limits = config.limits;
  if (!IsFiniteAndNonnegative(limits.min_ground_clearance) ||
    !IsFinite(limits.max_ground_clearance) ||
    limits.max_ground_clearance <= limits.min_ground_clearance ||
    !IsFinite(limits.max_roll) || limits.max_roll <= 0.0 ||
    !IsFinite(limits.max_pitch) || limits.max_pitch <= 0.0 ||
    !IsNonnegativeVector(limits.max_linear_velocity) ||
    !IsNonnegativeVector(limits.max_angular_velocity) ||
    !IsNonnegativeVector(limits.max_linear_acceleration) ||
    !IsNonnegativeVector(limits.max_angular_acceleration))
  {
    throw std::invalid_argument("Body limits are invalid");
  }

  for (const auto & gains : config.controller.gains.linear) {
    if (!IsValidGains(gains)) {
      throw std::invalid_argument("Body linear control gains are invalid");
    }
  }
  for (const auto & gains : config.controller.gains.angular) {
    if (!IsValidGains(gains)) {
      throw std::invalid_argument("Body angular control gains are invalid");
    }
  }
  if (!IsFinite(config.controller.max_time_step) || config.controller.max_time_step <= 0.0 ||
    !IsFinite(config.controller.max_leg_extension_adjustment) ||
    config.controller.max_leg_extension_adjustment <= 0.0 ||
    !IsFinite(config.controller.velocity_posture_time_horizon) ||
    config.controller.velocity_posture_time_horizon <= 0.0 ||
    !IsFiniteAndNonnegative(config.controller.turning_roll_gain) ||
    !IsFiniteAndNonnegative(config.controller.max_turning_roll) ||
    !IsFinite(config.controller.lateral_acceleration_filter_coefficient) ||
    config.controller.lateral_acceleration_filter_coefficient < 0.0 ||
    config.controller.lateral_acceleration_filter_coefficient > 1.0)
  {
    throw std::invalid_argument("Body controller parameters are invalid");
  }

  if (!IsFinite(config.trajectory.max_time_step) ||
    config.trajectory.max_time_step <= 0.0 ||
    !IsFiniteAndNonnegative(config.trajectory.position_tolerance) ||
    !IsFiniteAndNonnegative(config.trajectory.orientation_tolerance) ||
    !IsFiniteAndNonnegative(config.trajectory.linear_velocity_tolerance) ||
    !IsFiniteAndNonnegative(config.trajectory.angular_velocity_tolerance))
  {
    throw std::invalid_argument("Body trajectory parameters are invalid");
  }

  if (!IsFinite(config.coordinator.max_time_step) ||
    config.coordinator.max_time_step <= 0.0 ||
    !IsFinite(config.coordinator.max_wheel_speed) ||
    config.coordinator.max_wheel_speed <= 0.0 ||
    !IsFinite(config.coordinator.max_wheel_acceleration) ||
    config.coordinator.max_wheel_acceleration <= 0.0 ||
    !IsFiniteAndNonnegative(config.coordinator.lateral_velocity_tolerance))
  {
    throw std::invalid_argument("Wheel-leg coordinator parameters are invalid");
  }

  if (!IsFinite(config.estimator.orientation_filter_coefficient) ||
    config.estimator.orientation_filter_coefficient < 0.0 ||
    config.estimator.orientation_filter_coefficient > 1.0 ||
    !IsFinite(config.estimator.velocity_filter_coefficient) ||
    config.estimator.velocity_filter_coefficient < 0.0 ||
    config.estimator.velocity_filter_coefficient > 1.0 ||
    !IsFiniteAndNonnegative(config.estimator.contact_force_threshold) ||
    !IsFinite(config.estimator.gravity_magnitude) ||
    config.estimator.gravity_magnitude <= 0.0 ||
    !IsFinite(config.estimator.max_time_step) || config.estimator.max_time_step <= 0.0)
  {
    throw std::invalid_argument("Body estimator parameters are invalid");
  }

  if (!IsFinite(config.safety.command_timeout) || config.safety.command_timeout <= 0.0 ||
    !IsFinite(config.safety.actuator_state_timeout) ||
    config.safety.actuator_state_timeout <= 0.0 ||
    !IsFinite(config.safety.sensor_state_timeout) || config.safety.sensor_state_timeout <= 0.0)
  {
    throw std::invalid_argument("Body safety timing is invalid");
  }
}

}  // namespace wheel_dog_mujoco::body
