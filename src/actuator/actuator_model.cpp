#include "wheel_dog_mujoco/actuator/actuator_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace wheel_dog_mujoco::actuator
{
namespace
{

struct ActuatorProfile
{
  ActuatorKind actuator_kind{ActuatorKind::kLegJoint};
  double gear_ratio{1.0};
  ActuatorLimits limits{};
  GainProfileSet gains{};
  ControllerParameters controller{};
  SafetyParameters safety{};
};

constexpr std::array<std::pair<const char *, JointId>, kActuatorCount> kJointNames{{
  {"front_right_hip", JointId::kFrontRightHip},
  {"front_right_thigh", JointId::kFrontRightThigh},
  {"front_right_calf", JointId::kFrontRightCalf},
  {"front_left_hip", JointId::kFrontLeftHip},
  {"front_left_thigh", JointId::kFrontLeftThigh},
  {"front_left_calf", JointId::kFrontLeftCalf},
  {"rear_right_hip", JointId::kRearRightHip},
  {"rear_right_thigh", JointId::kRearRightThigh},
  {"rear_right_calf", JointId::kRearRightCalf},
  {"rear_left_hip", JointId::kRearLeftHip},
  {"rear_left_thigh", JointId::kRearLeftThigh},
  {"rear_left_calf", JointId::kRearLeftCalf},
  {"front_right_wheel", JointId::kFrontRightWheel},
  {"front_left_wheel", JointId::kFrontLeftWheel},
  {"rear_right_wheel", JointId::kRearRightWheel},
  {"rear_left_wheel", JointId::kRearLeftWheel},
}};

JointId ParseJointId(const std::string & name)
{
  const auto found = std::find_if(
    kJointNames.begin(), kJointNames.end(),
    [&name](const auto & entry) {return name == entry.first;});
  if (found == kJointNames.end()) {
    throw std::invalid_argument("Unknown joint_id: " + name);
  }
  return found->second;
}

bool IsWheel(const JointId joint_id) noexcept
{
  return ToIndex(joint_id) >= ToIndex(JointId::kFrontRightWheel);
}

ActuatorKind ParseActuatorKind(const std::string & value)
{
  if (value == "leg_joint") {
    return ActuatorKind::kLegJoint;
  }
  if (value == "wheel") {
    return ActuatorKind::kWheel;
  }
  throw std::invalid_argument("Unknown actuator kind: " + value);
}

ControlGains ParseGains(const YAML::Node & node)
{
  ControlGains gains;
  gains.kp = node["kp"].as<double>();
  gains.ki = node["ki"].as<double>();
  gains.kd = node["kd"].as<double>();
  gains.integral_limit = node["integral_limit"].as<double>();
  return gains;
}

YAML::Node ReadLimitValue(const YAML::Node & node, const char * key)
{
  if (node[key]) {
    return node[key];
  }
  const YAML::Node inherited = node["<<"];
  if (inherited && inherited.IsMap() && inherited[key]) {
    return inherited[key];
  }
  throw std::invalid_argument(std::string("Missing actuator limit: ") + key);
}

ActuatorLimits ParseLimits(const YAML::Node & node)
{
  ActuatorLimits limits;
  limits.position_limited = ReadLimitValue(node, "position_limited").as<bool>();
  limits.min_position = ReadLimitValue(node, "min_position").as<double>();
  limits.max_position = ReadLimitValue(node, "max_position").as<double>();
  limits.max_velocity = ReadLimitValue(node, "max_velocity").as<double>();
  limits.max_acceleration = ReadLimitValue(node, "max_acceleration").as<double>();
  limits.max_torque = ReadLimitValue(node, "max_torque").as<double>();
  limits.max_kp = ReadLimitValue(node, "max_kp").as<double>();
  limits.max_ki = ReadLimitValue(node, "max_ki").as<double>();
  limits.max_kd = ReadLimitValue(node, "max_kd").as<double>();
  limits.max_temperature = ReadLimitValue(node, "max_temperature").as<double>();
  return limits;
}

ActuatorProfile ParseProfile(const YAML::Node & node)
{
  ActuatorProfile profile;
  profile.actuator_kind = ParseActuatorKind(node["kind"].as<std::string>());
  profile.gear_ratio = node["gear_ratio"].as<double>();
  profile.limits = ParseLimits(node["limits"]);
  profile.gains.soft = ParseGains(node["gains"]["soft"]);
  profile.gains.normal = ParseGains(node["gains"]["normal"]);
  profile.gains.stiff = ParseGains(node["gains"]["stiff"]);
  profile.controller.mode_transition_duration =
    node["controller"]["mode_transition_duration"].as<double>();
  profile.controller.max_time_step = node["controller"]["max_time_step"].as<double>();
  profile.safety.command_timeout = node["safety"]["command_timeout"].as<double>();
  profile.safety.feedback_timeout = node["safety"]["feedback_timeout"].as<double>();
  profile.safety.position_limit_tolerance =
    node["safety"]["position_limit_tolerance"].as<double>();
  profile.safety.velocity_limit_tolerance =
    node["safety"]["velocity_limit_tolerance"].as<double>();
  return profile;
}

bool IsFiniteAndNonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

void ValidateGains(
  const ControlGains & gains, const ActuatorLimits & limits, const std::string & context)
{
  if (!IsFiniteAndNonnegative(gains.kp) || !IsFiniteAndNonnegative(gains.ki) ||
    !IsFiniteAndNonnegative(gains.kd) || !IsFiniteAndNonnegative(gains.integral_limit))
  {
    throw std::invalid_argument(context + " contains an invalid control gain");
  }
  if (gains.kp > limits.max_kp || gains.ki > limits.max_ki || gains.kd > limits.max_kd) {
    throw std::invalid_argument(context + " exceeds its configured gain limit");
  }
}

void ValidateConfiguration(const ActuatorConfig & config)
{
  const auto & limits = config.limits;
  if (config.motor_index >= driver::kMotorCount) {
    throw std::invalid_argument("motor_index is outside RobotCommand::motors");
  }
  if (!std::isfinite(config.direction) || std::abs(std::abs(config.direction) - 1.0) > 1.0E-9) {
    throw std::invalid_argument("direction must be either +1 or -1");
  }
  if (!std::isfinite(config.zero_offset) || !std::isfinite(config.gear_ratio) ||
    config.gear_ratio <= 0.0)
  {
    throw std::invalid_argument("zero_offset or gear_ratio is invalid");
  }
  if (!std::isfinite(config.controller.mode_transition_duration) ||
    config.controller.mode_transition_duration < 0.0 ||
    !std::isfinite(config.controller.max_time_step) || config.controller.max_time_step <= 0.0)
  {
    throw std::invalid_argument("controller timing parameters are invalid");
  }
  if (!std::isfinite(config.safety.command_timeout) || config.safety.command_timeout <= 0.0 ||
    !std::isfinite(config.safety.feedback_timeout) || config.safety.feedback_timeout <= 0.0 ||
    !IsFiniteAndNonnegative(config.safety.position_limit_tolerance) ||
    !IsFiniteAndNonnegative(config.safety.velocity_limit_tolerance))
  {
    throw std::invalid_argument("safety timeout parameters are invalid");
  }
  if (IsWheel(config.joint_id) != (config.actuator_kind == ActuatorKind::kWheel)) {
    throw std::invalid_argument("joint_id and actuator kind disagree");
  }
  if (limits.position_limited &&
    (!std::isfinite(limits.min_position) || !std::isfinite(limits.max_position) ||
    limits.min_position >= limits.max_position))
  {
    throw std::invalid_argument("position limits are invalid");
  }
  if (!std::isfinite(limits.max_velocity) || limits.max_velocity <= 0.0 ||
    !std::isfinite(limits.max_acceleration) || limits.max_acceleration <= 0.0 ||
    !std::isfinite(limits.max_torque) || limits.max_torque <= 0.0 ||
    !IsFiniteAndNonnegative(limits.max_kp) || !IsFiniteAndNonnegative(limits.max_ki) ||
    !IsFiniteAndNonnegative(limits.max_kd) || !std::isfinite(limits.max_temperature) ||
    limits.max_temperature <= 0.0)
  {
    throw std::invalid_argument("actuator limits are invalid");
  }

  ValidateGains(config.gains.soft, limits, "soft profile");
  ValidateGains(config.gains.normal, limits, "normal profile");
  ValidateGains(config.gains.stiff, limits, "stiff profile");
}

void ValidateCommandValues(const JointCommand & command, const ControlGains & gains)
{
  if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
    !std::isfinite(command.torque_feedforward) || !IsFiniteAndNonnegative(gains.kp) ||
    !IsFiniteAndNonnegative(gains.ki) || !IsFiniteAndNonnegative(gains.kd))
  {
    throw std::invalid_argument("Cannot encode a non-finite command or negative gain");
  }
}

}  // namespace

ActuatorModel::ActuatorModel(const std::string & config_path)
: configurations_(LoadConfigurations(config_path))
{
}

const ActuatorModel::Configurations & ActuatorModel::GetConfigurations() const noexcept
{
  return configurations_;
}

const ActuatorConfig & ActuatorModel::GetConfiguration(const JointId joint_id) const
{
  const auto index = ToIndex(joint_id);
  if (index >= configurations_.size()) {
    throw std::out_of_range("JointId is outside the actuator configuration array");
  }
  return configurations_[index];
}

const ControlGains & ActuatorModel::GetGains(
  const JointId joint_id, const GainProfile profile) const
{
  const auto & gains = GetConfiguration(joint_id).gains;
  switch (profile) {
    case GainProfile::kSoft:
      return gains.soft;
    case GainProfile::kNormal:
      return gains.normal;
    case GainProfile::kStiff:
      return gains.stiff;
  }
  throw std::invalid_argument("Unknown gain profile");
}

JointStateFrame ActuatorModel::DecodeFeedback(const driver::RobotFeedback & feedback) const
{
  JointStateFrame result;
  result.received_at = feedback.received_at;
  result.sequence = feedback.tick;

  for (const auto & config : configurations_) {
    const auto joint_index = ToIndex(config.joint_id);
    const auto & motor = feedback.motors[config.motor_index];
    auto & joint = result.joints[joint_index];
    auto & status = result.status[joint_index];

    joint.position = MotorToJointPosition(config.joint_id, motor.position);
    joint.velocity = MotorToJointVelocity(config.joint_id, motor.velocity);
    joint.acceleration = MotorToJointVelocity(config.joint_id, motor.acceleration);
    joint.estimated_torque = MotorToJointTorque(config.joint_id, motor.estimated_torque);
    joint.temperature = static_cast<double>(motor.temperature);
    joint.source_lost_count = motor.lost;

    const bool values_are_finite =
      std::isfinite(joint.position) && std::isfinite(joint.velocity) &&
      std::isfinite(joint.acceleration) && std::isfinite(joint.estimated_torque);
    joint.online = motor.lost == 0U && values_are_finite;
    if (motor.lost != 0U) {
      status.faults |= ToMask(Fault::kMotorOffline);
    }
    if (!values_are_finite) {
      status.faults |= ToMask(Fault::kInvalidFeedback);
    }
  }
  return result;
}

driver::MotorCommand ActuatorModel::EncodeCommand(
  const JointId joint_id, const JointCommand & command, const ControlGains & gains) const
{
  ValidateCommandValues(command, gains);
  const auto & config = GetConfiguration(joint_id);
  driver::MotorCommand result;
  const double squared_ratio = config.gear_ratio * config.gear_ratio;

  result.position = static_cast<float>(JointToMotorPosition(joint_id, command.position));
  result.velocity = static_cast<float>(JointToMotorVelocity(joint_id, command.velocity));
  result.torque = static_cast<float>(JointToMotorTorque(
      joint_id, command.torque_feedforward));
  result.kp = static_cast<float>(gains.kp / squared_ratio);
  result.kd = static_cast<float>(gains.kd / squared_ratio);

  switch (command.control_mode) {
    case ControlMode::kDisabled:
      result.control_mode = driver::MotorControlMode::kDisabled;
      break;
    case ControlMode::kDamping:
      result.control_mode = driver::MotorControlMode::kVelocity;
      result.position = 0.0F;
      result.velocity = 0.0F;
      result.torque = 0.0F;
      result.kp = 0.0F;
      break;
    case ControlMode::kTorque:
      result.control_mode = driver::MotorControlMode::kTorque;
      break;
    case ControlMode::kVelocity:
      result.control_mode = driver::MotorControlMode::kVelocity;
      break;
    case ControlMode::kPosition:
      result.control_mode = driver::MotorControlMode::kPosition;
      break;
    case ControlMode::kHybrid:
      result.control_mode = driver::MotorControlMode::kHybrid;
      break;
  }
  return result;
}

double ActuatorModel::MotorToJointPosition(
  const JointId joint_id, const double motor_position) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.direction * (motor_position - config.zero_offset) / config.gear_ratio;
}

double ActuatorModel::JointToMotorPosition(
  const JointId joint_id, const double joint_position) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.zero_offset + config.direction * config.gear_ratio * joint_position;
}

double ActuatorModel::MotorToJointVelocity(
  const JointId joint_id, const double motor_velocity) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.direction * motor_velocity / config.gear_ratio;
}

double ActuatorModel::JointToMotorVelocity(
  const JointId joint_id, const double joint_velocity) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.direction * config.gear_ratio * joint_velocity;
}

double ActuatorModel::MotorToJointTorque(
  const JointId joint_id, const double motor_torque) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.direction * config.gear_ratio * motor_torque;
}

double ActuatorModel::JointToMotorTorque(
  const JointId joint_id, const double joint_torque) const
{
  const auto & config = GetConfiguration(joint_id);
  return config.direction * joint_torque / config.gear_ratio;
}

ActuatorModel::Configurations ActuatorModel::LoadConfigurations(const std::string & config_path)
{
  if (config_path.empty()) {
    throw std::invalid_argument("Actuator configuration path must not be empty");
  }

  try {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node profiles = root["actuator_profiles"];
    const YAML::Node actuators = root["actuators"];
    if (!profiles.IsMap() || !actuators.IsSequence()) {
      throw std::invalid_argument(
              "Actuator configuration requires actuator_profiles map and actuators sequence");
    }
    if (actuators.size() != kActuatorCount) {
      throw std::invalid_argument("Actuator configuration must define exactly 16 actuators");
    }

    Configurations result{};
    std::array<bool, kActuatorCount> seen_joints{};
    std::array<bool, driver::kMotorCount> seen_motors{};

    for (const auto & node : actuators) {
      const JointId joint_id = ParseJointId(node["joint_id"].as<std::string>());
      const std::string profile_name = node["profile"].as<std::string>();
      const YAML::Node profile_node = profiles[profile_name];
      if (!profile_node) {
        throw std::invalid_argument("Unknown actuator profile: " + profile_name);
      }

      const ActuatorProfile profile = ParseProfile(profile_node);
      ActuatorConfig config;
      config.joint_id = joint_id;
      config.actuator_kind = profile.actuator_kind;
      config.motor_index = node["motor_index"].as<std::size_t>();
      config.direction = node["direction"].as<double>();
      config.zero_offset = node["zero_offset"].as<double>();
      config.gear_ratio = profile.gear_ratio;
      config.limits = profile.limits;
      config.gains = profile.gains;
      config.controller = profile.controller;
      config.safety = profile.safety;
      ValidateConfiguration(config);

      const auto joint_index = ToIndex(joint_id);
      if (seen_joints[joint_index]) {
        throw std::invalid_argument("Duplicate joint_id in actuator configuration");
      }
      if (seen_motors[config.motor_index]) {
        throw std::invalid_argument("Duplicate motor_index in actuator configuration");
      }
      seen_joints[joint_index] = true;
      seen_motors[config.motor_index] = true;
      result[joint_index] = config;
    }

    if (!std::all_of(seen_joints.begin(), seen_joints.end(), [](const bool seen) {return seen;})) {
      throw std::invalid_argument(
              "One or more JointId entries are missing from actuator configuration");
    }
    return result;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to parse actuator configuration '" + config_path + "': " + error.what());
  }
}

}  // namespace wheel_dog_mujoco::actuator
