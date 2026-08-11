#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace wheel_dog_mujoco::actuator
{

enum class JointId : std::uint8_t
{
  kFrontRightHip,
  kFrontRightThigh,
  kFrontRightCalf,
  kFrontLeftHip,
  kFrontLeftThigh,
  kFrontLeftCalf,
  kRearRightHip,
  kRearRightThigh,
  kRearRightCalf,
  kRearLeftHip,
  kRearLeftThigh,
  kRearLeftCalf,
  kFrontRightWheel,
  kFrontLeftWheel,
  kRearRightWheel,
  kRearLeftWheel,
  kCount,
};

inline constexpr std::size_t kActuatorCount = static_cast<std::size_t>(JointId::kCount);

constexpr std::size_t ToIndex(const JointId joint_id) noexcept
{
  return static_cast<std::size_t>(joint_id);
}

enum class ActuatorKind : std::uint8_t
{
  kLegJoint,
  kWheel,
};

enum class ControlMode : std::uint8_t
{
  kDisabled,
  kDamping,
  kTorque,
  kVelocity,
  kPosition,
  kHybrid,
};

enum class GainProfile : std::uint8_t
{
  kSoft,
  kNormal,
  kStiff,
};

enum class Fault : std::uint32_t
{
  kNone = 0U,
  kCommandTimeout = 1U << 0U,
  kFeedbackTimeout = 1U << 1U,
  kMotorOffline = 1U << 2U,
  kInvalidCommand = 1U << 3U,
  kInvalidFeedback = 1U << 4U,
  kPositionLimit = 1U << 5U,
  kVelocityLimit = 1U << 6U,
  kTorqueLimit = 1U << 7U,
  kOverTemperature = 1U << 8U,
};

using FaultFlags = std::uint32_t;

constexpr FaultFlags ToMask(const Fault fault) noexcept
{
  return static_cast<FaultFlags>(fault);
}

constexpr bool HasFault(const FaultFlags flags, const Fault fault) noexcept
{
  return (flags & ToMask(fault)) != 0U;
}

struct ControlGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{0.0};
};

struct GainProfileSet
{
  ControlGains soft{};
  ControlGains normal{};
  ControlGains stiff{};
};

// All values use joint-side SI units: rad, rad/s, N*m and seconds.
struct JointCommand
{
  ControlMode control_mode{ControlMode::kDisabled};
  GainProfile gain_profile{GainProfile::kNormal};
  double position{0.0};
  double velocity{0.0};
  double torque_feedforward{0.0};
};

struct JointCommandFrame
{
  std::array<JointCommand, kActuatorCount> joints{};
  std::chrono::steady_clock::time_point created_at{};
  std::uint64_t sequence{0};
};

// Conservative zero defaults require every actuator to be configured before enabling it.
struct ActuatorLimits
{
  bool position_limited{true};
  double min_position{0.0};
  double max_position{0.0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_torque{0.0};
  double max_kp{0.0};
  double max_ki{0.0};
  double max_kd{0.0};
  double max_temperature{0.0};
};

struct ControllerParameters
{
  double mode_transition_duration{0.0};
  double max_time_step{0.0};
};

struct SafetyParameters
{
  double command_timeout{0.0};
  double feedback_timeout{0.0};
  // Feedback can overshoot a simulated or mechanical hard stop by a small amount.
  // Command targets are still clamped to the exact position and velocity limits.
  double position_limit_tolerance{0.0};
  double velocity_limit_tolerance{0.0};
};

struct ActuatorConfig
{
  JointId joint_id{JointId::kFrontRightHip};
  ActuatorKind actuator_kind{ActuatorKind::kLegJoint};
  std::size_t motor_index{0};
  // joint_position = direction * (motor_position - zero_offset) / gear_ratio
  double direction{1.0};
  double zero_offset{0.0};
  double gear_ratio{1.0};
  ActuatorLimits limits{};
  GainProfileSet gains{};
  ControllerParameters controller{};
  SafetyParameters safety{};
};

struct JointState
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double estimated_torque{0.0};
  double temperature{0.0};
  std::uint32_t source_lost_count{0};
  bool online{false};
};

struct ActuatorStatus
{
  ControlMode active_mode{ControlMode::kDisabled};
  FaultFlags faults{ToMask(Fault::kNone)};
  bool command_limited{false};
};

struct JointStateFrame
{
  std::array<JointState, kActuatorCount> joints{};
  std::array<ActuatorStatus, kActuatorCount> status{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0};
};

}  // namespace wheel_dog_mujoco::actuator
