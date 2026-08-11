#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace wheel_dog_mujoco::body
{

// Coordinate convention:
//   world/body x: forward, y: left, z: up (right-handed)
//   angular x/y/z: roll/pitch/yaw axes
// Quaternion order is w, x, y, z. All physical values use SI units.

enum class LegId : std::uint8_t
{
  kFrontRight,
  kFrontLeft,
  kRearRight,
  kRearLeft,
  kCount,
};

inline constexpr std::size_t kLegCount = static_cast<std::size_t>(LegId::kCount);

constexpr std::size_t ToIndex(const LegId leg_id) noexcept
{
  return static_cast<std::size_t>(leg_id);
}

enum class ReferenceFrame : std::uint8_t
{
  kBody,
  kWorld,
};

// This is a continuous body-control mode. Stand-up, lie-down and other finite
// actions belong to the motion-skill layer and are not enumerated here.
enum class ControlMode : std::uint8_t
{
  kDisabled,
  kPose,
  kVelocity,
  kHybrid,
};

enum class Fault : std::uint32_t
{
  kNone = 0U,
  kCommandTimeout = 1U << 0U,
  kSensorTimeout = 1U << 1U,
  kActuatorStateTimeout = 1U << 2U,
  kActuatorStateInvalid = 1U << 3U,
  kSensorStateInvalid = 1U << 4U,
  kStateEstimateInvalid = 1U << 5U,
  kKinematicsInfeasible = 1U << 6U,
  kBodyPoseLimit = 1U << 7U,
  kBodyVelocityLimit = 1U << 8U,
  kBodyAccelerationLimit = 1U << 9U,
  kContactLost = 1U << 10U,
  kControllerFailure = 1U << 11U,
  kInvalidCommand = 1U << 12U,
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

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose
{
  Vector3 position{};
  Quaternion orientation{};
};

struct Twist
{
  Vector3 linear{};
  Vector3 angular{};
};

struct SpatialAcceleration
{
  Vector3 linear{};
  Vector3 angular{};
};

struct Wrench
{
  Vector3 force{};
  Vector3 torque{};
};

// ==================== Motion-skill layer -> body layer ====================

// In kPose mode desired_pose is used; in kVelocity mode desired_twist is used.
// In kHybrid mode pose controls body height/roll/pitch while twist controls
// planar x/y velocity and yaw rate. Unused fields keep their safe defaults.
struct BodyCommand
{
  ControlMode control_mode{ControlMode::kDisabled};
  ReferenceFrame pose_frame{ReferenceFrame::kWorld};
  ReferenceFrame twist_frame{ReferenceFrame::kBody};
  Pose desired_pose{};
  Twist desired_twist{};
  SpatialAcceleration acceleration_feedforward{};
};

struct BodyCommandFrame
{
  BodyCommand command{};
  std::chrono::steady_clock::time_point created_at{};
  std::uint64_t sequence{0};
};

// ==================== Normalized sensors -> body layer ====================

// This sensor boundary is transport-independent. Converting a simulator or a
// hardware packet into BodySensorFrame is the responsibility of an outer adapter.
struct ImuState
{
  Quaternion orientation_world_from_body{};
  Vector3 angular_velocity_body{};
  Vector3 linear_acceleration_body{};
  double temperature{0.0};
  bool orientation_valid{false};
  bool angular_velocity_valid{false};
  bool linear_acceleration_valid{false};
};

struct ContactState
{
  double normal_force{0.0};
  bool in_contact{false};
  bool valid{false};
};

struct BodySensorFrame
{
  ImuState imu{};
  std::array<ContactState, kLegCount> contacts{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0};
};

// ==================== Body layer -> motion-skill layer ====================

// pose_world is world-referenced. twist_body and acceleration_body are expressed
// in the base_link body frame. ground_clearance is relative to the support surface.
struct BodyState
{
  Pose pose_world{};
  Twist twist_body{};
  SpatialAcceleration acceleration_body{};
  double ground_clearance{0.0};
  std::array<ContactState, kLegCount> contacts{};
  bool valid{false};
};

struct BodyStatus
{
  ControlMode active_mode{ControlMode::kDisabled};
  FaultFlags faults{ToMask(Fault::kNone)};
  bool command_limited{false};
  bool state_estimate_valid{false};
};

struct BodyStateFrame
{
  BodyState state{};
  BodyStatus status{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0};
};

// ==================== Shared body-layer configuration ====================

struct AxisControlGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{0.0};
};

// linear[0..2] map to x/y/z and angular[0..2] map to roll/pitch/yaw.
struct SpatialControlGains
{
  std::array<AxisControlGains, 3> linear{};
  std::array<AxisControlGains, 3> angular{};
};

struct BodyGeometry
{
  std::array<Vector3, kLegCount> hip_positions_body{};
  double hip_link_length{0.0};
  double thigh_link_length{0.0};
  double calf_link_length{0.0};
  double wheel_radius{0.0};
};

// Maxima are absolute magnitudes. Conservative zero defaults require explicit
// configuration before the body controller is enabled.
struct BodyLimits
{
  double min_ground_clearance{0.0};
  double max_ground_clearance{0.0};
  double max_roll{0.0};
  double max_pitch{0.0};
  Vector3 max_linear_velocity{};
  Vector3 max_angular_velocity{};
  Vector3 max_linear_acceleration{};
  Vector3 max_angular_acceleration{};
};

struct ControllerParameters
{
  SpatialControlGains gains{};
  double max_time_step{0.0};
  double max_leg_extension_adjustment{0.0};
  double velocity_posture_time_horizon{0.0};
  double turning_roll_gain{0.0};
  double max_turning_roll{0.0};
  double lateral_acceleration_filter_coefficient{0.0};
};

struct TrajectoryParameters
{
  double max_time_step{0.0};
  double position_tolerance{0.0};
  double orientation_tolerance{0.0};
  double linear_velocity_tolerance{0.0};
  double angular_velocity_tolerance{0.0};
};

struct CoordinatorParameters
{
  double max_time_step{0.0};
  double max_wheel_speed{0.0};
  double max_wheel_acceleration{0.0};
  double lateral_velocity_tolerance{0.0};
};

struct EstimatorParameters
{
  double orientation_filter_coefficient{0.0};
  double velocity_filter_coefficient{0.0};
  double contact_force_threshold{0.0};
  double gravity_magnitude{0.0};
  double max_time_step{0.0};
  bool accelerometer_includes_gravity{true};
};

struct SafetyParameters
{
  double command_timeout{0.0};
  double actuator_state_timeout{0.0};
  double sensor_state_timeout{0.0};
};

struct BodyConfig
{
  BodyGeometry geometry{};
  BodyLimits limits{};
  ControllerParameters controller{};
  TrajectoryParameters trajectory{};
  CoordinatorParameters coordinator{};
  EstimatorParameters estimator{};
  SafetyParameters safety{};
};

}  // namespace wheel_dog_mujoco::body
