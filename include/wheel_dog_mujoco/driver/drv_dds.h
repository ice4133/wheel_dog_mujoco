#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace wheel_dog_mujoco::driver
{

inline constexpr std::size_t kMotorCount = 20;
inline constexpr std::size_t kFootCount = 4;

enum class MotorControlMode : std::uint8_t
{
  kDisabled,
  kTorque,
  kVelocity,
  kPosition,
  kHybrid,
};

struct MotorCommand
{
  MotorControlMode control_mode{MotorControlMode::kDisabled};
  float position{0.0F};
  float velocity{0.0F};
  float torque{0.0F};
  float kp{0.0F};
  float kd{0.0F};
};

struct RobotCommand
{
  std::array<MotorCommand, kMotorCount> motors{};
  bool battery_power_off{false};
};

struct MotorFeedback
{
  std::uint8_t mode{0};
  float position{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float estimated_torque{0.0F};
  float raw_position{0.0F};
  float raw_velocity{0.0F};
  float raw_acceleration{0.0F};
  std::int8_t temperature{0};
  std::uint32_t lost{0};
};

struct ImuFeedback
{
  std::array<float, 4> quaternion{};
  std::array<float, 3> gyroscope{};
  std::array<float, 3> accelerometer{};
  std::array<float, 3> rpy{};
  std::int8_t temperature{0};
};

struct BatteryFeedback
{
  std::uint8_t version_high{0};
  std::uint8_t version_low{0};
  std::uint8_t status{0};
  std::uint8_t state_of_charge{0};
  std::int32_t current{0};
  std::uint16_t cycle_count{0};
  std::array<std::int8_t, 2> cell_temperature{};
  std::array<std::int8_t, 2> controller_temperature{};
  std::array<std::uint16_t, 15> cell_voltage{};
};

struct RobotFeedback
{
  std::array<MotorFeedback, kMotorCount> motors{};
  ImuFeedback imu{};
  BatteryFeedback battery{};
  std::array<std::int16_t, kFootCount> foot_force{};
  std::array<std::int16_t, kFootCount> estimated_foot_force{};
  std::uint32_t tick{0};
  std::array<std::uint8_t, 40> wireless_remote{};
  std::uint8_t bit_flag{0};
  float reel_adc{0.0F};
  std::int8_t power_temperature_1{0};
  std::int8_t power_temperature_2{0};
  float power_voltage{0.0F};
  float power_current{0.0F};
  std::array<std::uint16_t, kFootCount> fan_frequency{};
  std::chrono::steady_clock::time_point received_at{};
};

class DrvDds
{
public:
  struct Config
  {
    std::string command_topic{"/lowcmd"};
    std::string state_topic{"/lowstate"};
    std::size_t queue_depth{10};
  };

  explicit DrvDds(rclcpp::Node & node);
  DrvDds(rclcpp::Node & node, const Config & config);
  ~DrvDds();

  DrvDds(const DrvDds &) = delete;
  DrvDds & operator=(const DrvDds &) = delete;
  DrvDds(DrvDds &&) = delete;
  DrvDds & operator=(DrvDds &&) = delete;

  // Returns false and publishes nothing when the command contains an invalid mode,
  // a non-finite value, or a negative gain.
  bool SendCommand(const RobotCommand & command);
  std::optional<RobotFeedback> GetFeedback() const;
  bool IsFeedbackFresh(std::chrono::duration<double> maximum_age) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace wheel_dog_mujoco::driver
