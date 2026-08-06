#include "wheel_dog_mujoco/driver/drv_dds.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace wheel_dog_mujoco::driver
{
namespace
{

using LowCommand = unitree_go::msg::LowCmd;
using LowState = unitree_go::msg::LowState;

constexpr float kPositionStop = 2.146E+9F;
constexpr float kVelocityStop = 16000.0F;
constexpr std::uint8_t kServoMode = 0x01;

struct RawBmsCommand
{
  std::uint8_t off;
  std::array<std::uint8_t, 3> reserve;
};

struct RawMotorCommand
{
  std::uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  std::array<std::uint32_t, 3> reserve;
};

struct RawLowCommand
{
  std::array<std::uint8_t, 2> head;
  std::uint8_t level_flag;
  std::uint8_t frame_reserve;
  std::array<std::uint32_t, 2> serial_number;
  std::array<std::uint32_t, 2> version;
  std::uint16_t bandwidth;
  std::array<RawMotorCommand, kMotorCount> motor_command;
  RawBmsCommand bms_command;
  std::array<std::uint8_t, 40> wireless_remote;
  std::array<std::uint8_t, 12> led;
  std::array<std::uint8_t, 2> fan;
  std::uint8_t gpio;
  std::uint32_t reserve;
  std::uint32_t crc;
};

static_assert(sizeof(RawLowCommand) == 812, "Unexpected Unitree LowCmd memory layout");

std::uint32_t ComputeCrc32(const std::uint32_t * data, const std::size_t word_count)
{
  constexpr std::uint32_t polynomial = 0x04C11DB7U;
  std::uint32_t crc = 0xFFFFFFFFU;

  for (std::size_t word = 0; word < word_count; ++word) {
    std::uint32_t bit = 1U << 31U;
    const std::uint32_t value = data[word];
    for (std::size_t index = 0; index < 32; ++index) {
      if ((crc & 0x80000000U) != 0U) {
        crc = (crc << 1U) ^ polynomial;
      } else {
        crc <<= 1U;
      }
      if ((value & bit) != 0U) {
        crc ^= polynomial;
      }
      bit >>= 1U;
    }
  }
  return crc;
}

void SetCrc(LowCommand & command)
{
  RawLowCommand raw{};
  std::memcpy(raw.head.data(), command.head.data(), raw.head.size());
  raw.level_flag = command.level_flag;
  raw.frame_reserve = command.frame_reserve;
  std::memcpy(raw.serial_number.data(), command.sn.data(), sizeof(raw.serial_number));
  std::memcpy(raw.version.data(), command.version.data(), sizeof(raw.version));
  raw.bandwidth = command.bandwidth;

  for (std::size_t index = 0; index < raw.motor_command.size(); ++index) {
    auto & destination = raw.motor_command[index];
    const auto & source = command.motor_cmd[index];
    destination.mode = source.mode;
    destination.q = source.q;
    destination.dq = source.dq;
    destination.tau = source.tau;
    destination.kp = source.kp;
    destination.kd = source.kd;
    std::memcpy(destination.reserve.data(), source.reserve.data(), sizeof(destination.reserve));
  }

  raw.bms_command.off = command.bms_cmd.off;
  std::memcpy(
    raw.bms_command.reserve.data(), command.bms_cmd.reserve.data(),
    raw.bms_command.reserve.size());
  std::memcpy(
    raw.wireless_remote.data(), command.wireless_remote.data(), raw.wireless_remote.size());
  std::memcpy(raw.led.data(), command.led.data(), raw.led.size());
  std::memcpy(raw.fan.data(), command.fan.data(), raw.fan.size());
  raw.gpio = command.gpio;
  raw.reserve = command.reserve;

  constexpr std::size_t word_count = sizeof(RawLowCommand) / sizeof(std::uint32_t);
  std::array<std::uint32_t, word_count> words{};
  std::memcpy(words.data(), &raw, sizeof(raw));
  command.crc = ComputeCrc32(words.data(), word_count - 1U);
}

bool IsValid(const MotorCommand & command)
{
  const bool values_are_valid =
    std::isfinite(command.position) && std::isfinite(command.velocity) &&
    std::isfinite(command.torque) && std::isfinite(command.kp) &&
    std::isfinite(command.kd) && command.kp >= 0.0F && command.kd >= 0.0F;
  if (!values_are_valid) {
    return false;
  }

  switch (command.control_mode) {
    case MotorControlMode::kDisabled:
    case MotorControlMode::kTorque:
    case MotorControlMode::kVelocity:
    case MotorControlMode::kPosition:
    case MotorControlMode::kHybrid:
      return true;
  }
  return false;
}

void ConvertMotorCommand(const MotorCommand & source, unitree_go::msg::MotorCmd & destination)
{
  destination.mode = kServoMode;
  destination.q = kPositionStop;
  destination.dq = kVelocityStop;
  destination.tau = 0.0F;
  destination.kp = 0.0F;
  destination.kd = 0.0F;

  switch (source.control_mode) {
    case MotorControlMode::kDisabled:
      break;
    case MotorControlMode::kTorque:
      destination.tau = source.torque;
      break;
    case MotorControlMode::kVelocity:
      destination.dq = source.velocity;
      destination.tau = source.torque;
      destination.kd = source.kd;
      break;
    case MotorControlMode::kPosition:
      destination.q = source.position;
      destination.dq = 0.0F;
      destination.tau = source.torque;
      destination.kp = source.kp;
      destination.kd = source.kd;
      break;
    case MotorControlMode::kHybrid:
      destination.q = source.position;
      destination.dq = source.velocity;
      destination.tau = source.torque;
      destination.kp = source.kp;
      destination.kd = source.kd;
      break;
  }
}

LowCommand ConvertCommand(const RobotCommand & source)
{
  LowCommand destination{};
  destination.head[0] = 0xFE;
  destination.head[1] = 0xEF;
  destination.level_flag = 0xFF;
  destination.bms_cmd.off = source.battery_power_off ? 1U : 0U;

  for (std::size_t index = 0; index < source.motors.size(); ++index) {
    ConvertMotorCommand(source.motors[index], destination.motor_cmd[index]);
  }
  SetCrc(destination);
  return destination;
}

RobotFeedback ConvertFeedback(const LowState & source)
{
  RobotFeedback destination;
  for (std::size_t index = 0; index < destination.motors.size(); ++index) {
    const auto & input = source.motor_state[index];
    auto & output = destination.motors[index];
    output.mode = input.mode;
    output.position = input.q;
    output.velocity = input.dq;
    output.acceleration = input.ddq;
    output.estimated_torque = input.tau_est;
    output.raw_position = input.q_raw;
    output.raw_velocity = input.dq_raw;
    output.raw_acceleration = input.ddq_raw;
    output.temperature = input.temperature;
    output.lost = input.lost;
  }

  std::copy(source.imu_state.quaternion.begin(), source.imu_state.quaternion.end(),
    destination.imu.quaternion.begin());
  std::copy(source.imu_state.gyroscope.begin(), source.imu_state.gyroscope.end(),
    destination.imu.gyroscope.begin());
  std::copy(source.imu_state.accelerometer.begin(), source.imu_state.accelerometer.end(),
    destination.imu.accelerometer.begin());
  std::copy(source.imu_state.rpy.begin(), source.imu_state.rpy.end(), destination.imu.rpy.begin());
  destination.imu.temperature = source.imu_state.temperature;

  destination.battery.version_high = source.bms_state.version_high;
  destination.battery.version_low = source.bms_state.version_low;
  destination.battery.status = source.bms_state.status;
  destination.battery.state_of_charge = source.bms_state.soc;
  destination.battery.current = source.bms_state.current;
  destination.battery.cycle_count = source.bms_state.cycle;
  std::copy(source.bms_state.bq_ntc.begin(), source.bms_state.bq_ntc.end(),
    destination.battery.cell_temperature.begin());
  std::copy(source.bms_state.mcu_ntc.begin(), source.bms_state.mcu_ntc.end(),
    destination.battery.controller_temperature.begin());
  std::copy(source.bms_state.cell_vol.begin(), source.bms_state.cell_vol.end(),
    destination.battery.cell_voltage.begin());

  std::copy(source.foot_force.begin(), source.foot_force.end(), destination.foot_force.begin());
  std::copy(source.foot_force_est.begin(), source.foot_force_est.end(),
    destination.estimated_foot_force.begin());
  destination.tick = source.tick;
  std::copy(source.wireless_remote.begin(), source.wireless_remote.end(),
    destination.wireless_remote.begin());
  destination.bit_flag = source.bit_flag;
  destination.reel_adc = source.adc_reel;
  destination.power_temperature_1 = source.temperature_ntc1;
  destination.power_temperature_2 = source.temperature_ntc2;
  destination.power_voltage = source.power_v;
  destination.power_current = source.power_a;
  std::copy(source.fan_frequency.begin(), source.fan_frequency.end(),
    destination.fan_frequency.begin());
  destination.received_at = std::chrono::steady_clock::now();
  return destination;
}

}  // namespace

class DrvDds::Impl
{
public:
  Impl(rclcpp::Node & node, const Config & config)
  {
    if (config.command_topic.empty() || config.state_topic.empty() || config.queue_depth == 0U) {
      throw std::invalid_argument(
              "Driver topics must not be empty and queue depth must be positive");
    }

    command_publisher_ = node.create_publisher<LowCommand>(
      config.command_topic, config.queue_depth);
    state_subscription_ = node.create_subscription<LowState>(
      config.state_topic, config.queue_depth,
      std::bind(&Impl::OnState, this, std::placeholders::_1));
  }

  bool SendCommand(const RobotCommand & command)
  {
    if (!std::all_of(command.motors.begin(), command.motors.end(), IsValid)) {
      return false;
    }
    command_publisher_->publish(ConvertCommand(command));
    return true;
  }

  std::optional<RobotFeedback> GetFeedback() const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_;
  }

  bool IsFeedbackFresh(const std::chrono::duration<double> maximum_age) const
  {
    if (maximum_age.count() <= 0.0) {
      return false;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_.has_value() &&
           std::chrono::steady_clock::now() - feedback_->received_at <= maximum_age;
  }

private:
  void OnState(const LowState::SharedPtr message)
  {
    RobotFeedback feedback = ConvertFeedback(*message);
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_ = std::move(feedback);
  }

  mutable std::mutex feedback_mutex_;
  std::optional<RobotFeedback> feedback_;
  rclcpp::Publisher<LowCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<LowState>::SharedPtr state_subscription_;
};

DrvDds::DrvDds(rclcpp::Node & node)
: DrvDds(node, Config{})
{
}

DrvDds::DrvDds(rclcpp::Node & node, const Config & config)
: impl_(std::make_unique<Impl>(node, config))
{
}

DrvDds::~DrvDds() = default;

bool DrvDds::SendCommand(const RobotCommand & command)
{
  return impl_->SendCommand(command);
}

std::optional<RobotFeedback> DrvDds::GetFeedback() const
{
  return impl_->GetFeedback();
}

bool DrvDds::IsFeedbackFresh(const std::chrono::duration<double> maximum_age) const
{
  return impl_->IsFeedbackFresh(maximum_age);
}

}  // namespace wheel_dog_mujoco::driver
