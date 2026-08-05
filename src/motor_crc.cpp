#include "wheel_dog_mujoco/motor_crc.h"

#include <array>
#include <cstring>

namespace wheel_dog_mujoco
{
namespace
{

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
  std::array<RawMotorCommand, 20> motor_command;
  RawBmsCommand bms_command;
  std::array<std::uint8_t, 40> wireless_remote;
  std::array<std::uint8_t, 12> led;
  std::array<std::uint8_t, 2> fan;
  std::uint8_t gpio;
  std::uint32_t reserve;
  std::uint32_t crc;
};

static_assert(sizeof(RawLowCommand) == 812, "Unexpected Unitree LowCmd memory layout");

}  // namespace

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

void SetCrc(unitree_go::msg::LowCmd & command)
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

}  // namespace wheel_dog_mujoco
