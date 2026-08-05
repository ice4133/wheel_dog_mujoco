#pragma once

#include <cstddef>
#include <cstdint>

#include "unitree_go/msg/low_cmd.hpp"

namespace wheel_dog_mujoco
{

inline constexpr float kPositionStop = 2.146E+9F;
inline constexpr float kVelocityStop = 16000.0F;

std::uint32_t ComputeCrc32(const std::uint32_t * data, std::size_t word_count);
void SetCrc(unitree_go::msg::LowCmd & command);

}  // namespace wheel_dog_mujoco
