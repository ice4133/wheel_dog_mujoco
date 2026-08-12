#pragma once

#include <array>
#include <cstddef>

#include "wheel_dog_mujoco/actuator/actuator_type.h"

namespace wheel_dog_mujoco::skill
{

// Stateless stand profile. It describes the poses, timing and constraints of
// the stand skill, but owns no state machine, trajectory or actuator command.
class StandSkill
{
public:
  static constexpr std::size_t kLegJointCount = 12U;
  using LegJointPositions = std::array<double, kLegJointCount>;

  struct Config
  {
    LegJointPositions crouch_pose{
      0.0, 1.36, -2.65,
      0.0, 1.36, -2.65,
      -0.2, 1.36, -2.65,
      0.2, 1.36, -2.65};
    LegJointPositions stand_pose{
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30,
      0.0, 0.67, -1.30};
    double crouch_duration{1.0};
    double rise_duration{1.6};
    double lie_down_duration{1.5};
    double position_tolerance{0.08};
    double velocity_tolerance{0.5};
    double velocity_timeout{0.25};
    double wheel_radius{0.086};
    double track_width{0.284};
    double max_wheel_speed{6.0};
    double wheel_acceleration{12.0};
  };

  explicit StandSkill(const Config & config);

  const Config & GetConfig() const noexcept;
  bool IsFeedbackValid(const actuator::JointStateFrame & state) const noexcept;
  bool IsCrouchReached(const actuator::JointStateFrame & state) const noexcept;
  bool IsStandReached(const actuator::JointStateFrame & state) const noexcept;

private:
  bool IsPoseReached(
    const actuator::JointStateFrame & state,
    const LegJointPositions & target) const noexcept;

  Config config_{};
};

}  // namespace wheel_dog_mujoco::skill
