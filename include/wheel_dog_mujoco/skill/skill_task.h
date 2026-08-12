#pragma once

#include <cstdint>

namespace wheel_dog_mujoco::skill
{

// Pure task-level state machine. It chooses one fine-grained action at a time,
// but performs no trajectory calculation and has no ROS or actuator dependency.
class SkillTask
{
public:
  enum class Target : std::uint8_t
  {
    kIdle,
    kStand,
  };

  enum class State : std::uint8_t
  {
    kIdle,
    kPreparingStand,
    kRising,
    kStanding,
    kLyingDown,
    kFault,
  };

  enum class Action : std::uint8_t
  {
    kDamping,
    kMoveToCrouch,
    kRiseToStand,
    kHoldStand,
    kLieDown,
  };

  struct Config
  {
    Target initial_target{Target::kIdle};
  };

  struct ActionResult
  {
    bool completed{false};
    bool failed{false};
  };

  struct Output
  {
    State state{State::kIdle};
    Action action{Action::kDamping};
    bool state_changed{false};
    bool action_changed{false};
  };

  explicit SkillTask(const Config & config);

  void RequestTarget(Target target) noexcept;
  Output Update(bool feedback_valid, const ActionResult & action_result) noexcept;
  void ForceFault() noexcept;
  void Reset() noexcept;

  State GetState() const noexcept;
  Target GetRequestedTarget() const noexcept;
  Action GetAction() const noexcept;
  bool IsIdle() const noexcept;
  static const char * TargetName(Target target) noexcept;
  static const char * StateName(State state) noexcept;
  static const char * ActionName(Action action) noexcept;

private:
  static Action ActionForState(State state) noexcept;

  Config config_{};
  State state_{State::kIdle};
  Target requested_target_{Target::kIdle};
  Action action_{Action::kDamping};
};

}  // namespace wheel_dog_mujoco::skill
