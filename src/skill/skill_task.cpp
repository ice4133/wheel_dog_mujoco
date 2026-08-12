#include "wheel_dog_mujoco/skill/skill_task.h"

namespace wheel_dog_mujoco::skill
{

SkillTask::SkillTask(const Config & config)
: config_(config)
{
  Reset();
}

void SkillTask::RequestTarget(const Target target) noexcept
{
  requested_target_ = target;
}

SkillTask::Output SkillTask::Update(
  const bool feedback_valid, const ActionResult & action_result) noexcept
{
  const State previous_state = state_;
  const Action previous_action = action_;

  if (!feedback_valid || action_result.failed) {
    state_ = State::kFault;
    action_ = Action::kDamping;
  } else {
    switch (state_) {
      case State::kIdle:
        if (requested_target_ == Target::kStand) {
          state_ = State::kPreparingStand;
        }
        break;
      case State::kPreparingStand:
        if (requested_target_ == Target::kIdle) {
          state_ = State::kLyingDown;
        } else if (action_result.completed) {
          state_ = State::kRising;
        }
        break;
      case State::kRising:
        if (requested_target_ == Target::kIdle) {
          state_ = State::kLyingDown;
        } else if (action_result.completed) {
          state_ = State::kStanding;
        }
        break;
      case State::kStanding:
        if (requested_target_ == Target::kIdle) {
          state_ = State::kLyingDown;
        }
        break;
      case State::kLyingDown:
        if (action_result.completed) {
          state_ = requested_target_ == Target::kStand ?
            State::kRising : State::kIdle;
        }
        break;
      case State::kFault:
        if (requested_target_ == Target::kIdle) {
          state_ = State::kIdle;
        }
        break;
    }
    action_ = ActionForState(state_);
  }

  return Output{
    state_, action_, state_ != previous_state, action_ != previous_action};
}

void SkillTask::ForceFault() noexcept
{
  state_ = State::kFault;
  requested_target_ = Target::kIdle;
  action_ = Action::kDamping;
}

void SkillTask::Reset() noexcept
{
  requested_target_ = config_.initial_target;
  state_ = requested_target_ == Target::kStand ?
    State::kPreparingStand : State::kIdle;
  action_ = ActionForState(state_);
}

SkillTask::State SkillTask::GetState() const noexcept
{
  return state_;
}

SkillTask::Target SkillTask::GetRequestedTarget() const noexcept
{
  return requested_target_;
}

SkillTask::Action SkillTask::GetAction() const noexcept
{
  return action_;
}

bool SkillTask::IsIdle() const noexcept
{
  return state_ == State::kIdle;
}

const char * SkillTask::TargetName(const Target target) noexcept
{
  switch (target) {
    case Target::kIdle:
      return "idle";
    case Target::kStand:
      return "stand";
  }
  return "unknown";
}

const char * SkillTask::StateName(const State state) noexcept
{
  switch (state) {
    case State::kIdle:
      return "idle";
    case State::kPreparingStand:
      return "preparing_stand";
    case State::kRising:
      return "rising";
    case State::kStanding:
      return "standing";
    case State::kLyingDown:
      return "lying_down";
    case State::kFault:
      return "fault";
  }
  return "unknown";
}

const char * SkillTask::ActionName(const Action action) noexcept
{
  switch (action) {
    case Action::kDamping:
      return "damping";
    case Action::kMoveToCrouch:
      return "move_to_crouch";
    case Action::kRiseToStand:
      return "rise_to_stand";
    case Action::kHoldStand:
      return "hold_stand";
    case Action::kLieDown:
      return "lie_down";
  }
  return "unknown";
}

SkillTask::Action SkillTask::ActionForState(const State state) noexcept
{
  switch (state) {
    case State::kPreparingStand:
      return Action::kMoveToCrouch;
    case State::kRising:
      return Action::kRiseToStand;
    case State::kStanding:
      return Action::kHoldStand;
    case State::kLyingDown:
      return Action::kLieDown;
    case State::kIdle:
    case State::kFault:
      return Action::kDamping;
  }
  return Action::kDamping;
}

}  // namespace wheel_dog_mujoco::skill
