#include "gtest/gtest.h"
#include "wheel_dog_mujoco/skill/skill_task.h"

namespace wheel_dog_mujoco::skill
{
namespace
{

SkillTask::ActionResult Completed()
{
  SkillTask::ActionResult result;
  result.completed = true;
  return result;
}

}  // namespace

TEST(SkillTaskTest, StartsIdleAndSelectsDampingByDefault)
{
  SkillTask task(SkillTask::Config{});

  const auto output = task.Update(true, SkillTask::ActionResult{});

  EXPECT_EQ(output.state, SkillTask::State::kIdle);
  EXPECT_EQ(output.action, SkillTask::Action::kDamping);
  EXPECT_TRUE(task.IsIdle());
}

TEST(SkillTaskTest, StandRequestSelectsEveryPhaseInOrder)
{
  SkillTask task(SkillTask::Config{});
  task.RequestTarget(SkillTask::Target::kStand);

  const auto preparing = task.Update(true, SkillTask::ActionResult{});
  EXPECT_EQ(preparing.state, SkillTask::State::kPreparingStand);
  EXPECT_EQ(preparing.action, SkillTask::Action::kMoveToCrouch);

  const auto rising = task.Update(true, Completed());
  EXPECT_EQ(rising.state, SkillTask::State::kRising);
  EXPECT_EQ(rising.action, SkillTask::Action::kRiseToStand);

  const auto standing = task.Update(true, Completed());
  EXPECT_EQ(standing.state, SkillTask::State::kStanding);
  EXPECT_EQ(standing.action, SkillTask::Action::kHoldStand);
}

TEST(SkillTaskTest, IdleRequestSelectsLieDownBeforeDamping)
{
  SkillTask task(SkillTask::Config{});
  task.RequestTarget(SkillTask::Target::kStand);
  task.Update(true, SkillTask::ActionResult{});
  task.Update(true, Completed());
  task.Update(true, Completed());

  task.RequestTarget(SkillTask::Target::kIdle);
  const auto lying_down = task.Update(true, SkillTask::ActionResult{});
  EXPECT_EQ(lying_down.state, SkillTask::State::kLyingDown);
  EXPECT_EQ(lying_down.action, SkillTask::Action::kLieDown);

  const auto idle = task.Update(true, Completed());
  EXPECT_EQ(idle.state, SkillTask::State::kIdle);
  EXPECT_EQ(idle.action, SkillTask::Action::kDamping);
}

TEST(SkillTaskTest, NewStandTargetWaitsForLieDownToComplete)
{
  SkillTask task(SkillTask::Config{});
  task.RequestTarget(SkillTask::Target::kStand);
  task.Update(true, SkillTask::ActionResult{});
  task.Update(true, Completed());
  task.Update(true, Completed());
  task.RequestTarget(SkillTask::Target::kIdle);
  task.Update(true, SkillTask::ActionResult{});

  task.RequestTarget(SkillTask::Target::kStand);
  const auto still_lying_down = task.Update(true, SkillTask::ActionResult{});
  EXPECT_EQ(still_lying_down.state, SkillTask::State::kLyingDown);

  const auto rising = task.Update(true, Completed());
  EXPECT_EQ(rising.state, SkillTask::State::kRising);
  EXPECT_EQ(rising.action, SkillTask::Action::kRiseToStand);
}

TEST(SkillTaskTest, CanCancelWhilePreparingOrRising)
{
  SkillTask task(SkillTask::Config{});
  task.RequestTarget(SkillTask::Target::kStand);
  task.Update(true, SkillTask::ActionResult{});

  task.RequestTarget(SkillTask::Target::kIdle);
  const auto cancelled = task.Update(true, SkillTask::ActionResult{});
  EXPECT_EQ(cancelled.state, SkillTask::State::kLyingDown);
  EXPECT_EQ(cancelled.action, SkillTask::Action::kLieDown);
}

TEST(SkillTaskTest, FaultsOnInvalidFeedbackAndRecoversOnlyToIdle)
{
  SkillTask task(SkillTask::Config{});
  task.RequestTarget(SkillTask::Target::kStand);
  task.Update(true, SkillTask::ActionResult{});

  const auto fault = task.Update(false, SkillTask::ActionResult{});
  EXPECT_EQ(fault.state, SkillTask::State::kFault);
  EXPECT_EQ(fault.action, SkillTask::Action::kDamping);

  task.RequestTarget(SkillTask::Target::kStand);
  EXPECT_EQ(task.Update(true, SkillTask::ActionResult{}).state, SkillTask::State::kFault);

  task.RequestTarget(SkillTask::Target::kIdle);
  const auto recovered = task.Update(true, SkillTask::ActionResult{});
  EXPECT_EQ(recovered.state, SkillTask::State::kIdle);
}

TEST(SkillTaskTest, StandCanBeConfiguredAsInitialTarget)
{
  SkillTask::Config config;
  config.initial_target = SkillTask::Target::kStand;
  SkillTask task(config);

  EXPECT_EQ(task.GetState(), SkillTask::State::kPreparingStand);
  EXPECT_EQ(task.GetAction(), SkillTask::Action::kMoveToCrouch);
}

}  // namespace wheel_dog_mujoco::skill
