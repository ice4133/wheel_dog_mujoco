#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "wheel_dog_mujoco/skill/stand_skill.h"

namespace wheel_dog_mujoco::skill
{
namespace
{

actuator::JointStateFrame MakeState(
  const StandSkill::LegJointPositions & pose, const double velocity = 0.0)
{
  actuator::JointStateFrame state;
  for (auto & joint : state.joints) {
    joint.online = true;
    joint.velocity = velocity;
  }
  for (std::size_t index = 0U; index < pose.size(); ++index) {
    state.joints[index].position = pose[index];
  }
  return state;
}

}  // namespace

TEST(StandSkillTest, OnlyDescribesProfileAndRecognizesTargetPoses)
{
  StandSkill skill(StandSkill::Config{});

  EXPECT_TRUE(skill.IsCrouchReached(MakeState(skill.GetConfig().crouch_pose)));
  EXPECT_TRUE(skill.IsStandReached(MakeState(skill.GetConfig().stand_pose)));
}

TEST(StandSkillTest, PositionAndVelocityTolerancesAreEnforced)
{
  StandSkill skill(StandSkill::Config{});
  auto state = MakeState(skill.GetConfig().stand_pose);
  state.joints[0].position += skill.GetConfig().position_tolerance * 2.0;
  EXPECT_FALSE(skill.IsStandReached(state));

  state = MakeState(
    skill.GetConfig().stand_pose, skill.GetConfig().velocity_tolerance * 2.0);
  EXPECT_FALSE(skill.IsStandReached(state));
}

TEST(StandSkillTest, InvalidFeedbackCannotCompleteAPose)
{
  StandSkill skill(StandSkill::Config{});
  auto state = MakeState(skill.GetConfig().stand_pose);
  state.joints[15].online = false;
  EXPECT_FALSE(skill.IsFeedbackValid(state));
  EXPECT_FALSE(skill.IsStandReached(state));

  state = MakeState(skill.GetConfig().stand_pose);
  state.joints[3].position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(skill.IsFeedbackValid(state));
}

TEST(StandSkillTest, RejectsInvalidProfile)
{
  StandSkill::Config config;
  config.rise_duration = 0.0;
  EXPECT_THROW(StandSkill{config}, std::invalid_argument);
}

}  // namespace wheel_dog_mujoco::skill
