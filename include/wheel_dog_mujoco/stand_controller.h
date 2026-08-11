#pragma once

#include <array>
#include <cstddef>

namespace wheel_dog_mujoco
{

class StandController
{
public:
  static constexpr std::size_t kLegMotorCount = 12;
  static constexpr std::size_t kWheelMotorCount = 4;
  static constexpr std::size_t kWheelMotorOffset = 12;

  using JointPositions = std::array<double, kLegMotorCount>;

  enum class State
  {
    kIdle,
    kMovingToCrouch,
    kStandingUp,
    kHolding,
    kLyingDown,
    kLying,
  };

  struct Config
  {
    JointPositions crouch_pose{
      0.0, 1.36, -2.65,
      0.0, 1.36, -2.65,
      -0.2, 1.36, -2.65,
      0.2, 1.36, -2.65};
    JointPositions stand_pose{
      0.0, 0.67, -1.3,
      0.0, 0.67, -1.3,
      0.0, 0.67, -1.3,
      0.0, 0.67, -1.3};
    double crouch_duration{1.0};
    double stand_duration{1.6};
    double lie_down_duration{1.5};
  };

  StandController();
  explicit StandController(const Config & config);

  void SetConfig(const Config & config);
  const Config & GetConfig() const noexcept;

  void Start(const JointPositions & current_pose);
  void StartLieDown(const JointPositions & current_pose);
  void Reset() noexcept;
  void Update(double elapsed_seconds);

  const JointPositions & GetDesiredPose() const noexcept;
  State GetState() const noexcept;
  bool IsHolding() const noexcept;
  bool IsLying() const noexcept;

private:
  static void ValidateConfig(const Config & config);
  static JointPositions Interpolate(
    const JointPositions & from, const JointPositions & to, double ratio);
  static double SmoothStep(double ratio) noexcept;

  Config config_{};
  JointPositions initial_pose_{};
  JointPositions desired_pose_{};
  State state_{State::kIdle};
  double elapsed_seconds_{0.0};
};

}  // namespace wheel_dog_mujoco
