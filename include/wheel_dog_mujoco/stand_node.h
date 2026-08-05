#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "wheel_dog_mujoco/stand_controller.h"

namespace wheel_dog_mujoco
{

class StandNode : public rclcpp::Node
{
public:
  explicit StandNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyTimePoint = std::chrono::steady_clock::time_point;

  StandController::Config LoadControllerConfig();
  static StandController::JointPositions ToJointPositions(
    const std::vector<double> & values, const std::string & parameter_name);

  void InitializeCommand();
  void OnLowState(const unitree_go::msg::LowState::SharedPtr message);
  void OnControlTimer();
  void ApplyControllerCommand();
  void LogStateTransition(StandController::State state);

  StandController controller_{};
  unitree_go::msg::LowCmd low_command_{};
  unitree_go::msg::LowState low_state_{};

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr command_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_low_state_{false};
  bool controller_started_{false};
  double control_period_seconds_{0.002};
  double startup_delay_seconds_{1.0};
  double state_timeout_seconds_{0.2};
  SteadyTimePoint first_state_time_{};
  SteadyTimePoint last_state_time_{};
  SteadyTimePoint last_control_time_{};
  StandController::State last_logged_state_{StandController::State::kIdle};
};

}  // namespace wheel_dog_mujoco
