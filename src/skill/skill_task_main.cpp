#include <chrono>
#include <csignal>
#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wheel_dog_mujoco/skill/skill_task_node.h"

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void HandleShutdownSignal([[maybe_unused]] const int signal_number)
{
  shutdown_requested = 1;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, HandleShutdownSignal);
  std::signal(SIGTERM, HandleShutdownSignal);

  auto node = std::make_shared<wheel_dog_mujoco::skill::SkillTaskNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && shutdown_requested == 0) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  executor.remove_node(node);
  if (rclcpp::ok() && shutdown_requested != 0) {
    node->ReturnToIdleBeforeShutdown();
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
