#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wheel_dog_mujoco/stand_node.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_dog_mujoco::StandNode>());
  rclcpp::shutdown();
  return 0;
}
