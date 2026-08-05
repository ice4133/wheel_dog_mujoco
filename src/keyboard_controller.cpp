#include "wheel_dog_mujoco/keyboard_controller.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

namespace wheel_dog_mujoco
{

KeyboardController::KeyboardController(const rclcpp::NodeOptions & options)
: Node("go2w_keyboard_teleop", options)
{
  publish_period_seconds_ = declare_parameter<double>("publish_period", 0.02);
  key_timeout_seconds_ = declare_parameter<double>("key_timeout", 0.65);
  curve_turn_ratio_ = declare_parameter<double>("curve_turn_ratio", 0.65);
  linear_speed_levels_[0] = declare_parameter<double>("slow_linear_speed", 0.15);
  linear_speed_levels_[1] = declare_parameter<double>("normal_linear_speed", 0.30);
  linear_speed_levels_[2] = declare_parameter<double>("fast_linear_speed", 0.50);
  angular_speed_levels_[0] = declare_parameter<double>("slow_angular_speed", 0.5);
  angular_speed_levels_[1] = declare_parameter<double>("normal_angular_speed", 1.0);
  angular_speed_levels_[2] = declare_parameter<double>("fast_angular_speed", 1.5);
  const auto velocity_topic = declare_parameter<std::string>("velocity_topic", "/cmd_vel");

  if (publish_period_seconds_ <= 0.0 || key_timeout_seconds_ <= 0.0 ||
    curve_turn_ratio_ <= 0.0 || curve_turn_ratio_ > 1.0 ||
    !std::all_of(
      linear_speed_levels_.begin(), linear_speed_levels_.end(),
      [](const double speed) {return speed > 0.0;}) ||
    !std::all_of(
      angular_speed_levels_.begin(), angular_speed_levels_.end(),
      [](const double speed) {return speed > 0.0;}))
  {
    throw std::invalid_argument("Keyboard teleop parameters are outside their valid range");
  }

  velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(velocity_topic, 10);
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(publish_period_seconds_));
  publish_timer_ = create_wall_timer(
    timer_period, std::bind(&KeyboardController::OnPublishTimer, this));

  ConfigureTerminal();
  RCLCPP_INFO(
    get_logger(), "Keyboard teleop publishes geometry_msgs/Twist on '%s'",
    velocity_topic.c_str());
  PrintHelp();
}

KeyboardController::~KeyboardController()
{
  RestoreTerminal();
}

void KeyboardController::ConfigureTerminal()
{
  if (!isatty(STDIN_FILENO)) {
    throw std::runtime_error("Keyboard teleop must be run from an interactive terminal");
  }
  if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) {
    throw std::runtime_error("Failed to read terminal settings");
  }

  termios raw_terminal = original_terminal_;
  raw_terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw_terminal.c_cc[VMIN] = 0;
  raw_terminal.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_terminal) != 0) {
    throw std::runtime_error("Failed to enable raw keyboard input");
  }

  original_stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (original_stdin_flags_ < 0 ||
    fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_ | O_NONBLOCK) != 0)
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
    throw std::runtime_error("Failed to enable non-blocking keyboard input");
  }
  terminal_configured_ = true;
}

void KeyboardController::RestoreTerminal() noexcept
{
  if (!terminal_configured_) {
    return;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
  fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_);
  terminal_configured_ = false;
}

void KeyboardController::PollKeyboard()
{
  char key = 0;
  errno = 0;
  ssize_t read_result = 0;
  while ((read_result = read(STDIN_FILENO, &key, 1)) == 1) {
    HandleKey(key);
  }
  if (read_result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to read keyboard input: errno=%d", errno);
  }
}

void KeyboardController::HandleKey(const char key)
{
  const char normalized = static_cast<char>(
    std::tolower(static_cast<unsigned char>(key)));

  switch (normalized) {
    case 'w':
      SetMotion(1.0, 0.0);
      break;
    case 's':
      SetMotion(-1.0, 0.0);
      break;
    case 'a':
      SetMotion(0.0, 1.0);
      break;
    case 'd':
      SetMotion(0.0, -1.0);
      break;
    case 'q':
      SetMotion(1.0, curve_turn_ratio_);
      break;
    case 'e':
      SetMotion(1.0, -curve_turn_ratio_);
      break;
    case 'z':
      SetMotion(-1.0, curve_turn_ratio_);
      break;
    case 'c':
      SetMotion(-1.0, -curve_turn_ratio_);
      break;
    case ' ':
    case 'x':
      StopMotion();
      PublishCommand();
      break;
    case '1':
      SelectSpeedLevel(0);
      break;
    case '2':
      SelectSpeedLevel(1);
      break;
    case '3':
      SelectSpeedLevel(2);
      break;
    case 'h':
      PrintHelp();
      break;
    case 27:
      StopMotion();
      PublishCommand();
      exit_requested_ = true;
      break;
    default:
      break;
  }
}

void KeyboardController::PrintHelp() const
{
  RCLCPP_INFO(
    get_logger(),
    "Keyboard controls:\n"
    "  W/S       forward/backward\n"
    "  A/D       turn left/right\n"
    "  Q/E       forward-left/forward-right arc\n"
    "  Z/C       backward-left/backward-right arc\n"
    "  Space/X   publish a zero-velocity command\n"
    "  1/2/3     slow/normal/fast speed\n"
    "  H         show this help\n"
    "  Esc       publish zero velocity and exit\n"
    "Hold a motion key to move; releasing it triggers the keyboard watchdog.");
}

void KeyboardController::OnPublishTimer()
{
  PollKeyboard();

  if (motion_key_active_) {
    const double command_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_motion_key_time_).count();
    if (command_age > key_timeout_seconds_) {
      StopMotion();
    }
  }

  PublishCommand();
  if (exit_requested_) {
    rclcpp::shutdown();
  }
}

void KeyboardController::SetMotion(
  const double linear_scale, const double angular_scale)
{
  velocity_command_.linear.x =
    std::clamp(linear_scale, -1.0, 1.0) * linear_speed_levels_[speed_level_index_];
  velocity_command_.angular.z =
    std::clamp(angular_scale, -1.0, 1.0) * angular_speed_levels_[speed_level_index_];
  motion_key_active_ = true;
  last_motion_key_time_ = std::chrono::steady_clock::now();
}

void KeyboardController::StopMotion()
{
  velocity_command_ = geometry_msgs::msg::Twist{};
  motion_key_active_ = false;
}

void KeyboardController::SelectSpeedLevel(const std::size_t level_index)
{
  speed_level_index_ = std::min(level_index, linear_speed_levels_.size() - 1);
  RCLCPP_INFO(
    get_logger(), "Speed level %zu: linear %.2f m/s, angular %.2f rad/s",
    speed_level_index_ + 1, linear_speed_levels_[speed_level_index_],
    angular_speed_levels_[speed_level_index_]);
}

void KeyboardController::PublishCommand()
{
  velocity_publisher_->publish(velocity_command_);
}

}  // namespace wheel_dog_mujoco

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_dog_mujoco::KeyboardController>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_controller"), "%s", exception.what());
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
