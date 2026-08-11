# wheel_dog_mujoco

Go2W 的 ROS 2 C++ 控制包。目前提供低层站立控制节点，并将站立轨迹封装为可复用控制器。

## 硬件通信接口

`driver/drv_dds.h` 定义与传输方式无关的 `RobotCommand` 和 `RobotFeedback`。上层只需为
20 个电机选择 `Disabled`、`Torque`、`Velocity`、`Position` 或 `Hybrid` 控制模式并调用
`DrvDds::SendCommand()`。`drv_dds.cpp` 负责转换为 Unitree `LowCmd`、填写协议停机值、计算
CRC 和发布 DDS 消息；接收到的 `LowState` 会被转换为电机、IMU、电池、足端力和电源反馈，
通过 `DrvDds::GetFeedback()` 提供给上层。公共驱动头文件不暴露 Unitree ROS 消息类型。

## 执行器控制接口

`ActuatorManager` 是本层对后续本体运动控制层提供的统一入口。上层使用语义化的
`JointCommandFrame` 给 16 个关节指定位置、速度、力矩或混合控制目标；manager 依次执行
反馈解码、安全检查、限位/限速、控制模式平滑切换、增益选择和电机侧单位映射，最终产生
可直接交给 `DrvDds::SendCommand()` 的 `RobotCommand`。反馈则通过
`ActuatorManager::DecodeFeedback()` 转成 `JointStateFrame`，上层无需依赖电机编号和减速比。

执行器映射、极限、PID 增益、超时和模式切换参数统一放在 `config.yaml`。`stand_node` 已作为
完整链路测试程序接入 manager，不再直接拼装底层电机命令。

## 编译

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco
source install/setup.bash
```

## 仿真运行

先启动配置为 `go2w` 的 `unitree_mujoco`，然后在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
ros2 run wheel_dog_mujoco stand_node
```

节点收到 `/lowstate` 后才会向 `/lowcmd` 发布控制命令。可通过 ROS 参数调整
`crouch_pose`、`stand_pose`、`crouch_duration`、`stand_duration`、
`lie_down_duration`、`control_period`、`startup_delay`、`state_timeout` 和
`actuator_config_path`。执行器 PID 参数统一在 `config.yaml` 中调整。使用 `Ctrl+C` 或向节点发送
`SIGTERM` 时，节点会先停止轮子并平滑下降到
`crouch_pose`，完成后再退出；强制使用 `SIGKILL` 无法执行退出轨迹。

## 键盘控制

`stand_node` 是唯一的 `/lowcmd` 发布者：它负责站立、保持姿态、订阅 `/cmd_vel`
并换算四轮速度。键盘节点只发布标准 `geometry_msgs/msg/Twist`，两个节点需要同时运行：

```bash
ros2 run wheel_dog_mujoco stand_node
```

打开另一个终端并加载相同 ROS/DDS 环境：

```bash
ros2 run wheel_dog_mujoco keyboard_controller
```

键位：

- `W/S`：前进/后退
- `A/D`：原地左转/右转
- `Q/E`：向左前/右前弧线运动
- `Z/C`：向左后/右后弧线运动
- `Space` 或 `X`：立即停车
- `1/2/3`：低速/中速/高速档
- `H`：显示帮助
- `Esc`：停车并退出

运动键需要持续按住；松开、键盘节点退出或 `/cmd_vel` 中断后，站立节点的看门狗
都会自动停车。遥控器和 Nav2 后续也只需发布 `/cmd_vel`，不能直接发布 `/lowcmd`。

站立节点的移动参数包括 `velocity_topic`、`velocity_timeout`、`wheel_radius`、
`track_width`、`max_wheel_speed` 和 `wheel_acceleration`。键盘参数包括
`key_timeout`、`slow_linear_speed`、`normal_linear_speed`、`fast_linear_speed`，以及
对应的三个角速度档位。
