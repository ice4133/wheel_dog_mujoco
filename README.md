# wheel_dog_mujoco

Go2W 的 ROS 2 C++ 控制包。目前提供低层站立控制节点，并将站立轨迹封装为可复用控制器。

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
`crouch_pose`、`stand_pose`、`crouch_duration`、`stand_duration`、`leg_kp`、
`leg_kd`、`wheel_kd`、`control_period`、`startup_delay` 和 `state_timeout`。

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
