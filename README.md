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
