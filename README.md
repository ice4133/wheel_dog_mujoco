# wheel_dog_mujoco

Go2W 四轮足的 ROS 2 推理与控制包。代码按“硬件通信层 → 执行器控制层 → 本体运动控制层
→ 运动技能层”组织，`BodyManager` 是本体运动控制层唯一的对外入口。

## 控制链路

```text
keyboard_controller
                 │ /cmd_vel
                 ▼
             StandSkill
                 │ BodyCommandFrame
                 ▼
             BodyManager
                 ├─ BodyModel
                 ├─ BodyStateEstimator
                 ├─ BodyTrajectory
                 ├─ BodyController
                 └─ WheelLegCoordinator
                            │ JointCommandFrame
                            ▼
                     ActuatorManager
                            │ RobotCommand
                            ▼
                         DrvDds
                            │ /lowcmd, /lowstate
                            ▼
                    unitree_mujoco / 真机
```

`StandSkill` 属于 L4 运动技能层，负责“等待状态 → 恢复蹲姿 → 起身 → 站立移动 →
趴下”的有限状态流程。它只输出 `BodyCommandFrame`，不依赖 ROS、DDS 或电机接口。

`BodyManager::Update()` 只接收与传输无关的 `BodyCommandFrame`、`JointStateFrame` 和
`BodySensorFrame`，返回完整 `JointCommandFrame` 和本体状态。本体层不依赖 DDS 或 ROS 消息。

`WheelLegCoordinator` 是四轮足专有部分，负责：

- 将目标高度、roll、pitch 和四腿反馈补偿变成四个轮心目标；
- 通过逆运动学和雅可比速度解算生成 12 个腿关节指令；
- 将前进速度和 yaw 角速度分配为左右轮差速；
- 保持轮速饱和前后的曲率，并限制轮速加速度；
- 在工作空间边缘投影到最近可达目标，在真正不可恢复时输出阻尼回退。

旧的 `StandController` 和 `stand_node` 已移除。`body_node` 是运行组合器，只负责 ROS/DDS、
定时器和退出流程；站立流程属于 `StandSkill`，连续本体计算属于 `BodyManager`。

## 编译

必须先加载 ROS 环境，否则 `/usr/bin/python3` 找不到 ROS 的 `ament_package`：

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco
source install/setup.bash
```

## 启动仿真和本体控制

先启动配置为 `go2w` 的 `unitree_mujoco`。然后打开新的终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
ros2 run wheel_dog_mujoco body_node
```

节点收到新鲜的 `/lowstate` 后立即进入控制，先恢复到 `recovery_ground_clearance`，稳定后
再升到 `stand_ground_clearance`。到达站高前轮子保持停止。正常按 `Ctrl+C` 时，技能先制动
轮子，再下降到 `lie_down_ground_clearance`，最后退出；`SIGKILL` 无法执行趴下流程。

常用 ROS 参数：

- `control_period`：本体控制周期，默认 `0.002 s`；
- `state_timeout`：底层反馈看门狗，默认 `0.2 s`；
- `velocity_timeout`：速度指令看门狗，默认 `0.25 s`；
- `recovery_ground_clearance`：起身前的恢复蹲姿高度，默认 `0.20 m`；
- `stand_ground_clearance`：站立离地高度，默认 `0.42 m`；
- `lie_down_ground_clearance`：退出时趴下高度，默认 `0.20 m`；
- `settle_duration`：姿态稳定后进入下一阶段的等待时间，默认 `0.25 s`；
- `contact_force_feedback_available`：是否使用足端力，`unitree_mujoco` 默认 `false`；
- `shutdown_timeout`：趴下最长等待时间，默认 `4.0 s`；
- `config_path`：本体层和执行器层共同使用的 YAML 配置文件。

例如降低测试站高：

```bash
ros2 run wheel_dog_mujoco body_node --ros-args \
  -p stand_ground_clearance:=0.38
```

## 键盘控制与丝滑度检查

保持 `body_node` 运行，再打开一个具有相同 ROS/DDS 环境的新终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
ros2 run wheel_dog_mujoco keyboard_controller
```

键位：

- `W/S`：前进/后退；
- `A/D`：原地左转/右转；
- `Q/E`：左前/右前弧线；
- `Z/C`：左后/右后弧线；
- `Space` 或 `X`：停车；
- `1/2/3`：低速/中速/高速；
- `H`：显示帮助；
- `Esc`：停车并退出键盘节点。

第一次验证建议使用 `1` 挡，并按以下顺序观察：原地站起、短按 `W`、短按 `S`、原地
`A/D`、弧线 `Q/E`、停车、`Ctrl+C` 趴下。正常现象是轮速不会阶跃，松键后连续减速，转弯
时机身只有受限且经过滤波的侧倾补偿，不应出现明显的 pitch 点头或突然的 yaw 冲击。

平滑性主要由 [config.yaml](config.yaml) 中以下参数控制：

- `body.limits.max_linear_acceleration` 和 `max_angular_acceleration`：本体目标加速度；
- `body.coordinator.max_wheel_acceleration`：每个轮子的角加速度；
- `body.controller.gains`：高度、roll、pitch 和速度闭环；
- `body.controller.lateral_acceleration_filter_coefficient`：转弯侧倾滤波；
- `actuator_profiles.*.controller.mode_transition_duration`：执行器模式切换时间。

如果仿真仍有明显冲击，应先降低两个加速度限制，再调闭环增益；不要先单独加大关节 `kp`。

## 配置和安全边界

[config.yaml](config.yaml) 同时保存本体几何、运动限制、估计器、轨迹、协调器，以及执行器
映射、极限和 PID 增益。轮腿顺序统一为 `FR, FL, RR, RL`，关节层使用语义化 `JointId`，
电机编号和方向只存在于执行器配置中。

当 `/cmd_vel` 中断时，技能看门狗会把目标速度置零，再由轨迹与协调器按加速度限制制动。
当 `/lowstate` 过期、传感器非法或命令过期时，活动指令不会继续下发。仿真未提供足端力时，
估计器使用四轮运动学支撑假设；真机启用足端力后，真实的离地状态仍会触发接触丢失告警。

## 单元测试

构建时开启测试后，可以单独运行本体控制链路的 40 个测试用例：

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/wheel_dog_mujoco --output-on-failure \
  -R 'actuator_manager_test|body_model_test|body_state_estimator_test|body_trajectory_test|body_controller_test|wheel_leg_coordinator_test|body_manager_test'
```
