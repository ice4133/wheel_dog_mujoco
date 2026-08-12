# wheel_dog_mujoco

Go2W 四轮足的 ROS 2 推理与控制包。代码按“硬件通信层 → 执行器控制层 → 本体运动控制层
→ 运动技能层”组织，`BodyManager` 是本体运动控制层唯一的对外入口。

## 控制链路

```text
keyboard_controller                         upper command
        │ /cmd_vel                            │ /skill_request
        └──────────────────┐                  │
                           ▼                  ▼
                         SkillTaskNode
                     ▲       │
       profile/limits│       │target trajectory + command
                     │       ▼
                 StandSkill  SkillTask
                     │       ▲
                     └───────┘ action/state
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

`SkillTask` 是任务级状态机，显式管理 `idle → preparing_stand → rising → standing`
以及 `lying_down`、`fault` 状态，并为每个状态选择一个细粒度动作。`StandSkill` 不再执行
动作，只保存站立技能的姿态、时长、约束和“是否到位”判定。姿态插值、轮速计算、速度看门狗
和 `JointCommandFrame` 生成统一由 `SkillTaskNode` 负责。

初始目标由 `skill.task.initial_state` 配置，当前默认是 `idle`，因此不会把 `stand` 硬编码成
Idle 的起始行为。当前运行链路仍由 `SkillTaskNode` 直接输出到执行器层，整个站立过程不使用
IK；以后修正本体层后，可以只替换 TaskNode 的动作执行后端，而不改变状态机和技能特性。

`BodyManager::Update()` 只接收与传输无关的 `BodyCommandFrame`、`JointStateFrame` 和
`BodySensorFrame`，返回完整 `JointCommandFrame` 和本体状态。本体层不依赖 DDS 或 ROS 消息。

`WheelLegCoordinator` 是四轮足专有部分，负责：

- 将目标高度、roll、pitch 和四腿反馈补偿变成四个轮心目标；
- 通过逆运动学和雅可比速度解算生成 12 个腿关节指令；
- 将前进速度和 yaw 角速度分配为左右轮差速；
- 保持轮速饱和前后的曲率，并限制轮速加速度；
- 在工作空间边缘投影到最近可达目标，在真正不可恢复时输出阻尼回退。

`skill_task` 是当前唯一的 `/lowcmd` 发布者。后续姿态技能只提供各自特性和约束，由
`SkillTask` 统一选动作、`SkillTaskNode` 统一计算和发送，避免技能对象各自维护控制循环或争用
底层命令。

## 编译

必须先加载 ROS 环境，否则 `/usr/bin/python3` 找不到 ROS 的 `ament_package`：

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco
source install/setup.bash
```

## 启动仿真和站立技能

先启动配置为 `go2w` 的 `unitree_mujoco`。然后打开新的终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
ros2 run wheel_dog_mujoco skill_task
```

节点收到新鲜的 `/lowstate` 后保持 `idle` 阻尼状态。另开一个终端发送站立请求：

```bash
ros2 topic pub --once /skill_request std_msgs/msg/String "{data: stand}"
```

状态可通过 `/skill_state` 查看。请求返回 Idle 时，状态机会先执行趴下动作：

```bash
ros2 topic pub --once /skill_request std_msgs/msg/String "{data: idle}"
```

正常按 `Ctrl+C` 时也会先请求 Idle；如果正在站立，则先制动轮子并回到蹲姿再退出。
`SIGKILL` 无法执行安全返回流程。

常用 ROS 参数：

- `control_period`：技能控制周期，默认 `0.002 s`；
- `state_timeout`：底层反馈看门狗，默认 `0.2 s`；
- `velocity_timeout`：速度指令看门狗，默认 `0.25 s`；
- `crouch_pose`：12 个腿关节的安全蹲姿；
- `stand_pose`：12 个腿关节的固定站姿；
- `crouch_duration`：移动到安全蹲姿的时间，默认 `1.0 s`；
- `rise_duration`：蹲姿到站姿的时间，默认 `1.6 s`；
- `lie_down_duration`：退出时回到蹲姿的时间，默认 `1.5 s`；
- `position_tolerance`、`velocity_tolerance`：姿态动作完成所需的反馈容差；
- `shutdown_timeout`：趴下最长等待时间，默认 `4.0 s`；
- `system_config_path`：系统装配配置，默认安装目录下的 `config/system.yaml`；
- `driver_config_path`、`actuator_config_path`、`skill_config_path`：可选的分层配置覆盖。

例如延长起身时间：

```bash
ros2 run wheel_dog_mujoco skill_task --ros-args \
  -p rise_duration:=2.5
```

## 键盘控制与丝滑度检查

保持 `skill_task` 运行并发送 `stand` 请求，再打开一个具有相同 ROS/DDS 环境的新终端：

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

平滑性主要由以下参数控制：

- `crouch_duration`、`rise_duration` 和 `lie_down_duration`：关节平滑插值时间；
- `wheel_acceleration`：轮速加速度；
- `actuator_profiles.*.controller.mode_transition_duration`：执行器模式切换时间。

如果仿真仍有明显冲击，应先延长姿态插值时间，再调整执行器增益。

## 配置和安全边界

[config/system.yaml](config/system.yaml) 负责运行时装配并引用各层配置：
`driver.yaml` 保存 DDS 通信设置，`actuator.yaml` 保存执行器映射、硬限制、增益和安全参数，
`body.yaml` 保存本体几何、运动限制、估计器、轨迹和协调器参数，`skill.yaml` 保存当前技能
参数。轮腿顺序统一为 `FR, FL, RR, RL`，关节层使用语义化 `JointId`，电机编号和方向只
存在于执行器配置中。

当 `/cmd_vel` 中断时，TaskNode 看门狗会把目标轮速置零并按 `wheel_acceleration` 制动。
当 `/lowstate` 过期或反馈非法时，活动指令不会继续下发，节点改发安全回退命令。

## 单元测试

构建时开启测试后，可以运行任务状态机、执行器和本体控制链路测试：

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheel_dog_mujoco --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/wheel_dog_mujoco --output-on-failure \
  -R 'skill_task_test|stand_skill_test|actuator_manager_test|body_model_test|body_state_estimator_test|body_trajectory_test|body_controller_test|wheel_leg_coordinator_test|body_manager_test'
```
