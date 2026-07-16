# Quest3 → Astribot S1 双臂遥操作实施计划

## 1. Goal

在不修改 Astribot Orin 现有 250 Hz 驱动栈、不直接发布内部关节控制 topic 的前提下，实现以下可验收目标：

```text
Quest3 WebXR 左右手位姿
  -> 控制 PC 最新帧缓冲
  -> VR 到 chassis 坐标映射（relative / absolute 可切换）
  -> clutch/recenter、滤波、限速、工作空间约束
  -> 控制 PC 上的 Astribot Python API
  -> ROS2 网络
  -> Orin 现有 Astribot 控制栈、IK 和关节控制
  -> S1 左右机械臂
```

第一版以 `100 Hz`、`control_way="filter"`、`use_wbc=False` 运行，只控制左右机械臂，不控制 torso、head 和 chassis。映射同时实现 `relative` 和 `absolute` 两种模式，通过配置或 CLI 选择；先完成单臂低速验证，再启用双臂和姿态控制。

## 2. 已确认的接口事实

控制 PC 与 Orin 的 ROS2 网络已经连通：

- `ROS_DOMAIN_ID=25`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Fast DDS 使用 `192.168.0.x` 网段
- 控制 PC 能发现 Orin 节点和机械臂状态 topic

已确认机械臂状态 topic：

| Topic | 类型 | 方向 | 结论 |
|---|---|---|---|
| `/astribot_arm_left/endpoint_current_states` | `astribot_msgs/msg/RobotCartesianState` | Orin 发布 | 当前末端状态，只读 |
| `/astribot_arm_right/endpoint_current_states` | `astribot_msgs/msg/RobotCartesianState` | Orin 发布 | 当前末端状态，只读 |
| `/astribot_arm_left/endpoint_desired_states` | `astribot_msgs/msg/RobotCartesianState` | Orin 发布 | SDK/控制器期望状态，只读 |
| `/astribot_arm_right/endpoint_desired_states` | `astribot_msgs/msg/RobotCartesianState` | Orin 发布 | SDK/控制器期望状态，只读 |
| `/astribot_arm_left/joint_space_command` | `astribot_msgs/msg/RobotJointController` | robotics library 发布、Orin 消费 | 内部低层关节命令，不允许 teleop 直接发布 |
| `/astribot_arm_right/joint_space_command` | `astribot_msgs/msg/RobotJointController` | robotics library 发布、Orin 消费 | 内部低层关节命令，不允许 teleop 直接发布 |
| `/astribot_arm_*/joint_space_command_recv` | `astribot_msgs/msg/RobotJointState` | Orin 发布 | 接收状态/回显，只读 |

`endpoint_current_states` 和 `endpoint_desired_states` 实测约为 `250 Hz`。当前 ROS 图中没有公开的外部 Cartesian pose subscriber，因此第一版不自行定义消息并绕过 SDK，也不向 `joint_space_command` 发布。

机械臂命令的唯一入口采用官方接口：

```python
astribot.set_cartesian_pose(
    [astribot.arm_left_name, astribot.arm_right_name],
    [left_target_pose, right_target_pose],
    control_way="filter",
    use_wbc=False,
    add_default_torso=True,
)
```

控制 PC 运行 `Astribot(freq=100.0)` 客户端。该客户端通过 ROS2 与 Orin 驱动通信并获取控制权；Orin 继续运行现有硬件、IK 和关节控制节点。启动 teleop 前必须确保没有另一个 SDK 客户端或 Web/VR 控制程序同时持有机器人控制权。

## 3. 最终控制架构

```text
┌───────────────────────┐
│ Quest3 Browser/WebXR  │
│ HMD + left/right hand │
└──────────┬────────────┘
           │ WebSocket tracking_frame
           v
┌─────────────────────────────────────────────────────────┐
│ Control PC                                              │
│                                                         │
│ WebXR receive task                                      │
│   └── overwrite-only LatestTrackingBuffer               │
│                                                         │
│ 100 Hz ArmControlLoop                                   │
│   ├── freshness/session/tracking checks                 │
│   ├── TeleopStateMachine + deadman + clutch             │
│   ├── ArmMapper                                         │
│   │   ├── operator/world pose                           │
│   │   ├── R_BV axis transform                           │
│   │   ├── relative / absolute position and orientation  │
│   │   └── per-axis scale                                │
│   ├── pose filter                                       │
│   ├── workspace + linear/angular velocity limits        │
│   └── AstribotAdapter.send_targets()                    │
│                                                         │
│ Astribot Python API                                     │
│   ├── get_desired_cartesian_pose() for recenter         │
│   ├── get_current_cartesian_pose() for monitoring       │
│   └── set_cartesian_pose() for commands                 │
└──────────┬──────────────────────────────────────────────┘
           │ ROS2 / Fast DDS, ROS_DOMAIN_ID=25
           v
┌─────────────────────────────────────────────────────────┐
│ Robot Orin                                              │
│ existing Astribot nodes at 250 Hz                       │
│ control rights + IK + filtering + joint command         │
│ endpoint current/desired state feedback                 │
└──────────┬──────────────────────────────────────────────┘
           v
┌───────────────────────┐
│ S1 left/right arms    │
└───────────────────────┘
```

### 3.1 线程和数据所有权

- WebSocket 接收任务只验证并覆盖最新帧，不执行机器人调用。
- `LatestTrackingBuffer` 最大只保存一帧，禁止队列积压。
- 机械臂控制线程独占 `Astribot` 实例，所有 SDK 调用都在同一线程完成。
- 状态/UI/日志线程只读取控制循环生成的 telemetry，不直接调用机器人。
- WujiHand 保留现有实现；双手和双臂只共享状态机、deadman、clutch 和时间基准。

### 3.2 位姿约定

- WebXR 页面当前将 HMD 和手关节都表达在同一 `local` 或 `local-floor` reference space，因此不需要再次计算 `T_V_H * T_H_C`。
- 如果未来输入源只提供手相对头部位姿，则在进入 mapper 前先恢复 `T_V_C = T_V_H * T_H_C`。
- Astribot 目标统一表达在 `astribot.chassis_frame_name` 下。
- 目标格式固定为 `[x, y, z, qx, qy, qz, qw]`，位置单位为米。
- `relative` 模式使用每次 engage 时采集的 VR 手腕和机器人末端基准。
- 第一版 `absolute` 是 `absolute_session`：每次 WebXR session 在控制 PC 标定一次，冻结标定时的 HMD yaw 和操作者原点，并为每侧计算手腕到末端的对齐。
- 跨 WebXR session 持久有效的 `absolute_world` 需要 spatial anchor、外部视觉标记或物理对应点标定，不属于第一版范围。
- WebXR 页面必须监听 `XRReferenceSpace` 的 `reset` 事件，并在每帧发送递增的 `reference_space_revision`；同一 session 内 revision 改变也会使 Absolute Session 标定失效。
- 两种模式共用相同的滤波、workspace、速度、超时和状态机安全层。

## 4. 映射和安全算法

### 4.1 映射模式和切换规则

支持以下两种模式：

| 模式 | 基准 | 适用场景 | 必需标定 |
|---|---|---|---|
| `relative` | engage 时的 VR 手腕和机器人末端位姿 | 默认遥操、clutch/recenter、跨用户快速使用 | `R_BV` 轴转换 |
| `absolute` | 当前 WebXR session 内冻结的操作者坐标系 | 同一 session 内固定工作空间和可重复位置 | HMD/双手/机器人 neutral pose 多帧标定 |

CLI 和配置都支持选择：

```text
--mapping-mode relative
--mapping-mode absolute
```

运行时允许切换，但必须遵守：

1. 只允许在 `IDLE`、`ARMED` 或 `PAUSED` 状态切换，`RUNNING` 中拒绝切换。
2. 切换时清空 mapper/filter 历史和未发送候选目标。
3. `relative` 模式切换后必须重新 engage/recenter。
4. `absolute` 模式切换前必须存在当前 `xr_session_id` 下有效的 session 标定，并校验 reference space 和当前绝对候选目标。
5. 将新模式首个候选目标与机器人当前 desired pose 比较；超过 `mode_switch_max_position_jump_m` 或 `mode_switch_max_rotation_jump_rad` 时拒绝 start。
6. 模式切换本身不发送机器人命令，操作者确认并执行 `start` 后才恢复输出。

### 4.2 Relative 模式：engage / clutch

只有以下条件全部满足才能进入 `ARMED`：

- SDK 初始化完成并持有控制权；
- 左右末端 desired pose 可读取；
- WebXR session active；
- 被启用侧 tracking valid；
- 最新帧未超时；
- 急停已释放；
- 工作空间和限速配置加载成功。

执行 `engage` 或 `recenter` 时，对每个启用侧记录：

```text
T_V_C0：当前 VR 手腕位姿
T_B_E0：当前机器人 desired 末端位姿
R_BV：VR/operator 到 chassis 的固定轴变换
```

首个输出目标必须等于 `T_B_E0`，不得在 engage 时跳变。

Relative 平移映射：

```text
delta_p_V = p_V_C(t) - p_V_C0
delta_p_B = R_BV * delta_p_V
p_B_target = p_B_E0 + scale_xyz ⊙ delta_p_B
```

第一轮实机建议：

```yaml
position_scale_xyz: [0.3, 0.3, 0.3]
max_linear_speed_mps: 0.10
```

轴向验证完成后，可逐步增加到 `0.2 m/s` 和 `0.5` 比例。

Relative 姿态映射：

```text
R_delta_V = R_V_C(t) * R_V_C0^T
R_delta_B = R_BV * R_delta_V * R_BV^T
R_B_target = scaled(R_delta_B, rotation_scale) * R_B_E0
```

`rotation_scale` 必须通过旋转向量或 quaternion slerp 实现，不能只保存配置而不使用。第一轮单臂实机保持初始机器人姿态；平移验证通过后再启用：

```yaml
enable_orientation: true
rotation_scale: 0.5
max_angular_speed_rad_s: 0.5
```

每次使用四元数前都归一化，并选择与上一帧点积为正的符号，避免 `q/-q` 跳变。

### 4.3 Absolute Session 模式：控制 PC 多帧标定

第一版 Absolute 不尝试建立跨 session 的物理 `T_B_V`。它在当前 WebXR session 内建立并冻结操作者坐标系 `O`：

```text
V：当前 WebXR local/local-floor reference space
H：HMD
O：标定时冻结的操作者坐标系
C：VR 手腕
B：机器人 chassis
E：机器人末端
```

#### 4.3.1 用户操作流程

标定入口位于控制 PC，不放在 Quest 页面。第一版至少提供等价的 CLI 命令；PC GUI 可在同一接口之上实现：

```text
进入 WebXR
  -> 机器人到 neutral pose
  -> 操作者站到地面标记位置并正对规定方向
  -> 双手摆到约定 neutral pose
  -> 控制 PC 点击“Absolute 标定”或执行 absolute-calibrate
  -> 3/2/1 倒计时
  -> 保持 1.5～2 秒
  -> 质量检查通过，状态变为 ABSOLUTE_CALIBRATED
  -> 只计算首目标并执行 jump gate
  -> 操作者单独点击 Start，才允许发送命令
```

点击标定只采样和计算，绝不触发机器人运动，也不自动进入 `RUNNING`。

标定状态明确为：

```text
UNCALIBRATED -> COUNTDOWN -> SAMPLING -> VALID
                         \-> INVALID
```

`Pause`、`Stop`、`E-Stop`、session 失效或操作者点击 `Cancel Calibration` 都必须立即取消倒计时/采样、丢弃未完成样本并进入 `INVALID` 或 `UNCALIBRATED`，不得留下部分有效标定。

#### 4.3.2 标定采样

标定期间连续同步采集：

```text
T_V_H(t)：HMD pose
T_V_C_left/right(t)：左右手腕 pose
T_B_E_left/right(t)：机器人左右末端 desired pose
xr_session_id
reference_space
reference_space_revision
本地 monotonic 接收时间
```

机器人末端基准必须来自：

```python
astribot.get_desired_cartesian_pose(
    names=[astribot.arm_left_name, astribot.arm_right_name],
    frame=astribot.chassis_frame_name,
)
```

采样时系统保持 `PAUSED`。默认采集 `1.5 s`，至少 `60` 个有效样本；对位置使用稳健均值或剔除离群值后的均值，对四元数使用符号统一后的旋转平均，禁止直接逐元素平均未对齐符号的四元数。

#### 4.3.3 冻结操作者坐标系

从平均 HMD pose 构造 `T_V_O`：

- 朝向只取标定时平均 HMD yaw；roll 和 pitch 强制为零。
- `local-floor`：原点使用平均 HMD 的水平 `x/z`，竖直原点使用地面高度。
- `local`：原点使用平均 HMD 的 `x/y/z`，且标定仅对当前 session 有效。
- `T_V_O` 标定完成后冻结，运行时不再随实时 HMD 更新。

因此标定后操作者转头、抬头或低头不会直接驱动机械臂。

每侧计算：

```text
T_O_C0 = inverse(T_V_O) * T_V_C0
T_B_E0 = 标定期间平均机器人 desired pose
```

位置 Absolute Session 映射：

```text
p_O_C(t) = position(inverse(T_V_O) * T_V_C(t))
p_B_target = p_B_E0
             + scale_xyz ⊙ (R_B_O * (p_O_C(t) - p_O_C0))
```

其中 `R_B_O` 来自已验证的 operator→chassis 轴转换。这里的 `p_B_E0` 是标定时固定的机器人 workspace anchor；pause/resume 不更新它，否则会退化成 Relative。

姿态对齐在标定时计算，使首目标严格等于 `R_B_E0`：

```text
R_C_E = inverse(R_B_O * R_O_C0) * R_B_E0
R_B_target(t) = R_B_O * R_O_C(t) * R_C_E
```

如果使用 `rotation_scale != 1`，围绕标定姿态 `R_O_C0` 缩放姿态增量，再与 `R_B_E0` 合成。

#### 4.3.4 标定质量门限

默认质量门限：

```yaml
absolute_session_calibration:
  countdown_sec: 3.0
  sample_duration_sec: 1.5
  minimum_valid_samples: 60
  max_head_position_std_m: 0.010
  max_hand_position_std_m: 0.015
  max_head_yaw_std_rad: 0.050
  max_hand_rotation_std_rad: 0.090
  max_robot_position_std_m: 0.005
  max_robot_rotation_std_rad: 0.030
  max_robot_desired_current_error_m: 0.020
```

任一启用侧 tracking 无效、样本不足、reference space/session 中途变化、操作者移动过大或机器人不稳定时，标定失败并保持 `PAUSED`。失败信息必须指出具体侧和具体门限。

#### 4.3.5 会话绑定和失效条件

标定结果仅保存在内存或作为诊断记录保存，不允许在下一个 WebXR session 自动复用。它绑定：

```text
xr_session_id
reference_space
reference_space_revision
标定开始/结束 monotonic time
启用的 arm sides
标定质量统计
```

发生以下任一事件立即将 Absolute 标定标记为无效：

- WebXR session end 或 `xr_session_id` 改变；
- reference space 类型改变；
- `reference_space_revision` 改变，即页面/设备触发 `XRReferenceSpace reset`；
- HMD invalid、全局 tracking reset 或系统检测到 origin discontinuity；
- 机器人重新回零、被其他控制源移动或重新获取控制权；
- 操作者显式执行 `invalidate-calibration`。

失效后必须重新点击 Absolute 标定，不能只点 Resume。

#### 4.3.6 Pause、clutch 和模式切换

- Absolute Session 中 clutch 只负责 pause/hold，不修改 `T_V_O`、`T_O_C0` 或 `T_B_E0`。
- Resume 前重新计算当前绝对候选目标并执行 jump gate。
- 若暂停期间手移动过远，系统保持 `PAUSED`，要求操作者回到可接受位置、重新标定或切回 Relative。
- 从 Relative 切到 Absolute 必须已有当前 session 的有效标定；切换本身不发送命令。
- 第一版不实现隐式 `absolute_nudge_offset`，避免 Absolute 悄悄退化为 Relative。

### 4.4 两种模式共用的滤波与限速

固定处理顺序：

```text
raw mapped target (relative or absolute)
  -> discontinuity rejection
  -> low-pass/One Euro filter
  -> workspace clamp
  -> dt-aware linear speed limit
  -> dt-aware angular speed limit
  -> final finite/quaternion validation
```

速度限制基于实际 `dt`，而不是假设每周期恒定：

```text
max_translation_step = max_linear_speed_mps * dt
max_rotation_step = max_angular_speed_rad_s * dt
```

`100 Hz`、`0.2 m/s` 时约为 `2 mm/cycle`。

### 4.5 超时和停止语义

建议初始策略：

| 条件 | 行为 |
|---|---|
| frame age `<= 50 ms` | 正常更新目标 |
| `50 ms < age <= 100 ms` | 保持最后安全目标，不消费旧帧 |
| age `> 100 ms` | disengage，状态进入 `PAUSED` |
| 单手 tracking lost | 对应机械臂 hold；另一侧按配置决定继续或一并 pause |
| HMD invalid / XR session inactive | 双臂立即 hold 并 pause |
| mapper 非有限值或突跳 | 拒绝该帧；连续错误进入 `FAULT` |
| SDK 异常或失去控制权 | 停止发送，进入 `FAULT` |
| 本地 E-stop | 停止 teleop 输出；物理急停作为最终安全保障 |

在真机前必须单独验证“控制 PC 进程被杀死、网线断开、SDK 客户端退出”时 Orin 的实际行为。不能在未验证前假设 Orin 会自动安全 hold。

## 5. 实施任务

### M0：冻结接口边界和基线

**目标：** 防止后续错误地直接发布 Orin 内部 topic。

**工作：**

- 在机械臂 README/配置中记录第 2 节接口结论。
- 将 `endpoint_*` 标记为 feedback-only。
- 将 `joint_space_command` 标记为 internal-only。
- 记录基线：current/desired state `250 Hz`、ROS domain、RMW 和网卡白名单。
- 添加启动检查：发现 competing SDK/control client 时拒绝 engage。

**验收：**

- 项目代码中不存在向 `/astribot_arm_*/joint_space_command` 或 `/endpoint_*` 创建 publisher 的实现。
- dry-run 不初始化 `Astribot`，不会获取控制权。

### M1：重写 AstribotAdapter

**目标：** 形成唯一、可 mock、签名正确的机器人 API 边界。

**修改文件：**

- `stardust_wuji_quest3_pc_retargeting/arm_control/astribot_adapter.py`
- `tests/test_astribot_adapter.py`

**接口：**

```python
adapter = AstribotAdapter(freq_hz=100.0, enable_real=False)
adapter.initialize()
poses = adapter.get_desired_poses(frame="chassis")
adapter.send_targets({"left": left_target, "right": right_target})
adapter.close()
```

**工作：**

- 使用正确 import：`astribot_sdk.core.astribot_api.astribot_client.Astribot`。
- real 模式构造 `Astribot(freq=100.0)`。
- 从 SDK 对象解析 `arm_left_name`、`arm_right_name` 和 `chassis_frame_name`。
- `send_targets()` 每周期只调用一次 `set_cartesian_pose(names, poses, ...)`，双臂目标批量发送。
- 增加 desired/current pose 读取和格式校验。
- dry-run 保存最近目标和调用统计，不 import SDK。
- 不假设 `stop()` 存在；查明 SDK shutdown/control-rights API 后再实现退出语义。

**验收：**

- mock 测试验证 `names`、双层 pose list、xyzw、`filter`、`use_wbc=False` 参数完全正确。
- dry-run 单测不触发任何 ROS 或 SDK 初始化。

### M2：完成 Relative / Absolute 双模式 ArmMapper

**目标：** 实现可配置切换、无未授权跳变的 VR→chassis 相对和绝对位姿映射。

**修改文件：**

- `stardust_wuji_quest3_pc_retargeting/arm_control/arm_mapper.py`
- `stardust_wuji_quest3_pc_retargeting/arm_control/absolute_session_calibration.py`
- `stardust_wuji_quest3_pc_retargeting/conversion/pose_math.py`
- `stardust_wuji_quest3_pc_retargeting/protocol/messages.py`
- `stardust_wuji_quest3_pc_retargeting/protocol/validation.py`
- `quest3_web/src/webxr_session.js`
- `configs/arm/s1_quest3_default.yaml`
- `tests/test_arm_mapper.py`
- `tests/test_absolute_arm_mapper.py`
- `tests/test_absolute_session_calibration.py`
- `tests/test_mapping_mode_switch.py`

**工作：**

- `position_scale` 改成 `position_scale_xyz`。
- 加入每侧或公共 `R_BV`。
- 定义 `MappingMode.RELATIVE` 和 `MappingMode.ABSOLUTE`，禁止散落字符串分支。
- Relative：实现 `engage(side, vr_pose, robot_pose)`、`disengage(side)`、`recenter(...)`。
- Relative：实现平移和姿态相对增量。
- Absolute Session：实现倒计时、多帧采集、离群值处理、位置/旋转稳定性统计。
- Absolute Session：从平均 HMD yaw 构造并冻结 `T_V_O`，计算每侧 `T_O_C0`、`T_B_E0` 和 `T_C_E`。
- Absolute Session：将标定绑定 `xr_session_id`、`reference_space` 和 `reference_space_revision`，实现显式 invalidation。
- WebXR：监听 reference-space `reset`，发送 `reference_space_revision`；协议层验证并传入 supervisor。
- Absolute Session：标定只保存在当前运行会话；允许保存诊断报告，但禁止自动加载为下一 session 的有效标定。
- Absolute Session：第一版不实现 `absolute_nudge_offset`。
- 实现只允许 paused/idle 状态发生的模式切换事务；切换时重置 filter 历史。
- 增加首目标与机器人 desired pose 的 position/orientation jump gate。
- 真正应用 `rotation_scale`。
- 支持 `enable_orientation=False` 的位置专用测试模式。
- 检查 `R_BV` 正交性和行列式接近 `+1`。
- 为未来 head-relative 输入保留独立的 pose composition helper，但不混入当前 WebXR world pose 路径。

**验收：**

- engage 后零运动输入得到原机器人 pose。
- Relative 单轴输入符合配置的轴转换和每轴比例。
- 左右手独立 recenter。
- `q` 与 `-q` 输入不产生姿态跳变。
- Absolute Session 标定瞬间的首目标等于机器人标定基准目标。
- 标定后改变实时 HMD yaw、pitch、roll，不会改变相同手腕 pose 对应的目标。
- 样本不足、头/手/机器人移动超限、错误 session/reference space 和 workspace 外首目标均 fail closed。
- session end、session ID/recenter/reference space 改变会立即使标定失效。
- reference-space reset/revision 的协议兼容性有测试；旧客户端缺字段时 Absolute fail closed，Relative 可按版本策略兼容。
- `RUNNING` 状态拒绝模式切换；paused 切换不产生任何命令。

### M3：完成 ArmSafetyFilter

**目标：** 将安全限制从简单逐轴 clamp 升级为基于时间和旋转角的约束。

**修改文件：**

- `stardust_wuji_quest3_pc_retargeting/safety/arm_safety_filter.py`
- `stardust_wuji_quest3_pc_retargeting/safety/teleop_safety.py`
- `configs/arm/s1_quest3_default.yaml`
- `tests/test_arm_safety_filter.py`

**工作：**

- 左右臂使用独立 workspace。
- 用向量范数限制平移速度，不使用每轴 `np.clip` 代替速度限制。
- 用 quaternion angle/slerp 限制角速度。
- 加入输入突跳阈值、四元数校验、实际 `dt` 上下限。
- 定义 `ACTIVE/HOLD/PAUSED/FAULT` 的确定性输出。
- 将 freshness monitor 接入实际控制路径。

**验收：**

- 不同 `dt` 下的最大步长正确。
- workspace 外目标被安全裁剪或拒绝。
- tracking lost/stale 时不继续沿旧速度移动。
- NaN、Inf、零四元数不会进入 adapter。

### M4：实现 100 Hz latest-value ArmControlLoop

**目标：** 解耦 WebXR 帧率与机器人命令频率，禁止历史帧积压。

**新增/修改文件：**

- `stardust_wuji_quest3_pc_retargeting/runtime/latest_tracking.py`
- `stardust_wuji_quest3_pc_retargeting/runtime/arm_control_loop.py`
- `stardust_wuji_quest3_pc_retargeting/runtime/supervisor.py`
- `tests/test_arm_control_loop.py`

**工作：**

- lock-protected overwrite-only latest buffer。
- 使用 `time.monotonic_ns()` 记录本地接收和循环时间。
- 控制线程按 deadline 调度 `100 Hz`，不使用 WebXR client timestamp 判断网络 freshness。
- 读取一帧后按 seq 去重；没有新帧时执行 hold 策略。
- SDK 调用只发生在控制线程。
- 统计 loop period、mapper time、SDK call time、frame age、missed deadline。
- 日志降频，控制循环中禁止逐帧打印。

**验收：**

- 输入 30/60/90 Hz 时输出仍保持约 100 Hz，且只使用最新帧。
- 人工快速灌入 1000 帧后没有队列增长。
- mock SDK 阻塞时能记录 deadline miss 并进入可配置 fault。

### M5：状态机和 CLI 集成

**目标：** 形成默认 dry-run、显式真机 opt-in 的单一机械臂入口。

**修改文件：**

- `stardust_wuji_quest3_pc_retargeting/runtime/supervisor.py`
- `stardust_wuji_quest3_pc_retargeting/runtime/control_commands.py`
- `stardust_wuji_quest3_pc_retargeting/tools/run_control_pc_supervisor.py`
- `stardust_wuji_quest3_pc_retargeting/tools/run_control_pc_panel.py`
- `stardust_wuji_quest3_pc_retargeting/ui/control_panel.py`
- `example/teleop_quest3_bimanual_real.py`
- `configs/services/control_pc_default.yaml`
- `tests/test_supervisor_arm_integration.py`
- `tests/test_control_panel_commands.py`

**工作：**

- CLI 默认 `--dry-run`，只有 `--enable-real-arm` 才初始化 SDK。
- 支持 `--arm left|right|both`。
- 支持 `--mapping-mode relative|absolute` 和 `--absolute-calibration-report <path>`。
- 标定报告只用于诊断和审计，不得跨 session 恢复为有效标定。
- 支持 paused 状态下的 `mode relative|absolute` 命令，并执行模式切换 jump gate。
- 支持 `absolute-calibrate`、`invalidate-calibration` 和 `calibration-status`。
- 支持 `arm/calibrate/start/pause/resume/recenter/stop/estop/reset`；其中 `calibrate` 在 Relative 中执行 recenter，在 Absolute 中执行 session 多帧标定。
- `start` 前必须完成 fresh tracking 和机器人 pose 校验。
- Relative 的 `pause/resume` 必须重新 recenter，禁止沿用暂停前的 VR 零点。
- Absolute 的 `pause/resume` 保留当前 session 内冻结的标定，但必须重新检查 session ID、reference space/revision、freshness、workspace 和首目标跳变。
- 配置文件真正驱动 rate、scale、workspace、滤波、限速和 timeout。
- 保持 WujiHand 现有执行路径不变，只共享上层状态事件。
- 实现控制 PC 本地可点击面板，至少显示 mapping mode、WebXR session/reference space、HMD/左右手 tracking、机器人连接/控制权、标定状态和失败原因。
- 第一版面板使用 Python 标准库 Tkinter，模块懒加载；无桌面或缺少 Tkinter 时 supervisor/CLI 仍可完整运行，且不自动安装系统依赖。
- 面板至少提供 `Relative Recenter`、`Absolute 标定`、`Cancel Calibration`、`Start`、`Pause`、`Stop`、软件 `E-Stop` 按钮。
- 点击 `Absolute 标定` 后显示 `3/2/1` 倒计时、采样进度和质量检查结果。
- GUI 只向线程安全的 supervisor command API 投递命令，不直接调用 mapper、SDK 或 ROS；CLI 使用同一个 command API。
- GUI 主线程只渲染状态和投递命令；标定采集与机器人读取仍由 supervisor/control thread 执行，禁止 GUI 回调阻塞 100 Hz 控制循环。
- 软件 E-Stop 不能替代机器人物理急停，界面必须明确显示该提示。

**验收：**

- 未传 real flag 时无论配置内容如何都不初始化机器人。
- 状态转换非法时不会产生 arm command。
- 单侧 tracking lost 的策略与配置一致。
- PC GUI 按钮与 CLI 调用同一个 supervisor command API，按钮本身不包含控制逻辑。
- 点击 Absolute 标定成功后状态为 `ARMED/ABSOLUTE_CALIBRATED`，不会自动 `start`。
- GUI 关闭或崩溃不会绕过 supervisor watchdog；是否联动 pause 由配置明确控制并有测试。

### M6：离线和在线 dry-run 验证

**目标：** 在真机运动前消除坐标、跳变和时序错误。

**工作：**

- 用 `mock_webxr_sender` 跑持续 10 分钟。
- 保存 target pose、frame age、loop timing 和 safety state。
- 用历史 Quest/WebXR 数据或录制数据回放左右手轨迹。
- 绘制位置、速度、角速度、workspace clipping 次数和 stale 次数。
- 手动断开 WebSocket、暂停 XR session、制造单手 tracking lost。
- 用 mock Astribot 注入 5/10/20 ms 调用延迟。

**验收：**

- 无 NaN/Inf；四元数范数误差在容差内。
- engage/recenter 的第一个目标与机器人基准目标一致。
- Relative 和 Absolute 用同一输入轨迹时，分别满足各自数学定义和安全上限。
- 模式切换只在 pause 后生效，且候选跳变超限时保持 paused。
- 目标速度和角速度从不超过配置上限。
- stale 后 `100 ms` 内进入 pause。
- 控制循环无历史帧积压。

### M7：SDK 与 Orin 失效行为验证

**目标：** 在手臂运动前明确控制权、断连和进程退出行为。

**环境：** 机器人安全模式、机械臂不承载物体、操作员手持物理急停。

**工作：**

- 单独运行最小 SDK 程序，只读取 desired/current pose。
- 确认 SDK 客户端是否成功获得控制权，以及与现有 Web/VR 客户端的冲突表现。
- 发送“等于当前 desired pose”的静止命令，观察 desired/current 状态。
- 验证退出 SDK、`Ctrl+C`、进程 kill、PC 网线断开时 Orin 的行为。
- 记录控制权服务和 heartbeat 的状态变化。
- 明确软件 hold、SDK shutdown、物理急停的职责边界。

**验收门：**

- 在上述失败场景行为未记录清楚之前，禁止进行 VR 动态实机测试。

### M8：单臂 Relative 模式实机验收

**目标：** 以最低风险验证坐标轴、相对标定和 100 Hz 控制。

**初始配置：**

```yaml
arm: left
mapping_mode: relative
control_rate_hz: 100.0
position_scale_xyz: [0.3, 0.3, 0.3]
enable_orientation: false
max_linear_speed_mps: 0.10
fresh_timeout_sec: 0.05
disable_timeout_sec: 0.10
control_way: filter
use_wbc: false
```

**步骤：**

1. 机械臂回安全初始位，清空周围空间。
2. 启动 teleop dry-run，确认目标合理。
3. 启用左臂 real 模式，保持 VR 手静止后 engage。
4. 分别只移动 VR 前后、左右、上下轴，每次不超过 `2 cm` VR 位移。
5. 验证机器人分别沿预期 chassis `+X/+Y/+Z` 方向运动。
6. 验证 pause、recenter、tracking lost、WebSocket 断开。
7. 测量 SDK 调用延迟和 100 Hz 周期抖动。
8. 右臂重复相同步骤。

**验收：**

- engage 无可见跳变。
- 三轴方向全部正确。
- 最大速度不超过配置。
- tracking lost 和 pause 不产生继续移动。
- `set_cartesian_pose()` 调用时间应明显低于 `10 ms`；记录 p50/p95/p99。

### M9：单臂 Absolute 模式实机验收

**目标：** 在 Relative 单臂验收通过后，验证控制 PC 多帧 Absolute Session 标定、冻结 HMD yaw、重复性和安全切换。

**步骤：**

1. 进入 WebXR，确认 `xr_session_id`、reference space、HMD 和启用侧手腕均有效。
2. 机器人移动到安全 neutral pose；操作者站到地面标记位置、面向规定方向、头部水平并摆好 neutral 手姿。
3. 在控制 PC 点击 `Absolute 标定`，确认倒计时和采样期间机器人不运动。
4. 保持 `1.5 s`，检查采样数量、头/手/机器人稳定性和 desired/current 误差报告。
5. 标定成功后确认状态为 `ABSOLUTE_CALIBRATED/ARMED`，且没有自动发送运动命令。
6. 在 dry-run 中验证标定 pose 和多个小范围手腕检查点对应的机器人目标。
7. 点击 `Start`，只启用单臂、固定姿态，以 `0.05 m/s` 上限验证三个 session 内绝对位置检查点。
8. 保持手腕 pose 不变并转头、抬头、低头，确认机械臂目标不随实时 HMD 改变。
9. 暂停后切换 `relative -> absolute`，确认切换不立即发命令并执行首目标 jump gate。
10. 制造手/头移动过大的失败标定、错误 reference space、session end/recenter 和超限首目标，确认系统保持 paused 且标定失效。
11. 重新进入 WebXR session，确认旧标定不能复用，必须再次点击 Absolute 标定。
12. 右臂重复相同步骤。

**验收：**

- 点击标定只采样和计算，绝不启动机器人。
- 标定瞬间首目标等于机器人 neutral desired pose。
- 标定后实时头部转动不改变同一手腕 pose 的机器人目标。
- 同一 session 内重复到达三个检查点的目标误差和实际误差均被记录。
- Absolute 启动不会绕过 workspace 和速度限制。
- session/reference space/tracking origin 改变时 fail closed，禁止继续使用旧标定。
- 标定稳定性不达标时界面显示具体失败原因，且无法 Start。
- Relative/Absolute 来回切换必须经过 pause 和首目标 jump gate。

### M10：姿态与双臂实机验收

**目标：** 在位置链路稳定后逐步开启完整双臂控制。

**步骤：**

1. Relative 单臂启用姿态，`rotation_scale=0.5`、`max_angular_speed=0.5 rad/s`。
2. Absolute Session 单臂启用姿态，验证标定时计算的 `T_C_E` 工具对齐。
3. 两种模式分别验证 roll、pitch、yaw 和四元数连续性。
4. 开启左右双臂位置控制，一次调用批量发送两侧目标。
5. 验证单侧 tracking lost 策略。
6. 两种模式分别开启双臂姿态控制。
7. 最后与已完成的 WujiHand 控制共享 deadman 和 pause/recenter。

**验收：**

- 左右目标在同一控制周期发送。
- 双臂均满足 workspace、线速度和角速度限制。
- 任何一侧无效都不会向该侧发送未经验证的新目标。
- Relative 和 Absolute 模式均通过双臂位置与姿态验收。
- 完整系统能连续运行 30 分钟，无控制权丢失、队列积压或持续 deadline miss。

## 6. 配置草案

`configs/arm/s1_quest3_default.yaml` 最终建议结构：

```yaml
control_rate_hz: 100.0
control_way: filter
use_wbc: false
add_default_torso: true

mapping:
  mode: relative
  position_scale_xyz: [0.3, 0.3, 0.3]
  enable_orientation: false
  rotation_scale: 0.5
  absolute_calibration_report: logs/latest_absolute_session_calibration.yaml
  robot_from_vr_axes:
    - [0.0, 0.0, -1.0]
    - [-1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]

absolute_session_calibration:
  countdown_sec: 3.0
  sample_duration_sec: 1.5
  minimum_valid_samples: 60
  max_head_position_std_m: 0.010
  max_hand_position_std_m: 0.015
  max_head_yaw_std_rad: 0.050
  max_hand_rotation_std_rad: 0.090
  max_robot_position_std_m: 0.005
  max_robot_rotation_std_rad: 0.030
  max_robot_desired_current_error_m: 0.020
  invalidate_on_session_change: true
  invalidate_on_reference_space_change: true
  require_reference_space_revision: true
  invalidate_on_recenter: true
  allow_restore_from_report: false

filter:
  type: low_pass
  position_alpha: 0.25
  orientation_alpha: 0.25

safety:
  fresh_timeout_sec: 0.05
  disable_timeout_sec: 0.10
  max_linear_speed_mps: 0.10
  max_angular_speed_rad_s: 0.50
  max_input_position_jump_m: 0.10
  max_input_rotation_jump_rad: 0.80
  mode_switch_max_position_jump_m: 0.05
  mode_switch_max_rotation_jump_rad: 0.35
  minimum_dt_sec: 0.002
  maximum_dt_sec: 0.050

arms:
  left:
    enabled: true
    workspace_xyz_min: [0.25, 0.10, 0.75]
    workspace_xyz_max: [0.65, 0.65, 1.35]
  right:
    enabled: true
    workspace_xyz_min: [0.25, -0.65, 0.75]
    workspace_xyz_max: [0.65, -0.10, 1.35]
```

矩阵和 workspace 只是保守初值，必须通过单轴验证和 S1 实际可达空间修正后才能进入双臂测试。`mode: absolute` 时必须在当前 WebXR session 内完成控制 PC 多帧标定；`absolute_calibration_report` 仅用于诊断，不得自动恢复有效标定。

## 7. 测试命令

纯 Python 单元测试：

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_astribot_adapter.py \
  tests/test_arm_mapper.py \
  tests/test_absolute_arm_mapper.py \
  tests/test_absolute_session_calibration.py \
  tests/test_mapping_mode_switch.py \
  tests/test_webxr_protocol.py \
  tests/test_quest3_web_cli.py \
  tests/test_arm_safety_filter.py \
  tests/test_arm_control_loop.py \
  tests/test_supervisor_arm_integration.py \
  tests/test_control_panel_commands.py \
  -q
```

编译检查：

```bash
python3 -m compileall \
  stardust_wuji_quest3_pc_retargeting/arm_control \
  stardust_wuji_quest3_pc_retargeting/runtime \
  stardust_wuji_quest3_pc_retargeting/safety
```

实机前环境检查：

```bash
conda deactivate
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/env.sh
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/install/setup.sh

test "$ROS_DOMAIN_ID" = "25"
test "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp"

ros2 topic hz /astribot_arm_left/endpoint_current_states
ros2 topic hz /astribot_arm_right/endpoint_current_states
```

禁止项目直接发布内部 topic，可用静态检查：

```bash
rg -n 'create_publisher|Publisher' stardust_wuji_quest3_pc_retargeting \
  | grep -E 'joint_space_command|endpoint_(current|desired)_states' \
  && echo "ERROR: internal Astribot topic publisher found" \
  || echo "OK: no internal Astribot topic publisher"
```

## 8. Definition of Done

本计划完成必须同时满足：

- 控制 PC 使用官方 `Astribot.set_cartesian_pose()`，不直接发布内部关节 topic。
- Quest3 输入和机械臂输出通过 overwrite-only latest buffer 解耦。
- 控制循环稳定运行在约 `100 Hz`，具备周期和 SDK 调用耗时统计。
- engage/recenter 使用真实机器人 desired pose，不使用硬编码零位姿。
- Relative 和 Absolute 两种映射均已实现，可通过配置和 CLI 选择。
- Relative 的 VR→chassis 轴转换、每轴比例、姿态增量和 quaternion 连续性有自动测试。
- Absolute Session 的冻结 HMD yaw、`T_V_O`、左右 neutral anchor/`T_C_E`、多帧质量检查和失效关闭有自动测试。
- 控制 PC 提供 Absolute 标定按钮/等价 CLI；标定只采样计算，不自动 Start 或发送运动命令。
- Absolute 标定绑定当前 `xr_session_id`、reference space 和 `reference_space_revision`，跨 session、recenter 或 origin reset 后必须重新标定。
- 运行时切换只能在 pause/idle 完成，并经过首目标 jump gate，不产生切换瞬间命令。
- workspace、线速度、角速度、突跳、tracking lost 和 stale 均有安全约束。
- 默认 dry-run，真实机械臂必须显式 opt-in。
- Relative 单臂、Absolute 单臂、两种模式姿态和双臂按顺序完成实机验收。
- SDK 客户端退出、进程崩溃和网络断开时的 Orin 行为已经实际测试并记录。
- 双臂与已完成的 WujiHand 链路能共享 deadman、pause、recenter 和 estop 状态，连续运行 30 分钟无严重故障。

## 9. 第一批开发范围

立即开始的第一批工作只包含 M1～M4：

1. 修正并测试 `AstribotAdapter`。
2. 完成 `ArmMapper` 的 Relative/Absolute Session 双模式、clutch/recenter、冻结 HMD yaw、多帧标定和安全切换。
3. 完成 dt-aware `ArmSafetyFilter`。
4. 完成 overwrite-only buffer 和 100 Hz `ArmControlLoop`。

这批工作全部可在 mock/dry-run 环境完成，不触发机器人运动。完成并通过测试后，再执行 M7 的 SDK/Orin 安全行为确认，最后进入单臂实机阶段。
