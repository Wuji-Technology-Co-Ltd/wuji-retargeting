# Quest3 网页端裸手双臂与 WujiHand 遥操作设计

## 目标

在当前 `wuji-retargeting` 工程内设计并实现一套基于 Quest 3 浏览器网页端的裸手遥操作系统，用 Quest 3 自身手部追踪同时控制星尘 S1 双臂和 WujiHand 灵巧手。

本设计明确不使用 Unity App。Quest 3 端改为在 Meta Quest Browser 中打开一个 WebXR 网页。网页只采集 HMD、左右手追踪和浏览器会话状态，并通过 WebSocket 发送给控制 PC；机器人控制决策、安全状态机、retargeting、开始、暂停、结束、急停和硬件输出全部在控制 PC 侧执行。

系统分三端运行：

- Quest 3：运行 WebXR 网页端，采集 HMD pose、左右手裸手关节和 tracking 状态。
- 机器人 Orin：托管或中继网页端服务，并作为 Quest 3 与控制 PC 之间的数据中继。
- 控制 PC：运行 retargeting、双臂目标生成、统一状态机、安全检查、开始/暂停/结束/急停控制，是唯一主控制与安全决策端。

第一阶段必须能先用 mock WebXR 和 dry-run 验证完整链路，再接入 Quest Browser 真机追踪，最后逐步接入 WujiHand 真机和星尘 S1 真机。

## WebXR 可行性边界

网页端采用 WebXR Hand Input。根据 Meta 的 WebXR Hands 文档，Quest Browser 已支持 WebXR 体验中的手部追踪；W3C 的 WebXR Hand Input Module 定义了通过 `XRInputSource.hand` 访问手部骨架关节位姿的方式。

网页端的边界条件：

- 必须在 Quest Browser 的 immersive WebXR session 中采集 HMD 和手部关节。
- 页面需要由用户手动进入 XR session，浏览器不允许静默自动进入。
- WebXR 需要安全上下文，部署时使用 `https://` 或浏览器认可的本地可信 origin。
- 手部追踪 API 可用性要在启动时检测，不可假设所有浏览器环境都支持。
- 网页端不应在退出 XR session、浏览器后台或页面暂停后继续输出控制帧。
- 网页端只显示连接和 tracking 状态；除浏览器必须的 `Connect` 和 `Enter XR` 外，不提供遥操作开始、暂停、结束或急停控制。

## 当前工程结论

当前仓库已经具备三类关键基础：

- WujiHand retargeting 核心：`wuji_retargeting.Retargeter` 接收 MediaPipe 21 点手部关键点并输出 20 维 WujiHand 关节目标。
- WujiHand ROS2 控制链路：`example/teleop_retarget_to_wujihandros2.py` 已经能把手部输入 retarget 后发布到 `/<hand_name>/joint_commands`，由 `wujihandros2` driver 控制真实灵巧手。
- 星尘 SDK 控制链路：`/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master` 中 `Astribot` 支持 `set_cartesian_pose()` 实时笛卡尔控制和 `move_cartesian_pose()` 离线移动，可作为双臂第一版控制接口。

当前也存在一份早期 Quest3 三端方案文档和部分代码痕迹，但实现未完整落地：`example/input_devices/quest3_device.py` 依赖的 `stardust_wuji_quest3_pc_retargeting` 包目前不在工作区内。因此后续实现要先补齐协议、转换、安全和工具包，再向上构建双臂与双手统一遥操作。

## 推荐方案

采用“Quest Browser 网页采集，Orin Web/WS 中继，控制 PC 主控”的三端架构：

```text
Quest3 Browser WebXR Page
  HMD pose + left/right XRHand joints + tracking confidence
  web UI: connect, enter XR, tracking/status display only
  no robot policy, no retargeting
  no teleop lifecycle control
        |
        | HTTPS + WSS
        | over Orin IP or adb reverse tunnel
        v
Orin Web Gateway
  static web hosting
  WebSocket relay
  reconnect, frame counter, diagnostics
  no parsing for control, no safety decision
        |
        | Ethernet
        v
Control PC Teleop Supervisor
  protocol validation
  coordinate calibration
  arm target generation
  WujiHand retargeting
  state machine and safety
        |
        +--> Astribot SDK adapter: set_cartesian_pose()
        |
        +--> wujihandros2 adapter: /left_hand/right_hand/joint_commands
```

该方案保留控制 PC 作为唯一主控端，同时去掉 Unity 构建、Android APK 安装和原生应用权限管理。Quest 端只需要打开一个网页并进入 WebXR session，迭代和调试速度更快。

## 端侧职责

### Quest 3 网页端

Quest 3 端实现一个静态 WebXR 页面，目录暂定 `quest3_web/`。

职责：

- 在 Quest Browser 中打开控制页面。
- 检测 `navigator.xr`、`immersive-vr` 和 `hand-tracking` 支持。
- 通过用户按钮进入 XR session。
- 每个 XR animation frame 读取 HMD viewer pose。
- 遍历 `session.inputSources`，读取左右手 `XRInputSource.hand`。
- 对 WebXR 手部关节生成固定顺序的 joint pose 数组。
- 给每只手单独上报 `valid`、`joint_count`、`confidence` 或可推导的 tracking 状态。
- 上报浏览器和 XR session 状态，例如 connected、entered XR、visibility、tracking FPS。
- 通过 WebSocket 发送 `hello`、`tracking_frame`、`heartbeat` 和 `status` 消息。
- 显示简单状态：WebSocket 连接、XR session 状态、左右手 tracking、发送 FPS、控制 PC 回传状态和最近错误。

不负责：

- 不做 WebXR joints 到 MediaPipe 21 点的最终控制转换。
- 不做 WujiHand retargeting。
- 不计算 S1 双臂目标。
- 不直接连接 ROS2、Astribot SDK 或 WujiHand。
- 不实现安全状态机和急停策略。
- 不提供开始、暂停、结束、急停、clutch 或 deadman 等遥操作控制入口。

### Orin Web Gateway

Orin 是 Quest 网页端和控制 PC 之间的 Web gateway。

职责：

- 提供 `quest3_web/` 静态页面服务。
- 提供浏览器连接的 WebSocket endpoint。
- 连接控制 PC supervisor 的 WebSocket endpoint。
- 双向转发原始消息。
- 支持两种 Quest 访问方式：
  - Quest Browser 访问 Orin 的 HTTPS 地址。
  - Quest Browser 访问 `https://127.0.0.1:<port>`，由 `adb reverse` 转发到 Orin。
- 记录连接状态、转发帧数、最近错误、Quest 断连和控制 PC 断连。
- 断线后自动重连。

不负责：

- 不解析手部关节用于控制。
- 不做坐标转换。
- 不运行 retargeting。
- 不做安全判定。
- 不直接发布机器人命令。

### 控制 PC

控制 PC 是主控制节点。

职责：

- 接收 Orin 转发的 WebXR 追踪帧。
- 校验协议、timestamp、frame id、左右手有效性和数值有限性。
- 维护统一状态机：`IDLE`、`ARMED`、`RUNNING`、`PAUSED`、`ESTOP`、`FAULT`。
- 提供控制 PC 本地入口控制开始、暂停、继续、结束、急停和 clutch。第一版使用 CLI/键盘热键；后续可扩展 ROS2 service、脚踏开关或 PC 端控制面板。
- 将 WebXR 手部追踪转换成 Wuji retargeting 可用的 MediaPipe 21 点。
- 调用 `Retargeter` 生成左、右 WujiHand 20 维关节目标。
- 发布 `sensor_msgs/msg/JointState` 到 `/left_hand/joint_commands` 和 `/right_hand/joint_commands`。
- 将 WebXR 左右手腕或掌心相对位姿映射到 S1 左右臂末端目标。
- 通过 Astribot SDK `set_cartesian_pose()` 控制双臂。
- 在仿真和 dry-run 模式下输出完全相同的数据流，但不碰真实硬件。

## 网页协议

协议采用 JSON over WebSocket。第一版优先可读性和调试便利，后续如果帧率或延迟不足，再增加 MessagePack 或二进制编码。

所有消息包含：

```json
{
  "schema": "quest3_web_teleop.v1",
  "type": "tracking_frame",
  "seq": 1234,
  "client_time_sec": 123.456,
  "xr_session_id": "session-uuid"
}
```

`hello`：

```json
{
  "schema": "quest3_web_teleop.v1",
  "type": "hello",
  "client": "quest3_web",
  "version": "0.1.0",
  "webxr": {
    "immersive_vr": true,
    "hand_tracking": true
  }
}
```

`tracking_frame`：

```json
{
  "schema": "quest3_web_teleop.v1",
  "type": "tracking_frame",
  "seq": 1234,
  "client_time_sec": 123.456,
  "hmd": {
    "valid": true,
    "position": [0.0, 1.6, 0.0],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]
  },
  "hands": {
    "left": {
      "valid": true,
      "joint_names": ["wrist", "thumb-metacarpal", "thumb-phalanx-proximal"],
      "positions": [[0.0, 0.0, 0.0]],
      "orientations_xyzw": [[0.0, 0.0, 0.0, 1.0]]
    },
    "right": {
      "valid": false,
      "joint_names": [],
      "positions": [],
      "orientations_xyzw": []
    }
  },
  "session": {
    "active": true,
    "visibility": "visible",
    "reference_space": "local-floor"
  }
}
```

网页端不发送开始、暂停、结束、急停、deadman 或 clutch 请求。控制 PC 可以向网页端回传 `control_state` 消息用于状态显示，但网页端回传显示不影响真实控制状态。

## 控制映射

### WujiHand 映射

WujiHand 路径复用现有 `Retargeter`：

```text
WebXR hand joints
  -> WebXR-to-MP21 converter
  -> Retargeter.from_yaml(...)
  -> qpos_20
  -> HandSafetyFilter
  -> /left_hand|right_hand/joint_commands
```

第一版保留现有每手独立 retargeting 方式，不在同一个优化器里耦合左右手。左右手 tracking valid 独立判断：

- 左手无效时只停止或保持左手输出。
- 右手无效时只停止或保持右手输出。
- 整帧 stale 时双手和双臂都进入安全 hold 或 pause。

### S1 双臂映射

双臂不直接使用 WebXR 世界坐标的绝对位姿。进入 `ARMED` 或执行 clutch 时记录：

- 当前机器人左、右末端位姿。
- 当前 WebXR 左、右手腕或掌心位姿。
- 当前 HMD yaw 或用户朝向，用于定义操作者局部坐标。

运行时使用相对映射：

```text
webxr_delta = current_webxr_hand_pose - clutch_webxr_hand_pose
robot_target = clutch_robot_eef_pose + scale_and_filter(webxr_delta)
```

姿态采用相对旋转：

```text
robot_target_rot = clutch_robot_rot * mapped_delta_rot
```

第一版只控制 S1 左右臂，不控制底盘、头部和 torso。torso 使用 SDK 默认或当前位姿，避免引入全身控制复杂度。后续如需要扩大工作空间，再设计 torso 或 chassis 的二阶段控制。

### 坐标系

控制 PC 维护一个显式标定对象：

- `webxr_reference_space`：WebXR 页面使用的 `local-floor` 或 `local` reference space。
- `operator_frame`：以 HMD yaw 或启动时手部姿态定义的操作者坐标。
- `robot_base_frame`：S1 机器人 base/world 坐标。
- `left_eef_frame` / `right_eef_frame`：星尘 SDK 中左右臂末端坐标。

第一版标定采用“启动相对标定”，不要求外参精确测量。后续可加入 AprilTag、手动点选或固定外参 YAML。

## 操作控制入口

遥操作控制入口全部在控制 PC。Quest 3 网页端只负责追踪采集和状态提示，因为在头显内操作网页按钮会干扰遥操作姿态，也不适合作为可靠安全入口。

控制 PC 第一版入口：

- `space` 或 `start` 命令：从 `ARMED` 进入 `RUNNING`。
- `p` 或 `pause` 命令：进入 `PAUSED`。
- `r` 或 `resume` 命令：从 `PAUSED` 回到 `RUNNING`。
- `c` 或 `calibrate` 命令：采集当前机器人末端和 WebXR 手部位姿，更新相对标定。
- `h` 或 `hold` 命令：保持当前双臂和双手目标。
- `q` 或 `stop` 命令：结束遥操作，回到 `IDLE` 或安全停止。
- `esc`、`estop` 命令或独立急停输入：立即进入 `ESTOP`。

Quest 网页端页面元素：

- `Connect`：连接 Orin WebSocket，这是浏览器连接所需，不代表遥操作开始。
- `Enter XR`：用户手势触发进入 WebXR immersive session，这是浏览器权限所需，不代表遥操作开始。
- 状态面板：连接状态、XR session、左右手 tracking、FPS、控制 PC 状态回传。

网页端不得提供 `Start`、`Pause`、`Resume`、`Calibrate`、`E-Stop` 等遥操作按钮。后续如果要做图形控制面板，应放在控制 PC 上，而不是 Quest Browser 内。

## 安全机制

基础安全必须在控制 PC 执行。

状态机：

- `IDLE`：接收数据但不输出硬件命令。
- `ARMED`：完成连接检查和相对标定，等待操作者开始。
- `RUNNING`：持续输出双臂和双手命令。
- `PAUSED`：保持当前目标或安全打开灵巧手，不更新新目标。
- `ESTOP`：立即停止输出，禁用 WujiHand，调用星尘停止接口或停止发送新命令。
- `FAULT`：出现协议错误、严重超时、硬件错误或安全检查失败，等待人工复位。

检查项：

- 帧超时：超过 `fresh_timeout_sec` 进入 hold，超过 `disable_timeout_sec` 进入 pause 或 fault。
- XR session 状态：退出 XR session 后立即停止使用网页端追踪帧。
- 数值检查：所有位置、旋转、关节目标必须为有限值。
- 双臂工作空间限制：限制左右末端 xyz 范围、单帧位移、线速度、角速度。
- 姿态限制：限制腕部相对旋转范围和单帧旋转跳变。
- WujiHand 关节限制：限制 qpos 上下界和单帧关节跳变。
- tracking valid：单手失效只影响对应手和对应臂；HMD 或全局时间异常影响全系统。
- deadman/clutch：`RUNNING` 期间必须满足控制 PC 判定的 deadman 条件。第一版 deadman 和 clutch 来自控制 PC 键盘或外接输入，不来自 Quest 网页。释放后进入 hold 或 pause。
- 急停：控制 PC 本地键盘/CLI/ROS2 service 或独立急停输入触发，网页端不作为急停输入。

真机默认策略：

- 默认 dry-run，不主动连接真实硬件。
- 需要显式 `--enable-real-hand` 才发布 WujiHand 真机命令。
- 需要显式 `--enable-real-arm` 才调用 Astribot SDK 真机控制。
- Quest3 网页真机输入可以先连接，但硬件输出默认关闭。

## 文件组织

建议在 `wuji-retargeting` 内新增和修改以下文件。

```text
wuji-retargeting/
  stardust_wuji_quest3_pc_retargeting/
    protocol/
      messages.py
      validation.py
      json_codec.py
    conversion/
      hand_joint_names.py
      webxr_to_mp21.py
      pose_math.py
    web_gateway/
      static_server.py
      websocket_relay.py
      adb_reverse.py
      gateway_status.py
    hand_control/
      wujihand_ros2_publisher.py
      retarget_pipeline.py
    arm_control/
      astribot_adapter.py
      arm_mapper.py
      workspace_limits.py
    safety/
      state_machine.py
      teleop_safety.py
      hand_safety_filter.py
      arm_safety_filter.py
    runtime/
      supervisor.py
      config.py
      telemetry.py
    sim/
      mock_webxr_sender.py
      dryrun_robot.py
    tools/
      run_orin_web_gateway.py
      run_control_pc_supervisor.py

  quest3_web/
    README.md
    index.html
    src/
      app.js
      webxr_session.js
      hand_frame_serializer.js
      websocket_client.js
      status_panel.js
      session_state.js
    styles/
      main.css

  example/
    input_devices/
      quest3_device.py
    teleop_quest3_bimanual_sim.py
    teleop_quest3_bimanual_real.py

  configs/
    quest3_web/
      webxr_hand_mapping_left.yaml
      webxr_hand_mapping_right.yaml
    retargeting/
      adaptive_analytical_quest3_left.yaml
      adaptive_analytical_quest3_right.yaml
    arm/
      s1_quest3_default.yaml
    safety/
      quest3_teleop_default.yaml
      wh110_left.yaml
      wh110_right.yaml
    services/
      orin_web_gateway_default.yaml
      control_pc_default.yaml

  tests/
    test_webxr_protocol.py
    test_webxr_to_mp21.py
    test_arm_mapper.py
    test_state_machine.py
    test_hand_safety_filter.py
    test_arm_safety_filter.py
    test_supervisor_dryrun.py
```

## 配置

核心配置文件 `configs/services/control_pc_default.yaml` 应包含：

```yaml
quest_web:
  listen_host: 0.0.0.0
  listen_port: 9001
  stale_timeout_sec: 0.2
  require_xr_session: true

state:
  start_in: IDLE
  require_deadman: true
  control_source: pc_keyboard

hands:
  left:
    enabled: true
    hand_name: left_hand
    retarget_config: configs/retargeting/adaptive_analytical_quest3_left.yaml
    mapping_config: configs/quest3_web/webxr_hand_mapping_left.yaml
    safety_config: configs/safety/wh110_left.yaml
  right:
    enabled: true
    hand_name: right_hand
    retarget_config: configs/retargeting/adaptive_analytical_quest3_right.yaml
    mapping_config: configs/quest3_web/webxr_hand_mapping_right.yaml
    safety_config: configs/safety/wh110_right.yaml

arms:
  enabled: true
  sdk_root: /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master
  control_rate_hz: 100.0
  position_scale: 1.0
  rotation_scale: 1.0
  workspace_config: configs/arm/s1_quest3_default.yaml

hardware:
  enable_real_hand: false
  enable_real_arm: false
```

Orin gateway 配置 `configs/services/orin_web_gateway_default.yaml` 应包含：

```yaml
web:
  host: 0.0.0.0
  https_port: 8443
  static_root: quest3_web
  cert_file: configs/certs/orin_gateway.crt
  key_file: configs/certs/orin_gateway.key

quest_ws:
  listen_host: 0.0.0.0
  listen_port: 9001

control_pc:
  url: ws://192.168.0.20:9001

adb_reverse:
  enabled: true
  device_port: 8443
  host_port: 8443
```

实际实现时配置加载器要解析相对路径为 repo root 下的绝对路径。

## CLI 工作流

### 阶段 1：纯 mock 仿真

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting
python -m stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender \
  --target ws://127.0.0.1:9001

python example/teleop_quest3_bimanual_sim.py \
  --config configs/services/control_pc_default.yaml
```

该阶段验证协议、左右手转换、WujiHand retargeting、双臂目标生成、状态机和日志，不连接 Quest Browser 和硬件。

### 阶段 2：Quest Browser 真输入，硬件 dry-run

控制 PC：

```bash
python -m stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --config configs/services/control_pc_default.yaml \
  --dry-run
```

Orin：

```bash
python -m stardust_wuji_quest3_pc_retargeting.tools.run_orin_web_gateway \
  --config configs/services/orin_web_gateway_default.yaml
```

Quest 3：

```text
Open Quest Browser -> https://<ORIN_IP>:8443
or
Open Quest Browser -> https://127.0.0.1:8443 when adb reverse is enabled
```

该阶段验证 Quest Browser WebXR 手部追踪、Orin gateway、延迟、掉线重连、左右手 valid 和坐标标定。

### 阶段 3：接入 WujiHand 真机

先启动现有左右手 driver：

```bash
/home/zxc/Desktop/wuji/wuji-teleop/run_left_hand_driver.sh
/home/zxc/Desktop/wuji/wuji-teleop/run_right_hand_driver.sh
```

再启动控制 PC supervisor：

```bash
python -m stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --config configs/services/control_pc_default.yaml \
  --enable-real-hand
```

### 阶段 4：接入星尘 S1 真机

先确认星尘 SDK 环境和机器人回初始位：

```bash
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/env.sh
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/install/setup.sh
```

再显式启用真机双臂：

```bash
python -m stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --config configs/services/control_pc_default.yaml \
  --enable-real-hand \
  --enable-real-arm
```

## 开发步骤

1. 补齐 `stardust_wuji_quest3_pc_retargeting.protocol`，实现 WebXR 消息 dataclass、JSON 编解码和严格校验。
2. 实现 `quest3_web/` 静态网页骨架，包含 WebSocket 连接、WebXR 支持检测、进入 XR、状态面板。
3. 实现 WebXR 手部关节采集和 `tracking_frame` 序列化。
4. 补齐 WebXR 手部关节到 MP21 的转换模块，使 `example/input_devices/quest3_device.py` 或新的 WebXR input device 可测试运行。
5. 实现 `sim.mock_webxr_sender` 和 `runtime.supervisor` 的 dry-run 主循环，先不引入 ROS2 和 Astribot SDK 硬依赖。
6. 实现 Orin `web_gateway`：HTTPS 静态服务、Quest WebSocket endpoint、到控制 PC 的 WebSocket relay、可选 adb reverse。
7. 实现 WujiHand ROS2 publisher，复用 `teleop_retarget_to_wujihandros2.py` 的 JointState 发布逻辑。
8. 实现 `HandSafetyFilter` 与统一状态机，并让真实手默认需要 `--enable-real-hand`。
9. 实现 `arm_mapper`：相对位姿 clutch、位置/旋转缩放、低通滤波、工作空间限制。
10. 实现 `AstribotAdapter`：封装 SDK import、环境检查、dry-run、`set_cartesian_pose()` 输出和 stop 行为。
11. 实现 `ArmSafetyFilter`：末端 xyz 限制、速度限制、姿态跳变限制、stale hold。
12. 增加 `teleop_quest3_bimanual_sim.py`，验证 mock WebXR 到双手和双臂 dry-run 输出。
13. 增加 `teleop_quest3_bimanual_real.py` 或统一 CLI，按显式 flag 接入 WujiHand 和 S1 真机。
14. 编写分层测试：协议、转换、状态机、安全、arm mapper、supervisor dry-run。
15. 在真机前执行 checklist：HTTPS/WebXR 支持、时间同步、ROS_DOMAIN_ID、FastDDS 白名单、WujiHand driver、S1 初始位、急停验证。

## 测试策略


- 协议合法帧、缺字段、错误关节数量、NaN/Inf、timestamp 回退。
- WebXR joints 到 MP21 的 shape、左右手独立 valid、wrist-relative 输出。
- 状态机的开始、暂停、继续、结束、急停、fault reset。
- WujiHand qpos 限幅、单帧跳变、deadman release、stale hold。
- 双臂相对位姿映射、位置缩放、旋转缩放、workspace clipping。

网页端测试：

- 非 Quest 浏览器打开时显示不支持 WebXR 或不支持 hand-tracking。
- Quest Browser 中能进入 XR session。
- 左右手离开视野时 valid 独立变化。
- WebSocket 断开时页面显示 disconnected 并停止发送控制帧。

集成测试：

- mock WebXR sender -> supervisor dry-run。
- Quest Browser -> Orin web gateway -> Control PC supervisor dry-run。
- WebXR input -> Retargeter -> qpos_20。
- Supervisor dry-run 同时生成 `left_hand`、`right_hand`、`left_arm`、`right_arm` 输出。
- Orin gateway 断线重连。

人工验收：

- Quest3 左右手在不同遮挡情况下 valid 独立变化。
- 退出 XR session 后控制 PC 停止使用追踪帧。
- 释放 deadman 后双臂不继续移动，WujiHand hold 或 safe-open。
- 急停在控制 PC 本地触发后停止所有输出。
- 真机模式必须显式 flag 才能进入。
- 星尘双臂相对移动方向与操作者直觉一致，且不会因启动姿态造成跳变。

## 里程碑

### M1：协议和 mock 链路

完成 WebXR 协议包、mock sender、WebXR-to-MP21 转换模块和 supervisor dry-run。验收标准是无 Quest3、无硬件时能持续输出双手 qpos 和双臂目标日志。

### M2：网页端真输入链路

完成 `quest3_web/` WebXR 页面和 Orin web gateway。验收标准是 Quest Browser 裸手追踪帧能稳定到达控制 PC，左右手 valid、延迟和帧率可观测。

### M3：WujiHand 真机

完成 Quest Browser 裸手到左右 WujiHand 的 ROS2 topic 控制。验收标准是 `--enable-real-hand` 下能控制灵巧手，deadman、stale、限幅和急停有效。

### M4：S1 双臂 dry-run 到真机

完成双臂相对位姿映射和 Astribot SDK adapter。验收标准是先 dry-run，再 `--enable-real-arm` 控制 S1 双臂小范围移动，workspace、速度和姿态限制有效。

### M5：统一双臂双手遥操作

完成双臂与双手统一 supervisor、状态控制、日志和启动脚本。验收标准是操作者戴 Quest3 后不使用手柄、不使用 Wuji 手套、不安装 Unity APK，只打开网页即可在控制 PC 管理完整遥操作生命周期。

## 非目标

第一版不做以下内容：

- 不使用 Unity、Unreal 或 Android 原生 App。
- 不复刻或接管星尘闭源 Quest 手柄控制方案。
- 不让 Quest 3 直接发布 ROS2 topic。
- 不让 Orin 执行机器人控制策略。
- 不控制底盘、头部和 torso。
- 不实现复杂三维网页 UI；网页端先以进入 XR、连接和状态显示为主。
- 不追求跨用户自动精确标定；第一版使用启动相对标定和 YAML 参数。

## 风险与缓解

- WebXR hand-tracking 支持差异：启动时检测 `navigator.xr` 和 hand input，不支持时阻止进入遥操作并显示原因。
- HTTPS 和证书问题：Orin gateway 提供固定 HTTPS 服务；开发阶段支持 adb reverse 到本地可信 origin；部署文档记录证书安装或信任流程。
- Quest Browser 页面被挂起：控制 PC 通过 heartbeat 和 frame timeout 进入 hold/pause。
- Quest3 裸手追踪遮挡：通过单手 valid、stale hold、deadman 和 pause 降低风险。
- Quest/WebXR 坐标与机器人坐标不一致：第一版采用相对 clutch 映射，不依赖全局绝对坐标。
- 星尘 SDK 控制频率和网络抖动：控制 PC 做低通、限速，并使用 SDK 示例中的 `Rate` 控制循环。
- WujiHand qpos 跳变：沿用关节上下界，增加单帧跳变限制和 safe-open/hold 策略。
- 真机误触发：真实硬件默认关闭，必须显式 `--enable-real-hand` 和 `--enable-real-arm`。
- 现有代码不完整：先补齐缺失包和测试，再接入硬件。
