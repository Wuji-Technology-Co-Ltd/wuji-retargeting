# Quest3 双手 dry-run 与 ROS2 Hand Bridge

本阶段只验证正式 supervisor 的双手数据管线和 ROS2 录制接口，不连接或控制真实 WujiHand。

## 数据流

```text
Quest WebXR tracking_frame
  -> ControlPCSupervisor
     -> left/right WebXRToMP21Converter
     -> left/right RetargetPipeline (dry-run fallback by default)
     -> left/right HandSafetyFilter
     -> timestamped UDP HandBridgeFrame
  -> standalone ROS2 Hand Bridge
     -> /teleop/quest3/<side>/keypoints
     -> /teleop/hand/<side>/target_raw
     -> /teleop/hand/<side>/target_safe
     -> /teleop/hand/bridge_state
```

The bridge does not publish `/left_hand/joint_commands` or
`/right_hand/joint_commands` unless the separate
`--publish-driver-commands` flag is supplied. Do not use that flag during this
dry-run stage.

## Recommended: one-command Control-PC startup

Regular operation should use the shared Control-PC lifecycle. The launcher
loads the unified environment once, starts the recording-only Bridge, keeps the
interactive Supervisor in the foreground, and stops both processes together:

```bash
/home/zxc/Desktop/wuji/wuji-teleop/start_quest3_astribot_teleop.sh
```

仓库内部等价入口是 `./scripts/run_full_arm_hand_teleop.sh`。

This is the real dual-arm fixed-anchor profile with real dual-hand Retargeters,
but WujiHand hardware commands remain disabled. The Orin Web Gateway still runs
separately on Orin. No rosbag is recorded unless the operator explicitly starts
`ros2 bag record` in another terminal.

For a no-motion check:

```bash
/home/zxc/Desktop/wuji/wuji-teleop/start_quest3_astribot_teleop.sh --dry-run
```

Stop normally with `P`, optionally use `R`, then press `Ctrl+C` in the launcher
terminal. The final line must be:

```text
Supervisor and Hand Bridge stopped.
```

## Manual split startup

Use the following commands only when Bridge and Supervisor logs must be
isolated for development or troubleshooting.

### Start the recording-only ROS2 bridge

Use a ROS2 environment that provides `rclpy`, `sensor_msgs`, `geometry_msgs`,
and `std_msgs`:

```bash
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/env.sh
source /home/zxc/Desktop/wuji/wuji-teleop/wujihandros2/install/setup.bash

cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting

python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_wujihand_ros2_bridge \
  --listen-host 127.0.0.1 \
  --listen-port 9011
```

Startup output must say:

```text
driver_commands=DISABLED
```

### Start Supervisor with dual-hand dry-run

Arm dry-run plus hand dry-run:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --enable-hand-dryrun \
  --hand-bridge-udp-host 127.0.0.1 \
  --hand-bridge-udp-port 9011 \
  --interactive
```

Validated real arms plus hand dry-run/recording interface:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --run-m8-fixed-anchor-real \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --m8-position-scale 2.0 \
  --m8-rotation-scale 1.0 \
  --enable-hand-dryrun \
  --hand-bridge-udp-host 127.0.0.1 \
  --hand-bridge-udp-port 9011
```

The existing `--run-m8-fixed-anchor-real` behavior is unchanged when the hand
flags are omitted.

## Supervisor status

After Quest tracking is active, run `S` and inspect:

```text
hand_pipeline_enabled
hand_retarget_real
hand_command_sink
hand_output_frames
hand_output_last_seq
hand_command_enabled.left/right
hand_safety_state.left/right
hand_output_last_error
hand_process_p95_ms
```

Expected behavior:

- Before `E`: safe targets are produced for recording, but command enabled is false.
- During `RUNNING`: valid hands have command enabled true.
- After `P`: safe qpos holds the previous target and command enabled becomes false.
- Invalid or stale tracking: the affected hand is HOLD/DISABLED and does not emit a driver command.

## ROS2 recording

```bash
ros2 bag record \
  /teleop/quest3/left/keypoints \
  /teleop/quest3/right/keypoints \
  /teleop/hand/left/target_raw \
  /teleop/hand/left/target_safe \
  /teleop/hand/right/target_raw \
  /teleop/hand/right/target_safe \
  /teleop/hand/bridge_state
```

`bridge_state` retains source `seq`, `xr_session_id`, teleop state, command
enable state, stale state, and transport counters. ROS message headers use the
bridge receive clock; the original Quest time and Control PC receive time are
retained in the UDP bridge frame for future typed-message expansion.

## Retarget modes

Default `--enable-hand-dryrun` uses the deterministic fallback retargeter. This
keeps the `/usr/bin/python3` Astribot environment free of `nlopt`/`pin`
requirements and is sufficient to validate synchronization, safety state,
bridge transport, and rosbag recording.

`--hand-retarget-real` loads the configured Wuji Retargeter and therefore
requires a Python environment containing the retargeting dependencies. It does
not by itself enable real hand hardware.

## Unified Astribot and real-Retargeter environment

The `wuji-ros2` Conda environment contains the real Wuji Retargeter. Astribot's
environment prepends its own Pinocchio 3.7 bindings, while the Conda packages
provide the missing eigenpy/urdfdom shared libraries. Always use the repository
launcher so the Python and native-library ordering remains deterministic:

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting

./scripts/bootstrap_unified_teleop_env.sh

./scripts/run_in_unified_teleop_env.sh \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.check_unified_retarget_env \
  --json-output logs/hand_retarget/unified_env_check.json
```

The report must show `passed: true` for both hands. This is an offline smoke
test: it loads the Astribot SDK factory and both real Retargeters and evaluates
synthetic open/closed MP21 poses without constructing an Astribot client or
publishing hand driver commands.

Start the supervisor with real retargeting but recording-only hand output:

```bash
./scripts/run_in_unified_teleop_env.sh \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --enable-hand-dryrun \
  --hand-retarget-real \
  --hand-bridge-udp-host 127.0.0.1 \
  --hand-bridge-udp-port 9011 \
  --interactive
```

This command does not enable real arms or real hands. For the already validated
real-arm profile plus recording-only hands, add `--hand-retarget-real` and the
hand bridge options to `--run-m8-fixed-anchor-real`, but still do not use
`--publish-driver-commands`.

After WebXR tracking starts, `S` must report:

```text
hand_retarget_real: true
hand_output_last_error: ""
hand_output_frames: increasing
```

Before `E`, both hand safety states must be `HOLD`; after `E`, they must be
`ACTIVE`; after `P` or `R`, they must return to `HOLD`.

For an interactive left/right routing and open/close response check, leave the
bridge in recording-only mode, press `E` in the supervisor so both hand safety
states are `ACTIVE`, and run in a third terminal:

```bash
./scripts/run_in_unified_teleop_env.sh \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.inspect_hand_retarget_mapping \
  --json-output logs/hand_retarget/live_mapping_check.json
```

The inspector aborts if the bridge reports driver commands enabled or if a
publisher exists on `/left_hand/joint_commands` or
`/right_hand/joint_commands`. Follow its three prompts: both hands open, left
closed/right open, then left open/right closed. The JSON report records the
20-joint median targets, per-finger response, inactive-hand movement, and
signed flexion deltas.

## Safety boundary

- `--enable-real-hand` remains rejected by the M8 arm supervisor.
- The ROS2 bridge defaults to recording-only topics.
- WujiHand drivers do not need to be running for this stage.
- Do not pass `--publish-driver-commands` until the real-hand authorization,
  watchdog, calibration, and staged hardware validation are complete.
