# wuji-retargeting

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-retargeting)](https://github.com/wuji-technology/wuji-retargeting/releases)

Hand pose retargeting system for Wuji Hand. High-precision retargeting based on adaptive analytical and key-vector optimization, with Wuji Glove as the recommended live input path. Apple Vision Pro, video files, Intel RealSense, ZED cameras, and MANUS-style external pipelines can also be used as input sources.

https://github.com/user-attachments/assets/72116289-7a33-4a6b-83ca-fb4d9aaece0d

**Get started with [Quick Start](#quick-start). For detailed documentation, please refer to [Retargeting Tutorial](https://docs.wuji.tech/en/wuji-hand/latest/retargeting) on Wuji Docs Center.**

## Quest3 + Astribot 一键遥操启动

当前 `quest3-astribot-teleop` 集成的默认操作方式是：Orin 单独运行 Web Gateway，控制 PC 用一条命令同时启动 ROS2 Hand Bridge 和真实双臂 Supervisor。

> 警告：默认一键命令会控制真实双机械臂，使用 2.0 m/s 线速度、3.0 rad/s 角速度和 2 m workspace。启动前必须确认物理急停可立即触达，机器人周围无人且无障碍物。

### 1. Orin：启动 Web Gateway

```bash
python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_orin_web_gateway \
  --web-host 0.0.0.0 --web-port 8443 \
  --ws-host 0.0.0.0 --ws-port 9002 \
  --control-pc-url ws://192.168.0.100:9001
```

### 2. 控制 PC：一键启动 Bridge 与 Supervisor

从 `wuji-teleop` 根目录直接启动：

```bash
/home/zxc/Desktop/wuji/wuji-teleop/start_quest3_astribot_teleop.sh
```

在仓库内部启动等价命令：

```bash
./scripts/run_full_arm_hand_teleop.sh
```

该脚本会：

- 加载统一 Astribot / ROS2 / Retargeter Python 环境；
- 后台启动 recording-only ROS2 Hand Bridge；
- 前台启动交互式真实双臂固定锚点 Supervisor；
- 启用左右手真实 Retargeter；
- 在退出时同时清理 Supervisor 和 Bridge。

当前不会启动真实 WujiHand 驱动，也不会发布
`/left_hand/joint_commands` 或 `/right_hand/joint_commands`。不主动运行
`ros2 bag record` 就不会录制 rosbag。

Quest 进入 WebXR 后，在 `teleop>` 中执行：

```text
S
E
```

每次 `E` 只输入一次，保持头部和双腕稳定约两秒，等待自动进入
`RUNNING`。日常命令为 `S`、`E`、`P`、`R`。

正常停止时先执行 `P`，需要时执行 `R`，然后在一键启动终端按
`Ctrl+C`。结束信息应为：

```text
Supervisor and Hand Bridge stopped.
```

不动真实机械臂的检查模式：

```bash
/home/zxc/Desktop/wuji/wuji-teleop/start_quest3_astribot_teleop.sh --dry-run
```

本机完整操作手册位于
`/home/zxc/Desktop/wuji/wuji-teleop/naked_teleop.md`，Bridge 与 Retargeter 说明见
[Quest3 双手 dry-run 与 ROS2 Hand Bridge](docs/quest3-wujihand-ros2-dryrun.md)。

## Repository Structure

```text
├── wuji_retargeting/                 // Core package: retargeter interface, optimizers, kinematics, coordinate transforms
│   ├── opt/                          // Optimizer implementations: adaptive analytical and key-vector
│   ├── viz/                          // Visualization tools for parameter tuning
│   └── wuji-description/             // URDF and mesh submodule for Wuji Hand
├── example/                          // Demonstration scripts for simulation and hardware control
│   ├── input_devices/                // Input device modules (Vision Pro, MediaPipe replay, video, RealSense, ZED, Wuji Glove)
│   ├── config/                       // YAML configuration files
│   ├── data/                         // Sample recording data
│   └── utils/                        // Helper utilities
├── requirements.txt                  // Python dependencies
└── README.md
```

## Quick Start

### Installation

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wuji-retargeting.git
cd wuji-retargeting
pip install -r requirements.txt
pip install -e .
```

### Running

Wuji Glove is the recommended live input path for current development and demos.

```bash
cd example

# Recommended: Wuji Glove live input
python teleop_sim.py --input wuji_glove --hand right --glove-sn <YOUR_SN>
python teleop_real.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Replay recording (default: adaptive analytical optimizer)
python teleop_sim.py --play data/avp1.pkl --hand left

# Key-vector optimizer
python teleop_sim.py --play data/avp1.pkl --hand right --config config/vector/vector_avp.yaml
```

### Other Input Sources

In addition to the recommended Wuji Glove path, you can use MP4 video files, Intel RealSense cameras, STEREOLABS ZED cameras, Apple Vision Pro, or custom/MANUS-style external pipelines as input sources.

```bash
# MP4 video input
pip install -e ".[video]"
mjpython teleop_sim.py --video path/to/hand_video.mp4 --hand right

# Intel RealSense camera
pip install -e ".[realsense]"
mjpython teleop_sim.py --realsense --hand right

# ZED camera
pip install -e ".[zed]"
mjpython teleop_sim.py --zed --hand right
```

Use `--show-video` to display the camera/video feed with MediaPipe landmarks overlay for input verification.

### Parameter Tuning Visualization Tool

An interactive tuning tool that displays **three skeleton layers** simultaneously to help you understand and adjust retargeting parameters:

| Color | Layer | Description |
|-------|-------|-------------|
| Orange | Input | Raw MediaPipe keypoints from hand tracking |
| Cyan | Scaled Target | Keypoints after `segment_scaling` applied |
| White | Robot FK | Actual robot joint positions from forward kinematics |

**Quick Start:**

```bash
cd example

# Launch tuning viewer with recording data
mjpython tuning_tool.py --play data/avp1.pkl --hand left

# Recommended: Wuji Glove live tuning
mjpython tuning_tool.py --wuji-glove --hand right --glove-sn <YOUR_SN>

# Other live camera tuning modes
mjpython tuning_tool.py --realsense --hand right
mjpython tuning_tool.py --zed --hand right
```

**Workflow:**
1. Start the viewer with a recording, video, or live input
2. Open the retarget config YAML in your editor (e.g., `config/adaptive_analytical_avp.yaml`)
3. Modify parameters (e.g., `segment_scaling`, `lp_alpha`, `norm_delta`)
4. Save the file — the viewer **auto-reloads** and shows parameter changes in real-time
5. Compare the three skeleton layers to evaluate your tuning

The viewer highlights affected fingers in red when parameters change, and prints tuning guidance in the terminal.

For a full parameter reference and the recommended tuning order, see the [Retargeting Parameter Tuning Guide](docs/wuji-tuning-guide.md).

### Recommended: Wuji Glove Input

Wuji Glove is the preferred live input device for this package. It is supported through `wuji_sdk`, publishes 21 MediaPipe-format hand keypoints, and uses the Wuji Glove example configs.

Before running teleoperation, prepare and verify the glove in Wuji Studio:

1. Download Wuji Studio from the [wuji-studio releases page](https://github.com/wuji-technology/wuji-studio/releases).
2. Install the desktop application on Ubuntu:
   ```bash
   sudo apt install ./wuji-studio_<version>_amd64.deb
   ```
3. Connect the Wuji Glove and launch Wuji Studio.
4. In the **Connect** view, search for the glove and connect it.
5. After the connection succeeds, open **Visualization** to check the glove's live spatial motion and contact status.
6. Before teleoperation, open **Calibration** and follow the on-screen calibration procedure. The calibration file is saved automatically after the procedure completes.

```bash
cd example

# Simulation
python teleop_sim.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Real hardware
python teleop_real.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Optional: calibrate neutral-pose wrist/thumb offsets
python calibrate_offset.py --hand right --glove-sn <YOUR_SN> \
    --config config/adaptive_analytical_wuji_glove_right.yaml

# Optional: tune with the interactive GUI
mjpython tuning_tool.py --wuji-glove --hand right --glove-sn <YOUR_SN>
```

The Wuji Glove path adds per-hand configs (`adaptive_analytical_wuji_glove_left.yaml` / `adaptive_analytical_wuji_glove_right.yaml`) and supports neutral-pose offset calibration via `calibrate_offset.py`.

### Custom Input Devices

Want to integrate your own hand input device? Follow the [Custom Input Device Integration Guide](docs/new-device-integration.md) — most cases require only a small wrapper class, no algorithm changes.

## Quest3 + Astribot 手动分开启动

日常操作优先使用前面的一键脚本。只有需要分别查看 Bridge 与
Supervisor 日志、替换端口或排查启动问题时，才使用以下方式。

先启动 recording-only Bridge：

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting

./scripts/run_in_unified_teleop_env.sh \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_wujihand_ros2_bridge \
  --listen-host 127.0.0.1 \
  --listen-port 9011
```

确认输出包含 `driver_commands=DISABLED`，然后在另一个控制 PC 终端启动
Supervisor：

```bash
./scripts/run_in_unified_teleop_env.sh \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --run-m8-fixed-anchor-real \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --m8-position-scale 2.0 \
  --m8-rotation-scale 1.0 \
  --enable-hand-dryrun \
  --hand-retarget-real \
  --hand-control-rate-hz 120 \
  --hand-bridge-udp-host 127.0.0.1 \
  --hand-bridge-udp-port 9011
```

## Citation

If you find this project useful, please consider citing:

```bibtex
@software{wuji2026retargeting,
  title={WujiHand Retargeting},
  author={Guanqi He and Wentao Zhang},
  year={2026},
  url={https://github.com/wuji-technology/wuji-retargeting},
  note={* Equal contribution}
}
```

## Acknowledgements

This project builds upon several excellent open-source projects:

- [MuJoCo](https://mujoco.org/) for physics simulation
- [dex-retargeting](https://github.com/dexsuite/dex-retargeting) for hand retargeting algorithms
- [DexPilot](https://arxiv.org/abs/1910.03135) for vision-based teleoperation insights
- [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop) for Apple Vision Pro streaming

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
