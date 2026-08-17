# wuji-retargeting

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-retargeting)](https://github.com/wuji-technology/wuji-retargeting/releases)

Hand pose retargeting system for Wuji Hand. High-precision retargeting based on adaptive analytical and key-vector optimization, with Wuji Glove as the recommended live input path. Apple Vision Pro, video files, Intel RealSense, ZED cameras, and MANUS-style external pipelines can also be used as input sources.

https://github.com/user-attachments/assets/72116289-7a33-4a6b-83ca-fb4d9aaece0d

**Get started below. For full documentation, see the [Retargeting Docs](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/) on Wuji Docs Center.**

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

> **Ubuntu 22.04 note.** The distro's stock `pip` (22.0.2) has a build-isolation
> bug that can install this package as `UNKNOWN 0.0.0` with none of its
> dependencies (even with a correct `[build-system]`). Upgrade pip first:
> `python3 -m pip install -U pip`, then run the install commands above.

Optional input extras: `pip install -e ".[video]"` for MP4 video, `".[realsense]"` for Intel RealSense, or `".[zed]"` for STEREOLABS ZED. See [Installation](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/installation/) for Docker and Apple Vision Pro setup.

### Running

Wuji Glove is the recommended live input for development and demos.

```bash
cd example

# Recommended: Wuji Glove live input
python teleop_sim.py --input wuji_glove --hand right --glove-sn <YOUR_SN>
python teleop_real.py --input wuji_glove --hand right --glove-sn <YOUR_SN>

# Replay a recording (adaptive analytical optimizer)
python teleop_sim.py --play data/avp1.pkl --hand left

# Key-vector optimizer
python teleop_sim.py --play data/avp1.pkl --hand right --config config/vector/vector_avp.yaml
```

Other input sources — video, RealSense, ZED, and Vision Pro — use the same `teleop_*.py` entry with the matching flag. For full commands, Wuji Glove preparation, Wuji Hand 2, and the tuning tool, see the docs below.

## Documentation

Full guides live on [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/):

- [Installation](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/installation/): Dependencies, input extras, Docker, and Apple Vision Pro setup
- [Quick Start](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/quick-start/): Simulation, real hardware, Wuji Glove input, and Wuji Hand 2
- [Parameter Tuning](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/tuning/): The interactive tuning tool and the recommended tuning order
- [API Reference](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/api/): Retargeter interface and config parameters
- [Appendix](https://docs.wuji.tech/docs/en/wuji-retargeting/latest/appendix/): Algorithm principles, troubleshooting, and custom input device integration

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
