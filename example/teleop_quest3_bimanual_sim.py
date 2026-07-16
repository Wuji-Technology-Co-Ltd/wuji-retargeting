#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from stardust_wuji_quest3_pc_retargeting.sim.mock_webxr_sender import build_mock_frame
from stardust_wuji_quest3_pc_retargeting.runtime.supervisor import DryRunSupervisor


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run one dry-run Quest3 bimanual teleop simulation step.")
    parser.add_argument("--config", default="configs/services/control_pc_default.yaml")
    args = parser.parse_args(argv)
    supervisor = DryRunSupervisor()
    supervisor.handle_command("calibrate")
    supervisor.handle_command("start")
    output = supervisor.process_payload(build_mock_frame(seq=1, t=1.0))
    print(f"state={output.state} hands={list(output.hands)} arms={list(output.arms)} config={args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
