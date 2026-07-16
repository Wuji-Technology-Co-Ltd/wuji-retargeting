#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor import main as supervisor_main


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Quest3 bimanual real-hardware entry point with explicit opt-in gates.")
    parser.add_argument("--config", default="configs/services/control_pc_default.yaml")
    parser.add_argument("--dry-run", action="store_true", default=True)
    parser.add_argument("--enable-real-hand", action="store_true")
    parser.add_argument("--enable-real-arm", action="store_true")
    args, rest = parser.parse_known_args(argv)
    if args.enable_real_hand or args.enable_real_arm:
        raise SystemExit("real hand/arm adapters are not started by default; verify hardware checklist before enabling adapters")
    return supervisor_main(["--config", args.config, "--dry-run", *rest])


if __name__ == "__main__":
    raise SystemExit(main())
