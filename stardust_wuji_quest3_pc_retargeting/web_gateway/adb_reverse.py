from __future__ import annotations

import subprocess


def ensure_adb_reverse(device_port: int, host_port: int, adb: str = "adb") -> subprocess.CompletedProcess:
    return subprocess.run(
        [adb, "reverse", f"tcp:{int(device_port)}", f"tcp:{int(host_port)}"],
        check=True,
        capture_output=True,
        text=True,
    )
