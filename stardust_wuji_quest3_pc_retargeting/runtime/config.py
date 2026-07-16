from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_yaml_config(path: str | Path) -> dict[str, Any]:
    cfg_path = Path(path).expanduser()
    if not cfg_path.is_absolute():
        cfg_path = repo_root() / cfg_path
    data = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    return resolve_relative_paths(data, cfg_path.parent)


def resolve_relative_paths(value: Any, base: Path) -> Any:
    if isinstance(value, dict):
        return {key: resolve_relative_paths(item, base) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_relative_paths(item, base) for item in value]
    if isinstance(value, str) and ("/" in value or value.endswith((".yaml", ".crt", ".key"))):
        path = Path(value).expanduser()
        if not path.is_absolute():
            return str((base / path).resolve())
    return value
