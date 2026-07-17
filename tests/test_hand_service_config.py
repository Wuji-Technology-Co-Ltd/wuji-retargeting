from pathlib import Path

from stardust_wuji_quest3_pc_retargeting.runtime.config import load_yaml_config


def test_control_pc_hand_paths_resolve_to_existing_project_configs():
    service = load_yaml_config("configs/services/control_pc_default.yaml")

    for side in ("left", "right"):
        entry = service["hands"][side]
        assert Path(entry["retarget_config"]).is_file()
        assert Path(entry["mapping_config"]).is_file()
        assert Path(entry["safety_config"]).is_file()
