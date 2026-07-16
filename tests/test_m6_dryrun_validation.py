from pathlib import Path

import yaml

from stardust_wuji_quest3_pc_retargeting.sim.dryrun_arm_validation import (
    augment_report_with_mapping_replay,
    run_mapping_replay,
)


def test_relative_and_absolute_replay_same_trajectory_against_math_definition():
    config = yaml.safe_load((Path(__file__).parents[1] / "configs/arm/s1_quest3_default.yaml").read_text())

    replay = run_mapping_replay(config)

    assert replay["passed"] is True
    assert replay["relative_math_error_m"] <= 1e-9
    assert replay["absolute_math_error_m"] <= 1e-9


def test_mapping_replay_creates_new_output_directory(tmp_path):
    config = yaml.safe_load((Path(__file__).parents[1] / "configs/arm/s1_quest3_default.yaml").read_text())
    output = tmp_path / "new" / "mapping"

    replay = augment_report_with_mapping_replay(config, output)

    assert replay["passed"] is True
    assert (output / "mapping_replay.yaml").is_file()
