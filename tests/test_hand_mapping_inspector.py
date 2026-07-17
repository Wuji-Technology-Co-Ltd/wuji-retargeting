import numpy as np

from stardust_wuji_quest3_pc_retargeting.tools.inspect_hand_retarget_mapping import (
    analyze_mapping_snapshots,
)


def test_analyze_mapping_snapshots_separates_left_and_right_response():
    opened = {
        "left": np.zeros(20),
        "right": np.zeros(20),
    }
    left_closed = {
        "left": np.ones(20),
        "right": np.full(20, 0.01),
    }
    right_closed = {
        "left": np.full(20, 0.02),
        "right": np.ones(20),
    }

    report = analyze_mapping_snapshots(opened, left_closed, right_closed)

    assert report["left"]["responsive"] is True
    assert report["right"]["responsive"] is True
    assert report["left"]["inactive_to_active_ratio"] < 0.1
    assert report["right"]["inactive_to_active_ratio"] < 0.1
    assert report["left"]["positive_flexion_joints"] == 8
    assert report["right"]["positive_flexion_joints"] == 8
