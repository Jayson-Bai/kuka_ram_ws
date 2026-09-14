import numpy as np
import pytest
from path_processing_core.rsi_validation import (
    RsiContinuityError,
    validate_final_npz,
)


def _write_final_npz(path, xyz, *, abc=None, tool_id=2):
    xyz = np.asarray(xyz, dtype=np.float32)
    rows = len(xyz)
    if abc is None:
        abc = np.zeros((rows, 3), dtype=np.float32)
    abc = np.asarray(abc, dtype=np.float32)
    np.savez(
        path,
        seq=np.arange(rows, dtype=np.int64),
        x=xyz[:, 0],
        y=xyz[:, 1],
        z=xyz[:, 2],
        a=abc[:, 0],
        b=abc[:, 1],
        c=abc[:, 2],
        event_flag=np.zeros(rows, dtype=np.uint8),
        planned_time_s=(
            np.arange(rows, dtype=np.float64) * 0.004
        ).astype(np.float32),
        tool_id=np.full(rows, tool_id, dtype=np.int64),
        move_type=np.ones(rows, dtype=np.int64),
        layer_index=np.zeros(rows, dtype=np.int64),
        move_type_vocab_keys=np.asarray([b"PRINT"]),
        move_type_vocab_vals=np.asarray([1], dtype=np.int64),
    )


def test_missing_resin_print_keeps_existing_warning_only_behavior(tmp_path):
    path = tmp_path / "fiber_only.npz"
    _write_final_npz(
        path,
        [
            (0.000, 0.0, 1.0),
            (0.001, 0.0, 1.0),
            (0.003, 0.0, 1.0),
        ],
        tool_id=1,
    )

    report = validate_final_npz(path)

    assert report["tcp_floor_ok"] is None
    assert report["ok"] is True
    assert "未找到树脂首层 PRINT/PRINT_FIT RSI 点，无法建立 TCP Z 安全阈值" in report["warnings"]
    assert report["abc_convention"] == "KUKA_AZ_BY_CX"
    assert report["max_orientation_step_deg"] == pytest.approx(0.0)


def test_orientation_validation_uses_kuka_a_as_z_rotation(tmp_path):
    path = tmp_path / "orientation.npz"
    _write_final_npz(
        path,
        [(0.0, 0.0, 1.0)] * 3,
        abc=[(0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.8, 0.0, 0.0)],
    )

    report = validate_final_npz(path, max_angular_speed_deg_s=100.0)

    assert report["max_orientation_step_deg"] == pytest.approx(0.4, abs=1e-5)
    assert report["max_observed_angular_speed_deg_s"] == pytest.approx(100.0, abs=1e-3)
    assert report["orientation_limit_ok"] is True


def test_orientation_validation_rejects_configured_4ms_limit(tmp_path):
    path = tmp_path / "orientation_jump.npz"
    _write_final_npz(
        path,
        [(0.0, 0.0, 1.0)] * 2,
        abc=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
    )

    with pytest.raises(RsiContinuityError, match="angular speed"):
        validate_final_npz(path, max_angular_speed_deg_s=100.0)
