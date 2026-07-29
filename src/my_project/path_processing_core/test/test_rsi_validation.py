import numpy as np
from path_processing_core.rsi_validation import validate_final_npz


def _write_final_npz(path, xyz, *, tool_id=2):
    xyz = np.asarray(xyz, dtype=np.float32)
    rows = len(xyz)
    np.savez(
        path,
        seq=np.arange(rows, dtype=np.int64),
        x=xyz[:, 0],
        y=xyz[:, 1],
        z=xyz[:, 2],
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
    assert report["warnings"] == [
        "未找到树脂首层 PRINT/PRINT_FIT RSI 点，无法建立 TCP Z 安全阈值"
    ]
