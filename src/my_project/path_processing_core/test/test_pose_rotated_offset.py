import json

import numpy as np

from path_processing_core.local_injector import (
    _calibration_id,
    _rotated_reference_offsets,
    inject_npz,
)
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import MoveCommand, Position


def test_flat_reference_offset_rotates_with_kuka_az_by_cx():
    rotated_x = _rotated_reference_offsets(
        np.asarray([0.0, 90.0, 0.0]),
        np.asarray([0.0, 0.0, 90.0]),
        np.asarray([0.0, 0.0, 0.0]),
        np.asarray([1.0, 0.0, 0.0]),
    )
    np.testing.assert_allclose(
        rotated_x,
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)),
        atol=1e-12,
    )

    rotated_y_by_c = _rotated_reference_offsets(
        np.asarray([0.0]),
        np.asarray([0.0]),
        np.asarray([90.0]),
        np.asarray([0.0, 1.0, 0.0]),
    )
    np.testing.assert_allclose(rotated_y_by_c, ((0.0, 0.0, 1.0),), atol=1e-12)


def test_calibration_id_is_stable_and_semantic():
    first = _calibration_id(np.asarray([-0.34, -1.24, 3.2]), -30.5)
    second = _calibration_id(np.asarray([-0.34, -1.24, 3.2]), -30.5)
    changed = _calibration_id(np.asarray([-0.34, -1.24, 3.1]), -30.5)

    assert first == second
    assert first.startswith("sha256:")
    assert changed != first


def test_injection_rotates_each_fiber_row_and_preserves_e(tmp_path):
    source = tmp_path / "pose_base.npz"
    ready = tmp_path / "pose_ready.npz"
    reinjected = tmp_path / "pose_reinjected.npz"
    start = Position(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    end = Position(10.0, 0.0, 1.0, 90.0, 0.0, 0.0)
    command = MoveCommand(
        type="PRINT",
        cmd="G1",
        start_pos=start,
        pos=end,
        e_val=2.0,
        delta_e=2.0,
        feedrate=600.0,
        line=1,
        layer=0,
        subtype="FIBER_PRINT",
        raw="pose_rotated_fiber",
    )
    export_npz(
        [command],
        str(source),
        dt=0.004,
        default_feed_mm_s=10.0,
        tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=0.0,
    )
    with np.load(source, allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in data.files}
    arrays["tool_id"][arrays["event_flag"] == 0] = 1
    np.savez(source, **arrays)

    inject_npz(source, ready, tool_offset=(1.0, 0.0, 0.0))
    with np.load(source, allow_pickle=False) as base, np.load(
        ready, allow_pickle=False
    ) as actual:
        expected_offset = _rotated_reference_offsets(
            base["a64"], base["b64"], base["c64"], np.asarray((1.0, 0.0, 0.0))
        )
        actual_delta = np.column_stack(
            [actual[key] - base[key] for key in ("x64", "y64", "z64")]
        )
        np.testing.assert_allclose(actual_delta, expected_offset, atol=1e-12)
        np.testing.assert_array_equal(actual["e64"], base["e64"])
        manifest = json.loads(str(actual["core_injection_manifest"].item()))
        assert manifest["injection_state"] == "machine_ready"
        assert manifest["offset_application"] == "per_sample_pose_rotated"

    sidecar = json.loads(ready.with_suffix(".offset.json").read_text(encoding="utf-8"))
    assert sidecar["tool_offset"] == [1.0, 0.0, 0.0]
    assert sidecar["calibration_id"].startswith("sha256:")

    inject_npz(ready, reinjected, tool_offset=(0.0, 1.0, 0.0))
    with np.load(source, allow_pickle=False) as base, np.load(
        reinjected, allow_pickle=False
    ) as actual:
        expected_offset = _rotated_reference_offsets(
            base["a64"], base["b64"], base["c64"], np.asarray((0.0, 1.0, 0.0))
        )
        actual_delta = np.column_stack(
            [actual[key] - base[key] for key in ("x64", "y64", "z64")]
        )
        np.testing.assert_allclose(actual_delta, expected_offset, atol=1e-12)
