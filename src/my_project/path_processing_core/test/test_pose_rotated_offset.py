import json

import numpy as np
import pytest

from path_processing_core.local_injector import (
    _calibration_id,
    _rotated_reference_offsets,
    inject_npz,
)
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import MoveCommand, Position


def test_exporter_rejects_pose_unaware_tool_offset_for_xyzabc_path(tmp_path):
    output = tmp_path / "forbidden_direct_offset.npz"
    command = MoveCommand(
        type="PRINT",
        cmd="G1",
        start_pos=Position(0.0, 0.0, 1.0, 0.0, 12.0, -8.0),
        pos=Position(5.0, 0.0, 1.0, 0.0, 12.0, -8.0),
        e_val=1.0,
        delta_e=1.0,
        feedrate=600.0,
        line=1,
    )

    with pytest.raises(ValueError, match="zero-offset base NPZ"):
        export_npz(
            [command],
            str(output),
            dt=0.004,
            tool_offset=(-0.34, -1.24, 3.2),
        )

    assert not output.exists()


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


def test_rebuilt_curved_cut_lift_follows_endpoint_surface_normal(tmp_path):
    from test_minimal_dual_material_equivalence import _commands, _decode

    source = tmp_path / "curved_cut_base.npz"
    ready = tmp_path / "curved_cut_ready.npz"
    commands = _commands()
    for command in commands:
        if isinstance(command, MoveCommand):
            command.start_pos.a = 0.0
            command.start_pos.b = 12.0
            command.start_pos.c = -8.0
            command.pos.a = 0.0
            command.pos.b = 12.0
            command.pos.c = -8.0

    export_npz(
        commands,
        str(source),
        dt=0.004,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
        tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=0.0,
        cut_lift_mm=20.0,
        cut_wait_s=15.0,
        external_npz_cut_absolute_e=True,
    )
    inject_npz(source, ready, cut_lift_mm=10.0)

    with np.load(ready, allow_pickle=False) as data:
        roles = _decode(data, "core_injection_role")
        role_by_name = {name: code for code, name in roles.items()}
        cut_events = np.flatnonzero(
            data["core_injection_role"] == role_by_name["cut_event"]
        )
        assert len(cut_events) > 0
        event_index = int(cut_events[0])
        block_id = int(data["core_injection_block_id"][event_index])
        action_rows = np.flatnonzero(
            (data["core_injection_block_id"] == block_id)
            & (data["core_injection_role"] == role_by_name["cut_action"])
            & (data["event_flag"] == 0)
        )
        assert len(action_rows) > 0
        low = np.asarray(
            [data[name][event_index - 1] for name in ("x", "y", "z", "a", "b", "c")],
            dtype=np.float64,
        )
        action_xyz = np.column_stack(
            [data[name][action_rows] for name in ("x", "y", "z")]
        ).astype(np.float64)
        high = action_xyz[
            int(np.argmax(np.linalg.norm(action_xyz - low[:3], axis=1)))
        ]
        expected = _rotated_reference_offsets(
            np.asarray([low[3]]),
            np.asarray([low[4]]),
            np.asarray([low[5]]),
            np.asarray((0.0, 0.0, 10.0)),
        )[0]
        np.testing.assert_allclose(high - low[:3], expected, atol=2e-5)
        assert abs(float(high[0] - low[0])) > 0.1
        assert abs(float(high[1] - low[1])) > 0.1
