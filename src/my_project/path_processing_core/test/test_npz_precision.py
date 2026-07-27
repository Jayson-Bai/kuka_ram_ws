import numpy as np

from path_processing_core.local_injector import inject_npz
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import GlobalCurveCommand, MoveCommand, Position


def _commands(feedrate=600.0):
    start = Position(0.1234567890123, 0.2345678901234, 0.5000000000001, 0.0, 0.0, 0.0)
    end = Position(2.3456789012345, 0.3456789012345, 0.5000000000002, 0.0, 0.0, 0.0)
    return [
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=Position(0.0, 0.0, 0.5, 0.0, 0.0, 0.0),
            pos=start,
            e_val=0.0,
            delta_e=0.0,
            feedrate=feedrate,
            line=0,
            layer=0,
            subtype="TRAVEL",
            raw="external_npz_start_xy_travel",
        ),
        GlobalCurveCommand(
            type="PRINT",
            cmd="POLYLINE",
            start_pos=start,
            control_points=[end],
            e_val=1.0,
            delta_e=1.0,
            feedrate=feedrate,
            line=1,
            raw="precision_test",
        )
    ]


def test_core_export_keeps_float32_consumer_and_float64_injection_fields(tmp_path):
    output = tmp_path / "precision.npz"
    export_npz(_commands(), str(output), dt=0.004)

    with np.load(output, allow_pickle=False) as data:
        for key in ("x64", "y64", "z64", "a64", "b64", "c64", "e64"):
            assert data[key].dtype == np.float64
            assert len(data[key]) == len(data["seq"])
            assert np.all(np.isfinite(data[key]))
        for key in ("x", "y", "z", "a", "b", "c", "e"):
            assert data[key].dtype == np.float32


def test_local_injector_updates_high_precision_and_public_fields(tmp_path):
    source = tmp_path / "source.npz"
    output = tmp_path / "injected.npz"
    export_npz(_commands(), str(source), dt=0.004)

    inject_npz(source, output, resin_z_print_compensation_mm=0.1)

    with np.load(output, allow_pickle=False) as data:
        assert data["z64"].dtype == np.float64
        assert data["z"].dtype == np.float32
        assert np.all(np.isfinite(data["z64"]))
        assert np.allclose(data["z"], data["z64"].astype(np.float32))
        assert np.all(np.diff(data["seq"].astype(np.int64)) == 1)


def test_local_injector_uses_manifest_anchor_without_zero_resin_marker_rows(tmp_path):
    source = tmp_path / "zero_resin_source.npz"
    unmarked = tmp_path / "zero_resin_unmarked.npz"
    output = tmp_path / "zero_resin_injected.npz"
    export_npz(_commands(), str(source), dt=0.004, resin_z_print_compensation_mm=0.0)

    with np.load(source, allow_pickle=False) as data:
        payload = {key: data[key].copy() for key in data.files}
    resin_mask = payload["core_injection_block_id"] == 1
    payload["core_injection_block_id"][resin_mask] = -1
    payload["core_injection_role"][resin_mask] = 0
    np.savez_compressed(unmarked, **payload)

    inject_npz(unmarked, output, resin_z_print_compensation_mm=0.1)

    with np.load(output, allow_pickle=False) as data:
        move_codes = {
            key.decode().rstrip("\x00"): int(value)
            for key, value in zip(
                data["move_type_vocab_keys"], data["move_type_vocab_vals"]
            )
        }
        role_codes = {
            key.decode().rstrip("\x00"): int(value)
            for key, value in zip(
                data["core_injection_role_vocab_keys"],
                data["core_injection_role_vocab_vals"],
            )
        }
        effective = np.flatnonzero(
            (data["event_flag"] == 0)
            & np.isin(data["move_type"], [move_codes["PRINT"], move_codes["PRINT_FIT"]])
        )
        compensation = np.flatnonzero(
            data["core_injection_role"] == role_codes["resin_z_compensation"]
        )
        assert len(compensation) > 0
        assert int(compensation[-1]) < int(effective[0])
        assert np.all(np.diff(data["seq"].astype(np.int64)) == 1)


def test_local_injector_accepts_existing_core_sampling_step_above_005_mm(tmp_path):
    source = tmp_path / "high_feed_source.npz"
    output = tmp_path / "high_feed_injected.npz"
    commands = _commands(feedrate=1200.0)
    start = Position(0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    end = Position(100.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    commands[0].start_pos = start
    commands[0].pos = start
    commands[1].start_pos = start
    commands[1].control_points = [end]
    export_npz(
        commands,
        str(source),
        dt=0.004,
        default_feed_mm_s=20.0,
    )

    with np.load(source, allow_pickle=False) as data:
        xyz = np.column_stack([
            data[key].astype(np.float64) for key in ("x", "y", "z")
        ])
        source_max_step = float(
            np.linalg.norm(np.diff(xyz, axis=0), axis=1).max()
        )
    assert source_max_step > 0.05

    inject_npz(source, output, resin_z_print_compensation_mm=0.1)
    with np.load(output, allow_pickle=False) as data:
        assert np.all(np.diff(data["seq"].astype(np.int64)) == 1)
