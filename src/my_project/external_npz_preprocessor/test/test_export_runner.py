from pathlib import Path

from external_npz_preprocessor.export_runner import (
    default_output_npz_dir,
    default_output_path_for_source,
    default_source_npz_template_dir,
    resolve_output_path,
)


def test_empty_output_path_defaults_to_named_data_output_directory(tmp_path):
    source = Path("/tmp/example/source_file.npz")

    assert resolve_output_path(source, "", data_root=tmp_path) == (
        tmp_path / "output_npz" / "source_file" / "source_file.npz"
    )


def test_default_preprocessor_data_directories_are_under_data_root(tmp_path):
    assert default_source_npz_template_dir(tmp_path) == (
        tmp_path / "external_npz_preprocessor" / "source_npz_templates"
    )
    assert default_output_npz_dir(tmp_path) == tmp_path / "output_npz"
    assert default_output_path_for_source("part_a.npz", data_root=tmp_path) == (
        tmp_path / "output_npz" / "part_a" / "part_a.npz"
    )


def test_convert_uses_shared_head_calibration_offsets(tmp_path, monkeypatch):
    import json
    import numpy as np

    import external_npz_preprocessor.export_runner as runner
    from external_npz_preprocessor.process_params import ProcessParams

    source = tmp_path / "source.npz"
    resin_paths = np.array([[[0.0, 0.0, 0.5], [2.0, 0.0, 0.5]]], dtype=np.float32)
    np.savez(source, meta=np.array(json.dumps({"format": "external_layer_paths_v1"})), layer_0000_R=resin_paths)
    calibration_path = tmp_path / "head_offsets.json"
    calibration_path.write_text(
        json.dumps(
            {
                "resin": {"z_print_compensation_mm": 1.25},
                "fiber": {
                    "x_print_compensation_mm": 2.0,
                    "y_print_compensation_mm": -3.0,
                    "z_offset_mm": 4.0,
                },
            }
        ),
        encoding="utf-8",
    )
    captured = {}

    def fake_export_npz(commands, output_path, **kwargs):
        captured["kwargs"] = kwargs
        return {"rows": 0, "parts": 0, "total_s": 0.0}

    monkeypatch.setattr(runner, "export_npz", fake_export_npz)

    runner.convert_external_npz(
        source,
        tmp_path / "out.npz",
        ProcessParams(),
        calibration_path=calibration_path,
        cut_lift_mm=22.0,
        cut_wait_s=11.0,
    )

    assert captured["kwargs"]["tool_offset"] == (2.0, -3.0, 4.0)
    assert captured["kwargs"]["resin_z_print_compensation_mm"] == 1.25
    assert captured["kwargs"]["cut_lift_mm"] == 22.0
    assert captured["kwargs"]["cut_wait_s"] == 11.0
    assert captured["kwargs"]["fiber_retract_length_mm"] == 10.0


def test_exporter_uses_curve_start_acceleration_without_changing_default(tmp_path, monkeypatch):
    import path_processing_core.npz_exporter as exporter
    from path_processing_core.polynomial_interpolator import InterpolatedPoint
    from path_processing_core.types import GlobalCurveCommand, Position

    captured = []

    def fake_sample_global_curve_iter(curve, **kwargs):
        captured.append((curve.raw, kwargs.get("t_acc")))
        yield InterpolatedPoint(
            t=0.0,
            pos=curve.start_pos,
            e=curve.e_val,
            extrude_speed=0.0,
            feedrate_mm_min=curve.feedrate,
            cmd_type=curve.type,
            line=curve.line,
            raw=curve.raw,
        )

    monkeypatch.setattr(
        exporter,
        "sample_global_curve_iter",
        fake_sample_global_curve_iter,
    )
    start = Position(0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    end = Position(10.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    commands = [
        GlobalCurveCommand(
            type="PRINT",
            cmd="POLYLINE",
            start_pos=start,
            control_points=[end],
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=1,
            raw="default_curve",
        ),
        GlobalCurveCommand(
            type="PRINT",
            cmd="POLYLINE",
            start_pos=start,
            control_points=[end],
            e_val=2.0,
            delta_e=1.0,
            feedrate=600.0,
            line=2,
            raw="fiber_curve",
            time_acc_s=4.5,
        ),
    ]

    exporter.export_npz(commands, str(tmp_path / "out.npz"), dt=0.004)

    assert captured == [("default_curve", None), ("fiber_curve", 4.5)]


def _decoded_src_lines(data):
    return [raw.decode("utf-8").rstrip("\x00") for raw in data["src_line"]]


def _decoded_event_types(data):
    vocab = {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(data["event_type_vocab_keys"], data["event_type_vocab_vals"])
    }
    return [vocab[int(value)] for value in data["event_type"]]


def _next_src_line_group(src_lines, after_index):
    start = next(
        idx
        for idx in range(after_index + 1, len(src_lines))
        if src_lines[idx] != src_lines[after_index]
    )
    source = src_lines[start]
    end = next(
        (
            idx
            for idx in range(start + 1, len(src_lines))
            if src_lines[idx] != source
        ),
        len(src_lines),
    )
    return list(range(start, end))


def test_external_npz_reset_anchor_starts_at_zero_without_changing_ordinary_holds(
    tmp_path,
):
    import numpy as np

    from path_processing_core.npz_exporter import export_npz
    from path_processing_core.types import (
        ExtrudeWait,
        MoveCommand,
        Position,
        ResetECommand,
    )

    dt = 0.004
    hold_position = Position(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def export_after_reset(raw, filename):
        commands = [
            MoveCommand(
                type="TRAVEL",
                cmd="G0",
                start_pos=hold_position,
                pos=hold_position,
                e_val=0.0,
                delta_e=0.0,
                feedrate=600.0,
                line=1,
                layer=0,
                subtype="TRAVEL",
                raw="establish_hold_position",
                is_pure_state_change=False,
            ),
            ExtrudeWait(
                type="EXTRUDE_WAIT",
                wait_sec=dt,
                delta_e=5.0,
                feedrate=600.0,
                line=2,
                layer=0,
                subtype="TRAVEL",
                raw="accumulate_e",
            ),
            ResetECommand(
                type="RESET_E",
                val=0.0,
                line=3,
                layer=0,
                subtype="TRAVEL",
                raw="external_npz_path_reset",
            ),
            ExtrudeWait(
                type="EXTRUDE_WAIT",
                wait_sec=dt,
                delta_e=0.0,
                feedrate=600.0,
                line=4,
                layer=0,
                subtype="TRAVEL",
                raw=raw,
            ),
        ]
        output = tmp_path / filename
        export_npz(commands, str(output), dt=dt, enable_extrude_wait=True)
        return output

    anchor_output = export_after_reset("external_npz_reset_anchor", "anchor.npz")
    with np.load(anchor_output) as anchor_data:
        anchor_src_lines = _decoded_src_lines(anchor_data)
        anchor_event_types = _decoded_event_types(anchor_data)
        reset_idx = anchor_event_types.index("extrude_reset")
        anchor_idx = [
            idx for idx, source in enumerate(anchor_src_lines) if source == "4"
        ]

        assert np.isclose(anchor_data["e"][reset_idx], 5.0)
        assert len(anchor_idx) == 1
        assert np.isclose(anchor_data["e"][anchor_idx[0]], 0.0)
        for field in ("x", "y", "z", "a", "b", "c"):
            assert np.isclose(
                anchor_data[field][anchor_idx[0]],
                anchor_data[field][reset_idx],
            )

    near_match_output = export_after_reset(
        "external_npz_reset_anchor_suffix",
        "near_match.npz",
    )
    with np.load(near_match_output) as near_match_data:
        near_match_src_lines = _decoded_src_lines(near_match_data)
        near_match_idx = [
            idx for idx, source in enumerate(near_match_src_lines) if source == "4"
        ]

        assert len(near_match_idx) == 1
        assert np.isclose(near_match_data["e"][near_match_idx[0]], 5.0)


def test_external_npz_prime_settle_exports_125_stationary_rows(tmp_path):
    import numpy as np

    from path_processing_core.npz_exporter import export_npz
    from path_processing_core.types import ExtrudeWait, MoveCommand, Position

    dt = 0.004
    hold_position = Position(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    commands = [
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=hold_position,
            pos=hold_position,
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=10,
            layer=0,
            subtype="TRAVEL",
            raw="establish_hold_position",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=dt,
            delta_e=2.0,
            feedrate=600.0,
            line=11,
            layer=0,
            subtype="TRAVEL",
            raw="external_npz_prime",
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=0.5,
            delta_e=0.0,
            feedrate=600.0,
            line=12,
            layer=0,
            subtype="TRAVEL",
            raw="external_npz_prime_settle",
        ),
    ]
    output = tmp_path / "prime_settle.npz"

    export_npz(commands, str(output), dt=dt, enable_extrude_wait=True)

    with np.load(output) as data:
        settle_idx = [
            idx for idx, source in enumerate(_decoded_src_lines(data)) if source == "12"
        ]
        assert len(settle_idx) == 125
        for field, expected in zip(
            ("x", "y", "z"),
            (hold_position.x, hold_position.y, hold_position.z),
        ):
            assert np.allclose(data[field][settle_idx], expected)
        assert np.allclose(data["e"][settle_idx], 2.0)


def test_fiber_cut_lift_retracts_before_travel_and_next_path_prepares_after_travel(tmp_path):
    import json
    import numpy as np

    from external_npz_preprocessor.export_runner import convert_external_npz
    from external_npz_preprocessor.process_params import FiberProcessParams, ProcessParams

    source = tmp_path / "two_fiber_paths.npz"
    fiber_paths = np.array(
        [
            [[0.0, 0.0, 0.6], [10.0, 0.0, 0.6]],
            [[30.0, 0.0, 0.6], [40.0, 0.0, 0.6]],
        ],
        dtype=np.float32,
    )
    np.savez(
        source,
        meta=np.array(json.dumps({"format": "external_layer_paths_v1"})),
        layer_0000_F=fiber_paths,
    )
    calibration_path = tmp_path / "head_offsets.json"
    calibration_path.write_text(
        json.dumps(
            {
                "resin": {"z_print_compensation_mm": 0.0},
                "fiber": {
                    "x_print_compensation_mm": 0.0,
                    "y_print_compensation_mm": 0.0,
                    "z_offset_mm": 0.0,
                },
            }
        ),
        encoding="utf-8",
    )
    out = tmp_path / "out.npz"
    params = ProcessParams(
        dt=1.0,
        fiber=FiberProcessParams(
            extrusion_scale=1.0,
            feed_mm_s=10.0,
            retract_length_mm=4.0,
            prime_length_mm=6.0,
            retract_speed_mm_s=4.0,
            prime_speed_mm_s=6.0,
        ),
        travel_feed_mm_s=10.0,
    )

    convert_external_npz(
        source,
        out,
        params,
        calibration_path=calibration_path,
        cut_lift_mm=20.0,
        cut_wait_s=0.0,
    )

    data = np.load(out)
    src_lines = _decoded_src_lines(data)
    event_types = _decoded_event_types(data)
    cut_idx = event_types.index("cut")
    cut_src = src_lines[cut_idx]
    cut_e = float(data["e"][cut_idx])
    cut_z = float(data["z"][cut_idx])

    local_reset_idx = cut_idx + 1
    assert event_types[local_reset_idx] == "extrude_reset"
    assert np.isclose(data["e"][local_reset_idx], cut_e)
    local_anchor_idx = local_reset_idx + 1
    assert event_types[local_anchor_idx] == ""
    assert np.isclose(data["e"][local_anchor_idx], 0.0)
    for field in ("x", "y", "z", "a", "b", "c"):
        assert np.isclose(data[field][local_anchor_idx], data[field][cut_idx])

    reset_idx = next(
        idx
        for idx in range(local_anchor_idx + 1, len(event_types))
        if event_types[idx] == "extrude_reset"
    )
    cut_motion_idx = [
        idx for idx in range(local_anchor_idx + 1, reset_idx) if src_lines[idx] == cut_src
    ]

    assert cut_motion_idx
    assert np.isclose(np.max(data["z"][cut_motion_idx]), cut_z + 20.0)
    assert np.isclose(np.max(data["e"][cut_motion_idx]), 20.0)
    assert np.isclose(data["z"][cut_motion_idx[-1]], cut_z + 20.0)
    assert np.isclose(
        data["e"][cut_motion_idx[-1]],
        -params.fiber.retract_length_mm,
    )

    assert event_types[reset_idx] == "extrude_reset"
    assert np.isclose(
        data["e"][reset_idx],
        -params.fiber.retract_length_mm,
    )
    assert np.isclose(data["z"][reset_idx], cut_z + 20.0)

    anchor_idx = _next_src_line_group(src_lines, reset_idx)
    assert len(anchor_idx) == 1
    anchor_row = anchor_idx[0]
    assert np.isclose(data["e"][anchor_row], 0.0)
    for field in ("x", "y", "z", "a", "b", "c"):
        assert np.isclose(data[field][anchor_row], data[field][reset_idx])

    travel_idx = _next_src_line_group(src_lines, anchor_row)
    assert travel_idx
    assert np.allclose(data["e"][travel_idx], 0.0)
    assert np.isclose(data["z"][travel_idx[0]], cut_z + 20.0)
    assert np.isclose(data["z"][travel_idx[-1]], cut_z)
    assert np.isclose(data["x"][travel_idx[0]], 10.0)
    assert np.isclose(data["x"][travel_idx[-1]], 30.0)
    assert np.allclose(data["y"][travel_idx], 0.0)

    prime_idx = _next_src_line_group(src_lines, travel_idx[-1])
    assert len(prime_idx) == 1
    assert np.isclose(data["e"][prime_idx[0]], 6.0)
    assert np.isclose(data["z"][prime_idx[0]], cut_z)
    assert np.isclose(data["x"][prime_idx[0]], 30.0)
    assert np.isclose(data["y"][prime_idx[0]], 0.0)

    settle_idx = _next_src_line_group(src_lines, prime_idx[-1])
    assert np.allclose(data["e"][settle_idx], 6.0)
    assert np.allclose(data["z"][settle_idx], cut_z)
    assert np.allclose(data["x"][settle_idx], 30.0)
    assert np.allclose(data["y"][settle_idx], 0.0)

    print_idx = _next_src_line_group(src_lines, settle_idx[-1])
    assert np.isclose(data["e"][print_idx[0]], 6.0)
    assert np.isclose(data["z"][print_idx[0]], cut_z)
    assert np.isclose(data["x"][print_idx[0]], 30.0)
    assert np.isclose(data["y"][print_idx[0]], 0.0)


def test_convert_writes_startup_events_and_tool_reset_order_to_npz(tmp_path):
    import json
    import numpy as np

    from external_npz_preprocessor.process_params import ProcessParams

    source = tmp_path / "source.npz"
    resin_paths = np.array([[[0.0, 0.0, 0.5], [2.0, 0.0, 0.5]]], dtype=np.float32)
    fiber_paths = np.array([[[2.0, 0.0, 0.6], [4.0, 0.0, 0.6]]], dtype=np.float32)
    np.savez(
        source,
        meta=np.array(json.dumps({"format": "external_layer_paths_v1"})),
        layer_0000_R=resin_paths,
        layer_0000_F=fiber_paths,
    )
    calibration_path = tmp_path / "head_offsets.json"
    calibration_path.write_text(
        json.dumps(
            {
                "resin": {"z_print_compensation_mm": 0.0},
                "fiber": {
                    "x_print_compensation_mm": 0.0,
                    "y_print_compensation_mm": 0.0,
                    "z_offset_mm": 0.0,
                },
            }
        ),
        encoding="utf-8",
    )
    out = tmp_path / "out.npz"

    from external_npz_preprocessor.export_runner import convert_external_npz

    convert_external_npz(source, out, ProcessParams(), calibration_path=calibration_path)

    with np.load(out) as data:
        events = _decoded_event_types(data)
        non_empty_events = [event for event in events if event]

        assert non_empty_events == [
            "fan_resin",
            "fan_cf",
            "heat_resin",
            "heat_cf",
            "extrude_reset",
            "extrude_reset",
            "extrude_reset",
            "tool_change_cf",
            "extrude_reset",
            "cut",
            "extrude_reset",
            "extrude_reset",
        ]

        cut_row_idx = events.index("cut")
        assert data["payload"][cut_row_idx].decode("utf-8").rstrip("\x00") == "1"


def test_convert_skips_fiber_startup_events_when_source_has_no_fiber_paths(tmp_path):
    import json
    import numpy as np

    from external_npz_preprocessor.export_runner import convert_external_npz
    from external_npz_preprocessor.process_params import ProcessParams

    source = tmp_path / "resin_only_source.npz"
    resin_paths = np.array(
        [[[0.0, 0.0, 0.5], [10.0, 0.0, 0.5]]],
        dtype=np.float32,
    )
    np.savez(
        source,
        meta=np.array(json.dumps({"format": "external_layer_paths_v1"})),
        layer_0000_R=resin_paths,
    )
    calibration_path = tmp_path / "head_offsets.json"
    calibration_path.write_text(
        json.dumps(
            {
                "resin": {"z_print_compensation_mm": 0.0},
                "fiber": {
                    "x_print_compensation_mm": 0.0,
                    "y_print_compensation_mm": 0.0,
                    "z_offset_mm": 0.0,
                },
            }
        ),
        encoding="utf-8",
    )
    out = tmp_path / "out.npz"

    convert_external_npz(source, out, ProcessParams(), calibration_path=calibration_path)

    data = np.load(out)
    event_vocab = {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(data["event_type_vocab_keys"], data["event_type_vocab_vals"])
    }
    events = [event_vocab[int(value)] for value in data["event_type"]]
    non_empty_events = [event for event in events if event]

    assert "fan_resin" in non_empty_events
    assert "heat_resin" in non_empty_events
    assert "fan_cf" not in non_empty_events
    assert "heat_cf" not in non_empty_events
