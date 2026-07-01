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

    runner.convert_external_npz(source, tmp_path / "out.npz", ProcessParams(), calibration_path=calibration_path)

    assert captured["kwargs"]["tool_offset"] == (2.0, -3.0, 4.0)
    assert captured["kwargs"]["resin_z_print_compensation_mm"] == 1.25


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

    data = np.load(out)
    event_vocab = {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(data["event_type_vocab_keys"], data["event_type_vocab_vals"])
    }
    events = [event_vocab[int(value)] for value in data["event_type"]]
    non_empty_events = [event for event in events if event]

    assert non_empty_events[:7] == [
        "fan_resin",
        "fan_cf",
        "heat_resin",
        "heat_cf",
        "extrude_reset",
        "tool_change_cf",
        "extrude_reset",
    ]
    fiber_switch_idx = non_empty_events.index("tool_change_cf")
    assert non_empty_events[fiber_switch_idx + 1] == "extrude_reset"
