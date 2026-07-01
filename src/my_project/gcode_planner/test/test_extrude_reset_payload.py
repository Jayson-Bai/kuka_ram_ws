import json

import numpy as np

import path_processing_core.npz_exporter as npz_exporter
from gcode_planner.cli import _default_output_path
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import (
    ExtrudeWait,
    MoveCommand,
    Position,
    ResetECommand,
    ToolChangeCommand,
)


def test_extrude_reset_payload_uses_current_tool_id(tmp_path):
    out = tmp_path / "reset_payload.npz"
    parsed = [
        ResetECommand(
            type="RESET_E",
            val=0.0,
            line=10,
            layer=0,
            subtype="Custom",
            raw="G92 E0",
        )
    ]

    export_npz(parsed, str(out))

    data = np.load(out)
    payload = data["payload"][0].decode("utf-8").rstrip("\x00")
    event_type_vocab = {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(
            data["event_type_vocab_keys"],
            data["event_type_vocab_vals"],
        )
    }

    assert event_type_vocab[int(data["event_type"][0])] == "extrude_reset"
    assert payload == "2"


def test_extrude_wait_exports_stationary_e_change(tmp_path):
    out = tmp_path / "extrude_wait.npz"
    parsed = [
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=Position(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
            pos=Position(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G0 X1 Y2 Z3",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=0.5,
            delta_e=1.0,
            feedrate=120.0,
            line=2,
            raw="G1 E1 F120",
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
    )

    data = np.load(out)
    assert data["x"][-1] == data["x"][-2]
    assert data["y"][-1] == data["y"][-2]
    assert data["z"][-1] == data["z"][-2]
    assert data["e"][-1] > data["e"][-2]
    assert np.isclose(data["e"][-1], 1.0)


def test_npz_export_includes_layer_progress_metadata(tmp_path):
    out = tmp_path / "layer_progress.npz"
    parsed = [
        MoveCommand(
            type="TRAVEL",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G1 X1 F600",
            is_pure_state_change=False,
        ),
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(2.0, 0.0, 0.2, 0.0, 0.0, 0.0),
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=2,
            layer=2,
            subtype="Perimeter",
            raw="G1 X2 Z0.2 E1 F600",
            is_pure_state_change=False,
        ),
    ]

    export_npz(parsed, str(out), dt=0.1, default_feed_mm_s=10.0)

    data = np.load(out)
    assert "layer_index" in data.files
    assert "total_layers" in data.files
    assert data["layer_index"][0] == 0
    assert data["layer_index"][-1] == 2
    assert data["total_layers"][0] == 3
    assert data["total_layers"][-1] == 3

def _decoded_src_lines(data):
    return [item.decode("utf-8").rstrip("\x00") for item in data["src_line"]]


def _decoded_event_type_vocab(data):
    return {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(
            data["event_type_vocab_keys"],
            data["event_type_vocab_vals"],
        )
    }


def test_prime_extrude_wait_overlaps_previous_travel_by_default(tmp_path):
    out = tmp_path / "prime_overlap.npz"
    parsed = [
        MoveCommand(
            type="TRAVEL",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G1 X10 F600",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=1.0,
            delta_e=4.0,
            feedrate=240.0,
            line=2,
            raw="G1 E4 F240",
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=1.0,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
    )

    data = np.load(out)
    src_lines = _decoded_src_lines(data)
    travel_idx = [idx for idx, src in enumerate(src_lines) if src == "1"]

    assert "2" not in src_lines
    assert np.isclose(data["e"][travel_idx[0]], 0.0)
    assert np.isclose(data["e"][travel_idx[-1]], 4.0)


def test_retract_wait_overlaps_previous_travel_before_reset_by_default(
        tmp_path):
    out = tmp_path / "retract_overlap.npz"
    parsed = [
        MoveCommand(
            type="TRAVEL",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=10.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G1 X10 F600",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=0.5,
            delta_e=-3.0,
            feedrate=360.0,
            line=2,
            raw="G1 E7 F360",
        ),
        ResetECommand(
            type="RESET_E",
            val=0.0,
            line=3,
            layer=0,
            subtype="TRAVEL",
            raw="G92 E0",
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=1.0,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
    )

    data = np.load(out)
    src_lines = _decoded_src_lines(data)
    travel_idx = [idx for idx, src in enumerate(src_lines) if src == "1"]
    event_type_vocab = _decoded_event_type_vocab(data)
    reset_idx = src_lines.index("3")

    assert "2" not in src_lines
    assert np.isclose(data["e"][travel_idx[0]], 10.0)
    assert np.isclose(data["e"][travel_idx[-1]], 7.0)
    assert (
        event_type_vocab[int(data["event_type"][reset_idx])]
        == "extrude_reset"
    )
    assert np.isclose(data["e"][reset_idx], 7.0)


def test_retract_across_reset_keeps_existing_extrude_wait_rows(tmp_path):
    out = tmp_path / "cross_reset_retract.npz"
    parsed = [
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=10.0,
            delta_e=10.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="Perimeter",
            raw="G1 X10 E10 F600",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=0.5,
            delta_e=-3.0,
            feedrate=360.0,
            line=2,
            raw="G1 E7 F360",
        ),
        ResetECommand(
            type="RESET_E",
            val=0.0,
            line=3,
            layer=0,
            subtype="Perimeter",
            raw="G92 E0",
        ),
        MoveCommand(
            type="TRAVEL",
            cmd="G1",
            start_pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(12.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=4,
            layer=0,
            subtype="TRAVEL",
            raw="G1 X12 F600",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=1.0,
            delta_e=4.0,
            feedrate=240.0,
            line=5,
            raw="G1 E4 F240",
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=1.0,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
    )

    data = np.load(out)
    src_lines = _decoded_src_lines(data)
    retract_idx = src_lines.index("2")
    travel_idx = [idx for idx, src in enumerate(src_lines) if src == "4"]

    assert "2" in src_lines
    assert "5" not in src_lines
    assert np.isclose(data["e"][retract_idx], 7.0)
    assert np.isclose(data["e"][travel_idx[0]], 0.0)
    assert np.isclose(data["e"][travel_idx[-1]], 4.0)


def test_travel_extrude_overlap_can_be_disabled(tmp_path):
    out = tmp_path / "prime_overlap_disabled.npz"
    parsed = [
        MoveCommand(
            type="TRAVEL",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G1 X10 F600",
            is_pure_state_change=False,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT",
            wait_sec=1.0,
            delta_e=4.0,
            feedrate=240.0,
            line=2,
            raw="G1 E4 F240",
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=1.0,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
        enable_travel_extrude_overlap=False,
    )

    data = np.load(out)
    src_lines = _decoded_src_lines(data)
    travel_idx = [idx for idx, src in enumerate(src_lines) if src == "1"]
    extrude_wait_idx = [idx for idx, src in enumerate(src_lines) if src == "2"]

    assert extrude_wait_idx
    assert np.isclose(data["e"][travel_idx[-1]], 0.0)
    assert np.isclose(data["e"][extrude_wait_idx[-1]], 4.0)


def test_cli_enables_extrude_wait_for_formal_exports():
    from pathlib import Path

    src = (
        Path(__file__).resolve().parents[1] /
        "gcode_planner" /
        "cli.py").read_text(
        encoding="utf-8")
    export_call = src.split(
        "    stats = export_npz(", 1)[1].split(
        "    t3 = time.perf_counter()", 1)[0]

    assert "enable_extrude_wait=True" in export_call
    assert (
        "resin_z_print_compensation_mm=args.resin_z_print_compensation_mm"
        in export_call
    )




def test_export_npz_applies_fiber_tool_offset_directly_after_resin_z(tmp_path):
    out = tmp_path / "direct_fiber_offset.npz"
    parsed = [
        ToolChangeCommand(
            type="TOOL_CHANGE",
            tool=0,
            line=1,
            layer=0,
            subtype="TRAVEL",
        )
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        tool_offset=(5.0, 4.0, -25.0),
        resin_z_print_compensation_mm=-20.0,
    )

    data = np.load(out)
    assert np.any(np.isclose(data["z"], -20.0, atol=1e-4))
    assert np.any(
        np.isclose(data["x"], 5.0, atol=1e-4)
        & np.isclose(data["y"], 4.0, atol=1e-4)
        & np.isclose(data["z"], -45.0, atol=1e-4)
    )

def test_export_npz_records_resin_z_compensation_sidecar(tmp_path):
    import json

    out = tmp_path / "z_comp.npz"
    parsed = [
        ToolChangeCommand(
            type="TOOL_CHANGE",
            tool=1,
            line=0,
            layer=0,
            subtype="TRAVEL",
        ),
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=Position(10.0, 20.0, 5.0, 1.0, 2.0, 3.0),
            pos=Position(10.0, 20.0, 5.0, 1.0, 2.0, 3.0),
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="TRAVEL",
            raw="G0 X10 Y20 Z5",
            is_pure_state_change=False,
        ),
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(10.0, 20.0, 5.0, 1.0, 2.0, 3.0),
            pos=Position(20.0, 20.0, 5.0, 1.0, 2.0, 3.0),
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=2,
            layer=0,
            subtype="Perimeter",
            raw="G1 X20 E1",
            is_pure_state_change=False,
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        resin_z_print_compensation_mm=-2.0,
    )

    data = np.load(out)
    assert data["move_type"][0] == 0
    assert np.isclose(data["x"][0], 0.0)
    assert np.isclose(data["y"][0], 0.0)
    assert np.isclose(data["z"][0], 0.0)
    before_extrude = data["e"] <= 1e-9
    assert np.any(np.isclose(data["z"][before_extrude], -2.0, atol=1e-4))
    assert np.any(np.isclose(data["z"][before_extrude], 3.0, atol=1e-4))

    sidecar = json.loads(
        out.with_suffix(".offset.json").read_text(encoding="utf-8")
    )
    assert sidecar["resin_z_print_compensation_mm"] == -2.0


def test_flat_export_generates_layer_preview_without_manifest(tmp_path):
    out = tmp_path / "flat_preview.npz"
    parsed = [
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=1,
            layer=1,
            subtype="Perimeter",
            raw="G1 X10 E1",
            is_pure_state_change=False,
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        split_by_layer_type=False,
        plot_layer_xy=True,
        plot_stride=1,
    )

    assert out.exists()
    manifest_path = tmp_path / "flat_preview" / "flat_preview_manifest.json"
    assert not manifest_path.exists()
    preview_path = (
        tmp_path / "flat_preview" / "layer_previews" / "layer_0001.png"
    )
    assert preview_path.exists()


def test_flat_preview_breaks_lines_across_travel_and_prime(tmp_path, monkeypatch):
    out = tmp_path / "flat_preview_break.npz"
    captured = {}

    def capture_plot(layer_points, base_root):
        captured["layer_points"] = layer_points
        captured["base_root"] = base_root

    monkeypatch.setattr(npz_exporter, "_plot_flat_layer_previews", capture_plot)

    parsed = [
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="SKIN",
            raw="G1 X1 E1",
            is_pure_state_change=False,
        ),
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.5,
            delta_e=-0.5,
            feedrate=2400.0,
            line=2,
            layer=0,
            subtype="SKIN",
            raw="G1 E0.5",
            is_pure_state_change=True,
        ),
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(100.0, 100.0, 0.0, 0.0, 0.0, 0.0),
            e_val=0.5,
            delta_e=0.0,
            feedrate=600.0,
            line=3,
            layer=0,
            subtype="TRAVEL",
            raw="G0 X100 Y100",
            is_pure_state_change=False,
        ),
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(100.0, 100.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(100.0, 100.0, 0.0, 0.0, 0.0, 0.0),
            e_val=1.5,
            delta_e=1.0,
            feedrate=900.0,
            line=4,
            layer=0,
            subtype="WALL-INNER",
            raw="G1 E1.5",
            is_pure_state_change=True,
        ),
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(100.0, 100.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(101.0, 100.0, 0.0, 0.0, 0.0, 0.0),
            e_val=2.5,
            delta_e=1.0,
            feedrate=600.0,
            line=5,
            layer=0,
            subtype="WALL-INNER",
            raw="G1 X101 E2.5",
            is_pure_state_change=False,
        ),
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        split_by_layer_type=False,
        plot_layer_xy=True,
        plot_stride=1,
        enable_extrude_wait=False,
    )

    xs, ys = captured["layer_points"][0]
    points = list(zip(xs, ys))
    assert any(np.isnan(x) and np.isnan(y) for x, y in points)
    assert not any(
        np.isclose(x1, 1.0)
        and np.isclose(y1, 0.0)
        and np.isclose(x2, 100.0)
        and np.isclose(y2, 100.0)
        for (x1, y1), (x2, y2) in zip(points, points[1:])
    )


def test_default_output_path_places_npz_inside_named_output_directory(tmp_path):
    gcode = tmp_path / "100_10_cylinder_624.gcode"
    output_dir = tmp_path / "output_npz"

    assert _default_output_path(str(gcode), str(output_dir)) == str(
        output_dir / "100_10_cylinder_624" / "100_10_cylinder_624.npz"
    )


def test_nested_flat_export_uses_parent_directory_for_previews(tmp_path, monkeypatch):
    out = tmp_path / "output_npz" / "sample" / "sample.npz"
    captured = {}

    def capture_plot(layer_points, base_root):
        captured["base_root"] = base_root

    monkeypatch.setattr(npz_exporter, "_plot_flat_layer_previews", capture_plot)

    parsed = [
        MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            pos=Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            e_val=1.0,
            delta_e=1.0,
            feedrate=600.0,
            line=1,
            layer=0,
            subtype="WALL",
            raw="G1 X1 E1",
            is_pure_state_change=False,
        )
    ]

    export_npz(
        parsed,
        str(out),
        dt=0.1,
        default_feed_mm_s=10.0,
        plot_layer_xy=True,
    )

    assert out.exists()
    assert (out.parent / "sample.offset.json").exists()
    assert captured["base_root"] == str(out.parent)
