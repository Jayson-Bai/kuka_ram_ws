import numpy as np

from gcode_planner.npz_exporter import export_npz
from gcode_planner.types import (
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
