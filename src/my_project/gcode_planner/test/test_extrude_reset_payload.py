import numpy as np

from gcode_planner.npz_exporter import export_npz
from gcode_planner.types import ExtrudeWait, MoveCommand, Position, ResetECommand


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
