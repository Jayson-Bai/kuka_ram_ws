"""Resin end retraction, Y wipe and tool-change continuity regressions."""

import json

import numpy as np
import pytest

from path_processing_core import npz_exporter
from path_processing_core.local_injector import inject_npz
from path_processing_core.resin_wipe import RESIN_WIPE_POLICY
from path_processing_core.types import (
    ExtrudeWait, GlobalCurveCommand, MoveCommand, Position, ResetECommand,
    ToolChangeCommand,
)


def _commands(end_y, legacy_tail=0.0):
    p = lambda x, y: Position(x, y, 0.5, 0.0, 0.0, 0.0)
    end = p(10.0, end_y)
    after = p(10.0 - legacy_tail, end_y)
    fiber_end = p(20.0, end_y)
    commands = [
        MoveCommand("TRAVEL", "G0", p(0, 0), p(0, 0), 0.0, 0.0, 600.0, 0),
        GlobalCurveCommand(
            type="PRINT", cmd="POLYLINE", start_pos=p(0, 0),
            control_points=[p(0, 20), p(10, 20), end],
            e_val=6.0, delta_e=6.0, feedrate=600.0, line=1,
        ),
        ExtrudeWait("EXTRUDE_WAIT", 0.1, -1.0, 600.0, 2),
        ResetECommand("RESET_E", 0.0, 3),
        ExtrudeWait("EXTRUDE_WAIT", 0.004, 0.0, 600.0, 4,
                    raw="external_npz_reset_anchor"),
    ]
    if legacy_tail:
        commands.append(MoveCommand(
            "TRAVEL", "G0", end, after, 0.0, 0.0, 600.0, 5,
            raw="external_npz_resin_layer_end_travel",
        ))
    commands.extend([
        ToolChangeCommand("TOOL_CHANGE", 0, 6),
        MoveCommand("TRAVEL", "G0", after, p(15, end_y), 0.0, 0.0, 600.0, 7),
        MoveCommand("PRINT", "G1", p(15, end_y), fiber_end, 2.0, 2.0, 600.0, 8,
                    subtype="FIBER_PRINT"),
    ])
    return commands


def _read(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def _manifest(data):
    return json.loads(str(data["core_injection_manifest"].item()))


def _assert_wipe(data, end_y, direction, lift=10.0):
    block = next(b for b in _manifest(data)["blocks"] if b["kind"] == "tool_change")
    assert block["resin_end_wipe"]["policy"] == RESIN_WIPE_POLICY
    pre = np.flatnonzero(
        (data["core_injection_block_id"] == block["id"])
        & (data["core_injection_role"] == 1)
    )
    assert len(pre)
    horizontal = pre[np.isclose(data["z"][pre], 0.5, atol=1e-7, rtol=0.0)]
    assert len(horizontal)
    assert np.allclose(data["x"][horizontal], 10.0)
    assert data["y"][horizontal[-1]] == pytest.approx(end_y + direction * 5.0)
    assert np.allclose(data["e"][pre], 0.0)
    assert np.allclose(data["a"][pre], 0.0)
    assert np.allclose(data["b"][pre], 0.0)
    assert np.allclose(data["c"][pre], 0.0)
    raised = pre[data["z"][pre] > 0.500001]
    assert len(raised)
    assert np.allclose(data["y"][raised], end_y + direction * 5.0)
    assert data["z"][pre[-1]] == pytest.approx(0.5 + lift)
    before = int(pre[0]) - 1
    assert data["x"][before] == pytest.approx(10.0)
    assert data["y"][before] == pytest.approx(end_y)
    assert data["e"][before] == pytest.approx(0.0)
    xyz = np.column_stack([data[key] for key in ("x", "y", "z")])
    assert np.max(np.linalg.norm(np.diff(xyz, axis=0), axis=1)) < 0.05
    assert np.all(np.diff(data["seq"].astype(np.int64)) == 1)
    assert np.all(np.diff(data["planned_time_s"]) >= 0)


@pytest.mark.parametrize("end_y,direction", [(2.0, -1), (18.0, 1)])
def test_full_export_wipes_after_retract_toward_nearest_y_boundary(tmp_path, end_y, direction):
    path = tmp_path / "full.npz"
    npz_exporter.export_npz(
        _commands(end_y), str(path), tool_change_safe_lift_mm=10.0,
        enable_extrude_wait=True,
    )
    _assert_wipe(_read(path), end_y, direction)


@pytest.mark.parametrize("storage_mode", ["memory", "disk"])
@pytest.mark.parametrize("legacy_tail", [5.0, 20.0])
def test_legacy_local_injection_replaces_end_travel_once(
    tmp_path, monkeypatch, storage_mode, legacy_tail,
):
    source, ready, again = (tmp_path / name for name in ("old.npz", "ready.npz", "again.npz"))
    with monkeypatch.context() as legacy:
        legacy.setattr(npz_exporter, "command_resin_wipes", lambda *args: {})
        npz_exporter.export_npz(
            _commands(2.0, legacy_tail=legacy_tail), str(source),
            tool_change_safe_lift_mm=10.0, enable_extrude_wait=True,
        )
    original = _read(source)
    inject_npz(source, ready, tool_change_safe_lift_mm=10.0, storage_mode=storage_mode)
    updated = _read(ready)
    _assert_wipe(updated, 2.0, -1)
    # Retraction/print payloads are untouched; only end travel and marked blocks change.
    for data in (original, updated):
        mask = (data["event_flag"] == 0) & np.isin(data["move_type"], [1, 3])
        data["_preserved"] = np.column_stack([data[k][mask] for k in ("x", "y", "z", "e")])
    assert np.array_equal(original["_preserved"], updated["_preserved"])
    inject_npz(ready, again, tool_change_safe_lift_mm=10.0, storage_mode=storage_mode)
    repeated = _read(again)
    for key in repeated:
        if key != "core_injection_manifest":
            assert np.array_equal(updated[key], repeated[key]), key


def test_ambiguous_legacy_travel_is_not_overwritten(tmp_path, monkeypatch):
    source = tmp_path / "unknown_route.npz"
    with monkeypatch.context() as legacy:
        legacy.setattr(npz_exporter, "command_resin_wipes", lambda *args: {})
        npz_exporter.export_npz(
            _commands(2.0, legacy_tail=7.0), str(source),
            tool_change_safe_lift_mm=10.0, enable_extrude_wait=True,
        )
    before = source.read_bytes()
    with pytest.raises(ValueError, match="cannot identify the legacy resin layer-end travel"):
        inject_npz(source, source, tool_change_safe_lift_mm=10.0)
    assert source.read_bytes() == before


def test_rebuild_changes_lift_without_adding_a_second_wipe(tmp_path):
    source, ready = tmp_path / "source.npz", tmp_path / "ready.npz"
    npz_exporter.export_npz(
        _commands(18.0), str(source), tool_change_safe_lift_mm=10.0,
        enable_extrude_wait=True,
    )
    inject_npz(source, ready, tool_change_safe_lift_mm=12.0)
    _assert_wipe(_read(ready), 18.0, 1, lift=12.0)


def test_startup_switch_without_resin_deposition_has_no_wipe(tmp_path):
    path = tmp_path / "startup.npz"
    commands = _commands(2.0)
    npz_exporter.export_npz([commands[0], *commands[5:]], str(path))
    block = next(b for b in _manifest(_read(path))["blocks"] if b["kind"] == "tool_change")
    assert "resin_end_wipe" not in block
