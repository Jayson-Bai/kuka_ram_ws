"""Tool-change bridges travel directly to the next start in both directions."""

import json

import numpy as np
import pytest

from path_processing_core.local_injector import inject_npz
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import (
    ExtrudeWait, MCommand, MoveCommand, Position, ResetECommand,
    ToolChangeCommand,
)


def _p(x, y, z=0.5):
    return Position(x, y, z, 0.0, 0.0, 0.0)


def _move(kind, start, end, line, layer, subtype, raw=""):
    return MoveCommand(
        kind, "G0" if kind == "TRAVEL" else "G1", start, end,
        1.0 if kind == "PRINT" else 0.0, 1.0 if kind == "PRINT" else 0.0,
        600.0, line, layer=layer, subtype=subtype, raw=raw,
    )


def _commands():
    return [
        _move("TRAVEL", _p(0, 0), _p(0, 0), 0, 0, "TRAVEL"),
        _move("PRINT", _p(0, 0), _p(10, 0), 1, 0, "RESIN_PRINT"),
        ToolChangeCommand("TOOL_CHANGE", 0, 2, layer=0, subtype="FIBER_PRINT"),
        ResetECommand("RESET_E", 0.0, 3, layer=0, subtype="FIBER_PRINT"),
        ExtrudeWait("EXTRUDE_WAIT", 0.04, 0.0, 600.0, 4),
        ExtrudeWait("EXTRUDE_WAIT", 0.08, 0.0, 600.0, 5),
        _move(
            "TRAVEL", _p(10, 0), _p(20, 5, 0.6), 6, 0, "TRAVEL",
            raw="external_npz_start_xy_travel",
        ),
        _move("PRINT", _p(20, 5, 0.6), _p(30, 5, 0.6), 7, 0, "FIBER_PRINT"),
        ToolChangeCommand("TOOL_CHANGE", 1, 8, layer=1, subtype="RESIN_PRINT"),
        ResetECommand("RESET_E", 0.0, 9, layer=1, subtype="RESIN_PRINT"),
        _move("TRAVEL", _p(30, 5, 0.6), _p(0, 10, 1.0), 10, 1, "TRAVEL"),
        _move("PRINT", _p(0, 10, 1.0), _p(10, 10, 1.0), 11, 1, "RESIN_PRINT"),
    ]


def _read(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def _vocab(data, name):
    return {
        int(value): key.decode().rstrip("\x00")
        for key, value in zip(data[f"{name}_vocab_keys"], data[f"{name}_vocab_vals"])
    }


def _bridges(data):
    events = _vocab(data, "event_type")
    roles = _vocab(data, "core_injection_role")
    result = []
    for index in np.flatnonzero(data["event_flag"]):
        event = events[int(data["event_type"][index])]
        if not event.startswith("tool_change"):
            continue
        block = int(data["core_injection_block_id"][index])
        bridge = np.flatnonzero(
            (data["core_injection_block_id"] == block)
            & np.asarray([
                roles[int(value)] == "tool_change_bridge"
                for value in data["core_injection_role"]
            ])
        )
        assert len(bridge)
        start, end = int(bridge[0]), int(bridge[-1]) + 1
        assert roles[int(data["core_injection_role"][end])] != "tool_change_post"
        result.append((event, index, start, end))
    return result


def _xyz(data):
    return np.column_stack([data[key].astype(np.float64) for key in ("x64", "y64", "z64")])


def _assert_direct(data, offset):
    xyz = _xyz(data)
    bridges = _bridges(data)
    assert [item[0] for item in bridges] == ["tool_change_cf", "tool_change_resin"]
    expected = {
        "tool_change_cf": np.array([20.0, 5.0, 0.6]) + offset,
        "tool_change_resin": np.array([0.0, 10.0, 1.0]),
    }
    for event, event_index, start, end in bridges:
        segment = xyz[start:end]
        assert np.allclose(segment[0], xyz[event_index], atol=1e-9)
        assert np.allclose(segment[-1], expected[event], atol=1e-6)
        assert np.min(segment[:, 2]) >= expected[event][2] - 1e-9
        assert np.max(np.linalg.norm(np.diff(segment, axis=0), axis=1)) < 0.05
    assert np.max(np.linalg.norm(np.diff(xyz, axis=0), axis=1)) < 0.05

    roles = _vocab(data, "core_injection_role")
    role_names = np.asarray([roles[int(value)] for value in data["core_injection_role"]])
    manifest = json.loads(str(data["core_injection_manifest"].item()))
    tool_blocks = [
        block for block in manifest["blocks"] if block.get("kind") == "tool_change"
    ]
    raw_origins = (
        np.asarray([10.0, 0.0, 0.5]),
        np.asarray([30.0, 5.0, 0.6]) + offset,
    )
    for block, origin, bridge in zip(tool_blocks, raw_origins, bridges):
        mask = data["core_injection_block_id"] == int(block["id"])
        pre = np.flatnonzero(mask & (role_names == "tool_change_pre"))
        assert len(pre)
        assert np.allclose(xyz[pre[0]], origin, atol=1e-6)
        wiped = origin.copy()
        wipe = block.get("resin_end_wipe")
        if wipe is not None:
            wiped[1] = wipe["y_max"] + 5.0
        sign = 1.0 if int(block["to_tool"]) == 1 else -1.0
        horizontal = wiped.copy()
        horizontal[:2] += sign * offset[:2]
        assert np.allclose(
            xyz[pre[np.isclose(xyz[pre, 2], wiped[2], atol=1e-9)][-1]],
            horizontal,
            atol=1e-5,
        )
        lifted = horizontal + np.asarray([0.0, 0.0, 10.0])
        assert np.allclose(
            xyz[pre[-1]], lifted, atol=1e-5,
        )
        assert np.allclose(xyz[pre[-1]], xyz[bridge[1]], atol=1e-9)
        assert not np.any(
            np.all(
                np.isclose(
                    xyz[bridge[1] + 1:],
                    origin,
                    atol=1e-6,
                ),
                axis=1,
            )
        )


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (3.0, 4.0, 2.0)])
def test_full_export_bridges_to_next_travel_destination(tmp_path, offset):
    path = tmp_path / "full.npz"
    export_npz(
        _commands(), str(path), tool_offset=offset, tool_change_safe_lift_mm=10.0,
        enable_extrude_wait=True,
    )
    _assert_direct(_read(path), np.asarray(offset))


def test_local_injection_matches_direct_full_export_and_repeats(tmp_path):
    base, ready, full, again = (
        tmp_path / name for name in ("base.npz", "ready.npz", "full.npz", "again.npz")
    )
    offset = (3.0, 4.0, 2.0)
    export_npz(
        _commands(), str(base), tool_change_safe_lift_mm=10.0,
        enable_extrude_wait=True,
    )
    export_npz(
        _commands(), str(full), tool_offset=offset, tool_change_safe_lift_mm=10.0,
        enable_extrude_wait=True,
    )
    inject_npz(
        base, ready, tool_offset=offset, tool_change_safe_lift_mm=10.0,
    )
    inject_npz(
        ready, again, tool_offset=(1.0, -2.0, 3.0),
        tool_change_safe_lift_mm=10.0,
    )

    expected, actual = _read(full), _read(ready)
    _assert_direct(actual, np.asarray(offset))
    _assert_direct(_read(again), np.asarray((1.0, -2.0, 3.0)))
    assert expected.keys() == actual.keys()
    for key in expected:
        if key == "core_injection_manifest":
            assert json.loads(str(actual[key].item()))["format"] == "core_npz_local_injection_v2"
        elif key in {"x64", "y64", "z64"}:
            assert np.allclose(expected[key], actual[key], rtol=0.0, atol=1e-12), key
        else:
            assert np.array_equal(expected[key], actual[key]), key


def test_local_injection_rebuilds_both_cut_holds_from_settle_parameter(tmp_path):
    commands = [
        _move("TRAVEL", _p(0, 0), _p(0, 0), 0, 0, "TRAVEL"),
        _move("PRINT", _p(0, 0), _p(10, 0), 1, 0, "FIBER_PRINT"),
        MCommand(
            type="M_COMMAND",
            code="CUT",
            params={"P": 1.0},
            line=2,
            layer=0,
            subtype="FIBER_PRINT",
            raw="external_npz_cut",
        ),
    ]
    base, direct, injected = (
        tmp_path / name for name in ("cut_base.npz", "cut_direct.npz", "cut_injected.npz")
    )
    common = dict(
        initial_tool_id=1,
        cut_lift_mm=2.0,
        cut_wait_s=0.0,
        external_npz_cut_absolute_e=True,
    )
    export_npz(commands, str(base), cut_settle_s=0.25, **common)
    export_npz(commands, str(direct), cut_settle_s=1.0, **common)
    inject_npz(base, injected, cut_settle_s=1.0)

    expected = _read(direct)
    actual = _read(injected)
    assert expected.keys() == actual.keys()
    for key in expected:
        if key == "core_injection_manifest":
            expected_manifest = json.loads(str(expected[key].item()))
            actual_manifest = json.loads(str(actual[key].item()))
            assert expected_manifest["base_parameters"]["cut_settle_s"] == 1.0
            assert actual_manifest["base_parameters"]["cut_settle_s"] == 1.0
        elif np.issubdtype(expected[key].dtype, np.number):
            assert np.allclose(expected[key], actual[key], rtol=0.0, atol=1e-12), key
        else:
            assert np.array_equal(expected[key], actual[key]), key
