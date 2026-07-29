"""Regression test for the minimal external-NPZ mixed-material contract."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from path_processing_core.local_injector import inject_npz
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import MoveCommand, Position, ToolChangeCommand


_ROOT = Path(__file__).resolve().parents[4]
_TEMPLATE = _ROOT / "data/input_gcode/resin_0728_minimal_dual_core_template.py"


def _commands():
    spec = importlib.util.spec_from_file_location("minimal_dual_template", _TEMPLATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.commands()


def _two_way_tool_change_commands():
    start = Position(-1.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    p0 = Position(0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    p1 = Position(10.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    p2 = Position(20.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    p3 = Position(30.0, 0.0, 0.5, 0.0, 0.0, 0.0)

    def move(start, end, line, layer, subtype):
        return MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=start,
            pos=end,
            e_val=float(line),
            delta_e=1.0,
            feedrate=600.0,
            line=line,
            layer=layer,
            subtype=subtype,
            raw=f"two_way_{subtype.lower()}",
        )

    return [
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=start,
            pos=p0,
            e_val=0.0,
            delta_e=0.0,
            feedrate=600.0,
            line=0,
            layer=0,
            subtype="TRAVEL",
            raw="two_way_start",
        ),
        move(p0, p1, 1, 0, "RESIN_PRINT"),
        ToolChangeCommand(type="TOOL_CHANGE", tool=0, line=2, layer=1,
                          subtype="FIBER_PRINT"),
        move(p1, p2, 3, 1, "FIBER_PRINT"),
        ToolChangeCommand(type="TOOL_CHANGE", tool=1, line=4, layer=2,
                          subtype="RESIN_PRINT"),
        move(p2, p3, 5, 2, "RESIN_PRINT"),
    ]


def _use_from_to_tool_manifest(path):
    with np.load(path, allow_pickle=False) as data:
        arrays = {key: data[key].copy() for key in data.files}
    raw = arrays["core_injection_manifest"].item()
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    manifest = json.loads(str(raw))
    current_tool = 2
    for block in manifest["blocks"]:
        if block.get("kind") != "tool_change":
            continue
        next_tool = int(block.pop("target_tool_id"))
        block["from_tool"] = current_tool
        block["to_tool"] = next_tool
        current_tool = next_tool
    arrays["core_injection_manifest"] = np.asarray(
        json.dumps(manifest, ensure_ascii=False, separators=(",", ":")),
    )
    np.savez(path, **arrays)


def _random_multilayer_switch_commands(seed=20260729):
    rng = np.random.default_rng(seed)
    line = 1
    e_value = 0.0
    current = Position(0.0, 0.0, 0.42, 0.0, 0.0, 0.0)
    commands = [
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=current,
            pos=current,
            e_val=0.0,
            delta_e=0.0,
            feedrate=1500.0,
            line=line,
            layer=0,
            subtype="TRAVEL",
            raw="external_npz_start_xy_travel",
        )
    ]

    def next_position(z):
        nonlocal current
        current = Position(
            current.x + float(rng.uniform(3.0, 7.0)),
            current.y + float(rng.uniform(-2.0, 5.0)),
            float(z),
            0.0,
            0.0,
            0.0,
        )
        return current

    def add_travel(layer, z):
        nonlocal line, current
        start = current
        end = next_position(z)
        line += 1
        commands.append(MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=start,
            pos=end,
            e_val=e_value,
            delta_e=0.0,
            feedrate=1500.0,
            line=line,
            layer=layer,
            subtype="TRAVEL",
            raw=f"random_layer_{layer}_travel",
        ))

    def add_print(layer, material):
        nonlocal line, current, e_value
        start = current
        end = next_position(current.z)
        delta_e = float(rng.uniform(0.8, 2.2))
        e_value += delta_e
        line += 1
        commands.append(MoveCommand(
            type="PRINT",
            cmd="G1",
            start_pos=start,
            pos=end,
            e_val=e_value,
            delta_e=delta_e,
            feedrate=1200.0,
            line=line,
            layer=layer,
            subtype=f"{material}_PRINT",
            raw=f"random_layer_{layer}_{material.lower()}_print",
        ))

    def change_tool(layer, gcode_tool, material):
        nonlocal line
        line += 1
        commands.append(ToolChangeCommand(
            type="TOOL_CHANGE",
            tool=gcode_tool,
            line=line,
            layer=layer,
            subtype=f"{material}_PRINT",
            raw=f"random_to_{material.lower()}",
        ))

    layer_z = [0.42]
    for _ in range(4):
        layer_z.append(layer_z[-1] + float(rng.uniform(0.24, 0.46)))

    add_print(0, "RESIN")
    add_travel(1, layer_z[1])
    add_print(1, "RESIN")
    change_tool(1, 0, "FIBER")
    add_print(1, "FIBER")
    add_travel(2, layer_z[2])
    add_print(2, "FIBER")
    change_tool(2, 1, "RESIN")
    add_print(2, "RESIN")
    add_travel(3, layer_z[3])
    add_print(3, "RESIN")
    change_tool(3, 0, "FIBER")
    add_print(3, "FIBER")
    add_travel(4, layer_z[4])
    add_print(4, "FIBER")
    change_tool(4, 1, "RESIN")
    add_print(4, "RESIN")
    return commands


def _decode(data, name):
    keys = data[f"{name}_vocab_keys"]
    values = data[f"{name}_vocab_vals"]
    return {int(value): key.decode().rstrip("\x00") for key, value in zip(keys, values)}


def _event_contract(data):
    event_types = _decode(data, "event_type")
    roles = _decode(data, "core_injection_role")
    events = []
    for index in np.flatnonzero(data["event_flag"]):
        before = data["e64"][index - 1] if index else data["e64"][index]
        after = data["e64"][index + 1] if index + 1 < len(data["e64"]) else before
        events.append((
            event_types[int(data["event_type"][index])],
            data["payload"][index].decode().rstrip("\x00"),
            int(data["tool_id"][index]),
            # The public NPZ E field is float32; local injection may rebuild
            # its float64 mirror from that representation. Compare the
            # semantic E phase, not sub-micro-unit storage noise.
            round(float(before), 6),
            round(float(data["e64"][index]), 6),
            round(float(after), 6),
        ))

    block_events = []
    block_ids = data["core_injection_block_id"].astype(np.int64)
    for block_id in sorted(set(block_ids[block_ids >= 0])):
        indices = np.flatnonzero(block_ids == block_id)
        block_events.append(tuple(
            (
                event_types[int(data["event_type"][index])],
                roles[int(data["core_injection_role"][index])],
            )
            for index in indices
            if data["event_flag"][index]
        ))
    return tuple(events), tuple(block_events)


def _max_xyz_step(data):
    xyz = np.column_stack([data[key].astype(np.float64) for key in ("x64", "y64", "z64")])
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).max())


def test_minimal_dual_material_core_and_injection_contract(tmp_path):
    source = tmp_path / "minimal_source.npz"
    direct = tmp_path / "minimal_direct.npz"
    injected = tmp_path / "minimal_injected.npz"

    base = dict(
        dt=0.004,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
        tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=0.0,
        tool_change_safe_lift_mm=20.0,
        cut_lift_mm=20.0,
        cut_wait_s=15.0,
        external_npz_cut_absolute_e=True,
    )
    changed = dict(base)
    changed.update(
        tool_offset=(-0.34, -0.24, 3.25),
        resin_z_print_compensation_mm=-33.3,
        tool_change_safe_lift_mm=10.0,
        cut_lift_mm=12.0,
        cut_wait_s=12.0,
    )

    export_npz(_commands(), str(source), **base)
    export_npz(_commands(), str(direct), **changed)
    inject_npz(
        source,
        injected,
        tool_offset=changed["tool_offset"],
        resin_z_print_compensation_mm=changed["resin_z_print_compensation_mm"],
        tool_change_safe_lift_mm=changed["tool_change_safe_lift_mm"],
        cut_lift_mm=changed["cut_lift_mm"],
        cut_wait_s=changed["cut_wait_s"],
    )

    with np.load(direct, allow_pickle=False) as direct_data, np.load(
        injected, allow_pickle=False
    ) as injected_data, np.load(source, allow_pickle=False) as source_data:
        assert direct_data.files == injected_data.files
        for key in direct_data.files:
            assert direct_data[key].dtype == injected_data[key].dtype, key
            assert direct_data[key].shape == injected_data[key].shape, key
            assert np.array_equal(direct_data[key], injected_data[key]), key


def test_fiber_offset_change_keeps_resin_return_target_and_print_rows_fixed(tmp_path):
    source = tmp_path / "minimal_source.npz"
    offset_325 = tmp_path / "offset_325.npz"
    offset_305 = tmp_path / "offset_305.npz"
    base = dict(
        dt=0.004,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
        tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=-33.3,
        tool_change_safe_lift_mm=20.0,
        cut_lift_mm=20.0,
        cut_wait_s=15.0,
        external_npz_cut_absolute_e=True,
    )
    common = dict(
        resin_z_print_compensation_mm=-33.3,
        tool_change_safe_lift_mm=10.0,
        cut_lift_mm=12.0,
        cut_wait_s=12.0,
    )

    export_npz(_two_way_tool_change_commands(), str(source), **base)
    _use_from_to_tool_manifest(source)
    inject_npz(source, offset_325, tool_offset=(-0.34, -0.24, 3.25), **common)
    inject_npz(source, offset_305, tool_offset=(-0.34, -0.24, 3.05), **common)

    with np.load(offset_325, allow_pickle=False) as old, np.load(
        offset_305, allow_pickle=False
    ) as new:
        manifest_raw = old["core_injection_manifest"].item()
        if isinstance(manifest_raw, bytes):
            manifest_raw = manifest_raw.decode("utf-8")
        manifest = json.loads(str(manifest_raw))
        resin_return = next(
            block
            for block in manifest["blocks"]
            if block.get("kind") == "tool_change" and block.get("to_tool") == 2
        )
        assert "target_tool_id" not in resin_return

        roles = {
            key.decode().rstrip("\x00"): int(value)
            for key, value in zip(
                old["core_injection_role_vocab_keys"],
                old["core_injection_role_vocab_vals"],
            )
        }
        moves = {
            key.decode().rstrip("\x00"): int(value)
            for key, value in zip(old["move_type_vocab_keys"], old["move_type_vocab_vals"])
        }
        return_event = np.flatnonzero(
            (old["core_injection_block_id"] == int(resin_return["id"]))
            & (old["core_injection_role"] == roles["tool_change_event"])
        )
        assert len(return_event) == 1
        assert float(new["z"][return_event[0]]) == pytest.approx(
            float(old["z"][return_event[0]]), abs=1e-6
        )

        print_mask = (old["event_flag"] == 0) & np.isin(
            old["move_type"], [moves["PRINT"], moves["PRINT_FIT"]]
        )
        resin_print = print_mask & (old["tool_id"] == 2)
        fiber_print = print_mask & (old["tool_id"] == 1)
        assert np.array_equal(new["z"][resin_print], old["z"][resin_print])
        assert np.allclose(
            new["z"][fiber_print] - old["z"][fiber_print],
            -0.2,
            rtol=0.0,
            atol=2e-6,
        )


def test_random_multilayer_injection_matches_full_export_after_fiber_z_only_change(
    tmp_path,
):
    full_325 = tmp_path / "random_full_325.npz"
    full_305 = tmp_path / "random_full_305.npz"
    injected_305 = tmp_path / "random_injected_305.npz"
    common = dict(
        dt=0.004,
        default_feed_mm_s=10.0,
        enable_extrude_wait=True,
        resin_z_print_compensation_mm=-33.3,
        tool_change_safe_lift_mm=10.0,
        cut_lift_mm=12.0,
        cut_wait_s=12.0,
        external_npz_cut_absolute_e=True,
    )

    export_npz(
        _random_multilayer_switch_commands(),
        str(full_325),
        tool_offset=(-0.34, -0.24, 3.25),
        **common,
    )
    export_npz(
        _random_multilayer_switch_commands(),
        str(full_305),
        tool_offset=(-0.34, -0.24, 3.05),
        **common,
    )
    inject_npz(
        full_325,
        injected_305,
        tool_offset=(-0.34, -0.24, 3.05),
        resin_z_print_compensation_mm=-33.3,
        tool_change_safe_lift_mm=10.0,
        cut_lift_mm=12.0,
        cut_wait_s=12.0,
    )

    with np.load(full_325, allow_pickle=False) as old, np.load(
        full_305, allow_pickle=False
    ) as expected, np.load(injected_305, allow_pickle=False) as actual:
        assert expected.files == actual.files
        for key in expected.files:
            assert expected[key].dtype == actual[key].dtype, key
            assert expected[key].shape == actual[key].shape, key
            if key in {"x64", "y64", "z64"}:
                assert np.allclose(
                    expected[key], actual[key], rtol=0.0, atol=1e-12
                ), key
            else:
                assert np.array_equal(expected[key], actual[key]), key

        moves = {
            key.decode().rstrip("\x00"): int(value)
            for key, value in zip(old["move_type_vocab_keys"], old["move_type_vocab_vals"])
        }
        print_mask = (old["event_flag"] == 0) & np.isin(
            old["move_type"], [moves["PRINT"], moves["PRINT_FIT"]]
        )
        resin_print = print_mask & (old["tool_id"] == 2)
        fiber_print = print_mask & (old["tool_id"] == 1)
        assert resin_print.any()
        assert fiber_print.any()
        assert np.array_equal(expected["z"][resin_print], old["z"][resin_print])
        assert np.allclose(
            expected["z"][fiber_print] - old["z"][fiber_print],
            -0.2,
            rtol=0.0,
            atol=2e-6,
        )

        raw = expected["core_injection_manifest"].item()
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8")
        manifest = json.loads(str(raw))
        tool_blocks = [
            block for block in manifest["blocks"] if block.get("kind") == "tool_change"
        ]
        assert [int(block["target_tool_id"]) for block in tool_blocks] == [1, 2, 1, 2]
        assert np.array_equal(np.unique(expected["layer_index"][print_mask]), np.arange(5))
