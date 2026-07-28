"""Regression test for the minimal external-NPZ mixed-material contract."""

import importlib.util
from pathlib import Path

import numpy as np

from path_processing_core.local_injector import inject_npz
from path_processing_core.npz_exporter import export_npz


_ROOT = Path(__file__).resolve().parents[4]
_TEMPLATE = _ROOT / "data/input_gcode/resin_0728_minimal_dual_core_template.py"


def _commands():
    spec = importlib.util.spec_from_file_location("minimal_dual_template", _TEMPLATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.commands()


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
        assert _event_contract(direct_data) == _event_contract(injected_data)
        # Local injection must not create a larger pose discontinuity than the
        # original Core-sampled source, even when all UI parameters change.
        assert _max_xyz_step(injected_data) <= _max_xyz_step(source_data) + 1e-6
        assert np.all(np.diff(injected_data["seq"].astype(np.int64)) == 1)
        assert np.all(np.diff(injected_data["planned_time_s"]) >= -1e-6)
