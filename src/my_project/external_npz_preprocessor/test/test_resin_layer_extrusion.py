"""Only entity-layer boundary E distances are locally adjustable."""
import json
import math

import numpy as np
import pytest

from external_npz_preprocessor.converter import source_job_to_parsed_commands
from external_npz_preprocessor.process_params import ProcessParams, ResinProcessParams
from external_npz_preprocessor.source_npz import LayerPaths, MaterialPath, SourceJob
from path_processing_core.local_injector import LocalInjectionParams, inject_npz
from path_processing_core.npz_exporter import export_npz
from path_processing_core.types import ExtrudeWait, GlobalCurveCommand, MoveCommand, Position


def _read(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def _manifest(data):
    return json.loads(str(data["core_injection_manifest"].item()))


def _source(tmp_path):
    layers = []
    for layer in range(2):
        paths = [MaterialPath("R", index, np.array([
            [0, index * 2, 0.5 + layer * 0.5, 0, 0, 0],
            [4, index * 2, 0.5 + layer * 0.5, 0, 0, 0],
        ], dtype=np.float32)) for index in range(3)]
        fiber = MaterialPath("F", 3, np.array([
            [0, 0, 0.5 + layer * 0.5, 0, 0, 0],
            [4, 0, 0.5 + layer * 0.5, 0, 0, 0],
        ], dtype=np.float32))
        layers.append(LayerPaths(index=layer, resin_paths=paths, fiber_paths=[fiber]))
    commands = source_job_to_parsed_commands(
        SourceJob(meta={}, layers=layers),
        ProcessParams(resin=ResinProcessParams(prime_length_mm=2, prime_speed_mm_s=10,
                                               retract_length_mm=1, retract_speed_mm_s=5),
                      primeline_length_mm=5, prime_settle_s=0.04),
    )
    source = tmp_path / "source.npz"
    export_npz(commands, str(source), enable_extrude_wait=True,
               tool_change_safe_lift_mm=2, cut_lift_mm=1, cut_wait_s=0.004)
    return source, commands


def _rows(data, line):
    return np.flatnonzero((data["src_line"] == str(line).encode()) & (data["event_flag"] == 0))


def _delta(data, line):
    rows = _rows(data, line)
    assert len(rows) and rows[0] > 0
    return float(data["e64"][rows[-1]] - data["e64"][rows[0] - 1])


@pytest.mark.parametrize("mode", ["memory", "disk"])
def test_boundary_override_excludes_primeline_and_preserves_other_extrusion(tmp_path, mode):
    source, commands = _source(tmp_path)
    original = _read(source)
    entries = _manifest(original)["resin_extrusion_boundaries"]["entries"]
    assert len(entries) == 4 and all(item["available"] for item in entries)
    targets = {entry["source_line"]: entry for entry in entries}
    ready = tmp_path / "ready.npz"
    inject_npz(source, ready, resin_layer_start_prime_mm=3,
               resin_layer_end_retract_mm=0.5, storage_mode=mode)
    updated = _read(ready)
    for cmd in commands:
        if isinstance(cmd, ExtrudeWait):
            if cmd.raw == "external_npz_reset_anchor":
                assert np.all(updated["e64"][_rows(updated, cmd.line)] == 0.0)
                continue
            expected = (3 if cmd.delta_e > 0 else -0.5) if cmd.line in targets else cmd.delta_e
            assert _delta(updated, cmd.line) == pytest.approx(expected, abs=1e-8)
            if cmd.line in targets:
                speed = targets[cmd.line]["speed_mm_s"]
                assert len(_rows(updated, cmd.line)) == math.ceil(abs(expected) / speed / 0.004)
        elif isinstance(cmd, GlobalCurveCommand):
            # Absolute E may shift, but the deposited E increments and full pose stay identical.
            before, after = _rows(original, cmd.line), _rows(updated, cmd.line)
            assert len(before) == len(after)
            for key in ("x64", "y64", "z64", "a64", "b64", "c64"):
                np.testing.assert_array_equal(original[key][before], updated[key][after])
            old_e = np.r_[original["e64"][before[0] - 1], original["e64"][before]]
            new_e = np.r_[updated["e64"][after[0] - 1], updated["e64"][after]]
            np.testing.assert_allclose(np.diff(old_e), np.diff(new_e), rtol=0, atol=1e-12)
    assert np.all(np.diff(updated["seq"]) == 1)
    assert np.all(np.diff(updated["planned_time_s"]) >= 0)
    np.testing.assert_array_equal(updated["e"], updated["e64"].astype(np.float32))
    # Same overrides must not accumulate E or timing changes.
    again = tmp_path / "again.npz"
    inject_npz(ready, again, resin_layer_start_prime_mm=3,
               resin_layer_end_retract_mm=0.5, storage_mode=mode)
    repeated = _read(again)
    for key in updated:
        if key != "core_injection_manifest":
            np.testing.assert_array_equal(updated[key], repeated[key], err_msg=key)


def test_blank_preserves_original_and_zero_can_be_reenabled(tmp_path):
    source, _ = _source(tmp_path)
    ready = tmp_path / "ready.npz"
    inject_npz(source, ready, tool_change_safe_lift_mm=2,
               resin_layer_end_retract_mm=None, resin_layer_start_prime_mm=None)
    before, after = _read(source), _read(ready)
    for key in before:
        if key != "core_injection_manifest":
            np.testing.assert_array_equal(before[key], after[key], err_msg=key)
    inject_npz(ready, ready, resin_layer_start_prime_mm=0, resin_layer_end_retract_mm=0)
    zero = _read(ready)
    for entry in _manifest(zero)["resin_extrusion_boundaries"]["entries"]:
        assert _delta(zero, entry["source_line"]) == pytest.approx(0)
        assert len(_rows(zero, entry["source_line"])) == 1
    inject_npz(ready, ready, resin_layer_start_prime_mm=2, resin_layer_end_retract_mm=1)
    restored = _read(ready)
    for entry in _manifest(restored)["resin_extrusion_boundaries"]["entries"]:
        expected = 2 if entry["parameter"] == "resin_layer_start_prime_mm" else -1
        assert _delta(restored, entry["source_line"]) == pytest.approx(expected)


def test_missing_entity_provenance_rejects_override_without_overwrite(tmp_path):
    source, _ = _source(tmp_path)
    data = _read(source)
    manifest = _manifest(data)
    manifest.pop("resin_extrusion_boundaries")
    data["core_injection_manifest"] = np.asarray(json.dumps(manifest))
    np.savez_compressed(source, **data)
    before = source.read_bytes()
    with pytest.raises(ValueError, match="primeline"):
        inject_npz(source, source, resin_layer_start_prime_mm=3)
    assert source.read_bytes() == before


def test_gcode_primeline_cloned_waits_with_same_line_numbers_are_not_overridden(tmp_path):
    from gcode_planner.primeline import insert_resin_primeline

    p0, p1 = Position(0, 0, 0.5, 0, 0, 0), Position(4, 0, 0.5, 0, 0, 0)
    commands = insert_resin_primeline([
        MoveCommand("TRAVEL", "G0", p0, p0, 0, 0, 600, 1),
        ExtrudeWait("EXTRUDE_WAIT", 0.2, 2, 600, 2),
        MoveCommand("PRINT", "G1", p0, p1, 4, 2, 600, 3, subtype="RESIN_PRINT"),
        ExtrudeWait("EXTRUDE_WAIT", 0.2, -1, 300, 4),
    ], length_mm=5)
    source = tmp_path / "gcode.npz"
    export_npz(commands, str(source), enable_extrude_wait=True)
    inject_npz(source, source, resin_layer_start_prime_mm=3, resin_layer_end_retract_mm=0.5)
    data = _read(source)
    # The inserted primeline intentionally clones the same prime/retract source lines.
    for line, expected in ((2, [2, 3]), (4, [-1, -0.5])):
        indices = _rows(data, line)
        groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1) + 1)
        deltas = [float(data["e64"][group[-1]] - data["e64"][group[0] - 1]) for group in groups]
        assert deltas == pytest.approx(expected)


@pytest.mark.parametrize("parameter", ["resin_layer_start_prime_mm", "resin_layer_end_retract_mm"])
def test_independent_override_keeps_other_boundary_original(tmp_path, parameter):
    source, _ = _source(tmp_path)
    inject_npz(source, source, **{parameter: 4})
    data = _read(source)
    for entry in _manifest(data)["resin_extrusion_boundaries"]["entries"]:
        positive = entry["parameter"] == "resin_layer_start_prime_mm"
        expected = (4 if positive else -4) if entry["parameter"] == parameter else (2 if positive else -1)
        assert _delta(data, entry["source_line"]) == pytest.approx(expected)


@pytest.mark.parametrize("parameter", ["resin_layer_start_prime_mm", "resin_layer_end_retract_mm"])
@pytest.mark.parametrize("value", [-1, float("nan"), float("inf")])
def test_invalid_boundary_values_rejected(parameter, value):
    with pytest.raises(ValueError):
        LocalInjectionParams(**{parameter: value}).as_overrides()
