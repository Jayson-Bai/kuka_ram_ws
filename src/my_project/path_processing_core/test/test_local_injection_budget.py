"""Budget accounting must not change the generated injection trajectory."""

import json

import numpy as np
import pytest

from path_processing_core import local_injector as injector
from path_processing_core.npz_exporter import export_npz
from test_minimal_dual_material_equivalence import _commands


def _estimate(arrays, manifest, **overrides):
    values = dict(
        tool_changed=False, resin_changed=False, delta_resin=0.0,
            safe=10.0, lift=12.5, wait=12.0, settle=3.0,
            new_offset=(0.0, 0.0, 0.0),
        feed=20.0, dt=0.004, roles={41: "cut_event", 42: "cut_action"},
    )
    values.update(overrides)
    return injector._estimate_injected_rows(arrays, manifest, **values)


def _cut_arrays(counts, total=None):
    size = total or sum(count + 1 for count in counts)
    arrays = {
        "seq": np.arange(size, dtype=np.uint32),
        "core_injection_block_id": np.full(size, -1, dtype=np.int32),
        "core_injection_role": np.zeros(size, dtype=np.uint8),
        "event_flag": np.zeros(size, dtype=np.uint8),
        "src_line": np.full(size, b"7", dtype="S8"),
    }
    blocks = []
    start = 0
    for bid, count in enumerate(counts, start=1):
        end = start + count + 1
        arrays["core_injection_block_id"][start:end] = bid
        arrays["core_injection_role"][start:end] = 42
        arrays["core_injection_role"][start] = 41
        arrays["event_flag"][start] = 1
        blocks.append({"id": bid, "kind": "cut", "source_line": 7})
        start = end
    return arrays, {"blocks": blocks, "base_parameters": {
        "cut_lift_mm": 20.0, "cut_wait_s": 15.0,
    }}


def test_many_existing_cuts_do_not_double_count_source_rows():
    # Field NPZ: 3,110,069 source rows, 360 CUTs, 4,503 action rows each.
    arrays, manifest = _cut_arrays([4503] * 360, total=3_110_069)
    assert len(arrays["seq"]) + 360 * 3000 > injector._MAX_ESTIMATED_ROWS
    assert _estimate(arrays, manifest) == len(arrays["seq"])
    assert _estimate(arrays, manifest, wait=120.0) > injector._MAX_ESTIMATED_ROWS


def test_shrinking_cut_cannot_fund_another_cut_growth():
    arrays, manifest = _cut_arrays([4503, 100])
    assert _estimate(arrays, manifest) == len(arrays["seq"]) + 2900


def test_unchanged_cut_parameters_do_not_add_rebuild_budget():
    arrays, manifest = _cut_arrays([100])
    assert _estimate(arrays, manifest, lift=20.0, wait=15.0) == len(arrays["seq"])


def test_asymmetric_cut_holds_are_summed_in_budget_estimate():
    arrays, manifest = _cut_arrays([100])
    expected_rows = int(np.ceil((2.0 * 12.5 / 20.0 + 10.0 + 1.0) / 0.004))
    assert _estimate(
        arrays, manifest, settle=10.0, retract_settle=1.0,
    ) == len(arrays["seq"]) + expected_rows - 100


def test_duplicate_catalog_entry_cannot_reuse_replacement_credit():
    arrays, manifest = _cut_arrays([4503])
    manifest["blocks"].append(dict(manifest["blocks"][0]))
    assert _estimate(arrays, manifest) == len(arrays["seq"]) + 3000


def test_budget_keeps_events_connectors_and_other_source_rows():
    arrays, manifest = _cut_arrays([100])
    arrays["event_flag"][1] = 1
    arrays["core_injection_role"][2] = 9  # retained connector/anchor
    arrays["src_line"][3] = b"8"  # retained following command
    # A same-source action before the CUT is retained too.
    arrays["core_injection_role"][0] = 42
    arrays["event_flag"][0] = 0
    arrays["core_injection_role"][4] = 41
    arrays["event_flag"][4] = 1
    assert _estimate(arrays, manifest) == len(arrays["seq"]) + 3000 - 96


def test_ambiguous_or_missing_markers_do_not_receive_credit():
    arrays, manifest = _cut_arrays([100])
    arrays["core_injection_role"][1] = 41
    assert _estimate(arrays, manifest) == len(arrays["seq"]) + 3000
    del arrays["src_line"]
    assert _estimate(arrays, manifest) == len(arrays["seq"]) + 3000


def test_actual_replacement_limit_is_checked_before_allocating(monkeypatch):
    arrays = {"seq": np.arange(5, dtype=np.uint32)}
    original = arrays["seq"]
    monkeypatch.setattr(injector, "_MAX_ESTIMATED_ROWS", 5)
    monkeypatch.setattr(injector, "_rows_array", lambda *args: pytest.fail("allocated"))
    with pytest.raises(ValueError, match="replacement would produce 6 rows > 5"):
        injector._replace(arrays, 0, 1, [{"seq": 0}, {"seq": 1}])
    assert arrays["seq"] is original


def test_replacement_at_limit_preserves_exact_rows(monkeypatch):
    arrays = {"seq": np.arange(5, dtype=np.uint32)}
    monkeypatch.setattr(injector, "_MAX_ESTIMATED_ROWS", 5)
    injector._replace(arrays, 1, 3, [{"seq": 10}, {"seq": 11}])
    np.testing.assert_array_equal(arrays["seq"], [0, 10, 11, 3, 4])


def test_budget_fix_preserves_all_injection_output_fields(tmp_path, monkeypatch):
    source = tmp_path / "base.npz"
    checked = tmp_path / "checked.npz"
    reference = tmp_path / "reference.npz"
    export_npz(
        _commands(), str(source), dt=0.004, default_feed_mm_s=20.0,
        enable_extrude_wait=True, tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=0.0, tool_change_safe_lift_mm=20.0,
        cut_lift_mm=20.0, cut_wait_s=15.0, external_npz_cut_absolute_e=True,
    )
    params = dict(
        tool_offset=(-0.34, -0.24, 3.25), resin_z_print_compensation_mm=-34.0,
        tool_change_safe_lift_mm=10.0, cut_lift_mm=12.5, cut_wait_s=12.0,
    )
    with pytest.raises(ValueError, match="safe in-memory row budget"):
        injector.inject_npz(source, checked, **dict(params, cut_wait_s=100_000.0))
    assert not checked.exists()
    injector.inject_npz(source, checked, **params)
    monkeypatch.setattr(injector, "_estimate_injected_rows", lambda *a, **kw: 0)
    injector.inject_npz(source, reference, **params)
    with np.load(checked, allow_pickle=False) as actual, np.load(
        reference, allow_pickle=False
    ) as expected:
        assert actual.files == expected.files
        for key in actual.files:
            if key == "core_injection_manifest":
                a, b = (json.loads(str(data[key].item())) for data in (actual, expected))
                a.pop("injected_at")
                b.pop("injected_at")
                assert a == b
            else:
                np.testing.assert_array_equal(actual[key], expected[key], err_msg=key)
