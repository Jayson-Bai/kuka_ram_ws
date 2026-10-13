"""Storage modes must preserve the complete serialized execution payload."""
from datetime import datetime, timezone
import hashlib

import numpy as np
import pytest

from path_processing_core import local_injector as injector
from path_processing_core import npz_workspace as storage
from path_processing_core.npz_exporter import export_npz
from path_processing_core.rsi_validation import validate_final_npz
from test_minimal_dual_material_equivalence import _commands


class _Clock:
    @staticmethod
    def now(tz=None):
        return datetime(2026, 10, 13, tzinfo=timezone.utc)


def _export(path, *, chunk_size=100000, curved=False):
    commands = _commands()
    if curved:
        for command in commands:
            for attribute in ("pos", "start_pos"):
                position = getattr(command, attribute, None)
                if position is not None:
                    position.b = 12.0
                    position.c = -8.0
    export_npz(
        commands, str(path), dt=0.004, default_feed_mm_s=20.0,
        enable_extrude_wait=True, tool_offset=(0.0, 0.0, 0.0),
        resin_z_print_compensation_mm=0.0, tool_change_safe_lift_mm=20.0,
        cut_lift_mm=20.0, cut_wait_s=15.0, external_npz_cut_absolute_e=True,
        chunk_size=chunk_size,
    )


PARAMS = dict(
    tool_offset=(-0.34, -0.24, 3.25), resin_z_print_compensation_mm=-34.0,
    tool_change_safe_lift_mm=10.0, cut_lift_mm=12.5, cut_wait_s=12.0,
)


def _assert_identical(left, right):
    left_parts, right_parts = injector._part_files(left), injector._part_files(right)
    assert len(left_parts) == len(right_parts)
    for a, b in zip(left_parts, right_parts):
        assert hashlib.sha256(a.read_bytes()).digest() == hashlib.sha256(b.read_bytes()).digest()
    for suffix in (".offset.json", ".timing.json"):
        assert left.with_suffix(suffix).read_bytes() == right.with_suffix(suffix).read_bytes()


@pytest.mark.parametrize("curved", [False, True])
@pytest.mark.parametrize("chunk_size", [100000, 700])
def test_npz_and_sidecars_are_byte_identical_across_storage_modes(
    tmp_path, monkeypatch, curved, chunk_size,
):
    monkeypatch.setattr(injector, "datetime", _Clock)
    source = tmp_path / "base.npz"
    memory, disk = tmp_path / "memory.npz", tmp_path / "disk.npz"
    _export(source, curved=curved, chunk_size=chunk_size)
    injector.inject_npz(source, memory, storage_mode="memory", **PARAMS)
    injector.inject_npz(source, disk, storage_mode="disk", **PARAMS)
    _assert_identical(memory, disk)
    assert validate_final_npz(memory)["ok"]
    assert validate_final_npz(disk)["ok"]
    # A second injection must preserve precision mirrors and existing bridges.
    second_memory, second_disk = tmp_path / "second-memory.npz", tmp_path / "second-disk.npz"
    params = dict(PARAMS, tool_offset=(-0.34, -0.24, 3.05))
    injector.inject_npz(memory, second_memory, storage_mode="memory", **params)
    injector.inject_npz(disk, second_disk, storage_mode="disk", **params)
    _assert_identical(second_memory, second_disk)


def test_auto_uses_uncompressed_size_before_loading(tmp_path, monkeypatch):
    source = tmp_path / "base.npz"
    _export(source)
    monkeypatch.setattr(storage, "IN_MEMORY_BYTES", 1)
    monkeypatch.setattr(injector, "_MAX_ESTIMATED_ROWS", 1)
    result = injector.inject_npz(source, tmp_path / "ready.npz", **PARAMS)
    assert result["rows"] > 1
    with pytest.raises(ValueError, match="safe in-memory row budget"):
        injector.inject_npz(source, tmp_path / "ram.npz", storage_mode="memory", **PARAMS)


def test_auto_spills_if_rebuild_budget_exceeds_ram_limit(tmp_path, monkeypatch):
    source = tmp_path / "base.npz"
    _export(source)
    # Load the small source into RAM, then discover it exceeds the row budget.
    monkeypatch.setattr(storage, "IN_MEMORY_BYTES", 10 ** 12)
    monkeypatch.setattr(injector, "_MAX_ESTIMATED_ROWS", 1)
    assert injector.inject_npz(source, tmp_path / "ready.npz", **PARAMS)["rows"] > 1


@pytest.mark.parametrize("raise_error", [False, True])
def test_workspace_closes_and_removes_all_temporary_arrays(raise_error):
    workspace = storage.NpzWorkspace()
    root = workspace.root
    try:
        with workspace:
            arrays = storage.DiskArrays(workspace)
            arrays["seq"] = np.arange(6, dtype=np.uint32)
            injector._replace(arrays, 1, 3, [{"seq": 20}])
            np.testing.assert_array_equal(arrays["seq"], [0, 20, 3, 4, 5])
            if raise_error:
                raise RuntimeError("injected failure")
    except RuntimeError:
        assert raise_error
    assert not root.exists()
    assert not workspace.mappings


def test_disk_shortage_fails_before_touching_input_or_output(tmp_path, monkeypatch):
    source, output = tmp_path / "base.npz", tmp_path / "ready.npz"
    _export(source)
    original = source.read_bytes()
    output.write_bytes(b"existing output")
    def no_space(self, required):
        raise ValueError("temporary disk budget exceeded")
    monkeypatch.setattr(storage.NpzWorkspace, "check_space", no_space)
    with pytest.raises(ValueError, match="disk budget"):
        injector.inject_npz(source, output, storage_mode="disk", **PARAMS)
    assert source.read_bytes() == original
    assert output.read_bytes() == b"existing output"


def test_disk_mode_still_limits_single_huge_synthetic_block(tmp_path):
    with storage.NpzWorkspace() as workspace:
        arrays = storage.DiskArrays(workspace)
        arrays["seq"] = np.arange(1, dtype=np.uint32)
        with pytest.raises(ValueError, match="synthetic block.*row budget"):
            injector._wait_rows(
                arrays, 0, np.zeros(6), 0.0, 0.0, 1e9,
                block_id=1, role_code=1, print_code=1, dt=0.004,
            )


def test_in_place_disk_splices_match_numpy_concatenation():
    rng = np.random.default_rng(17)
    reference = {"seq": np.arange(20, dtype=np.uint32), "x": rng.normal(size=20).astype(np.float32)}
    with storage.NpzWorkspace() as workspace:
        disk = storage.DiskArrays(workspace)
        for key, value in reference.items():
            disk[key] = value.copy()
        for count in [35, 0, 18, 2, 0, 43, 7, 0] * 3:
            length = len(reference["seq"])
            start = int(rng.integers(0, length + 1))
            end = int(rng.integers(start, length + 1))
            rows = [{"seq": np.uint32(i), "x": np.float32(rng.normal())} for i in range(count)]
            injector._replace(reference, start, end, rows)
            injector._replace(disk, start, end, rows)
            for key in reference:
                assert reference[key].dtype == disk[key].dtype
                assert reference[key].tobytes() == disk[key].tobytes()


def test_non_native_endian_input_keeps_legacy_output_bytes(tmp_path, monkeypatch):
    monkeypatch.setattr(injector, "datetime", _Clock)
    source = tmp_path / "base.npz"
    _export(source)
    with np.load(source, allow_pickle=False) as data:
        payload = {key: data[key] for key in data.files}
    for key, value in payload.items():
        if value.dtype.kind in "uif" and value.dtype.itemsize > 1 and not key.endswith("64"):
            payload[key] = value.astype(value.dtype.newbyteorder(">"))
    np.savez_compressed(source, **payload)
    memory, disk = tmp_path / "ram.npz", tmp_path / "disk.npz"
    injector.inject_npz(source, memory, storage_mode="memory", **PARAMS)
    injector.inject_npz(source, disk, storage_mode="disk", **PARAMS)
    _assert_identical(memory, disk)


def test_disk_reader_matches_memory_precision_rejection(tmp_path):
    source = tmp_path / "base.npz"
    _export(source)
    with np.load(source, allow_pickle=False) as data:
        payload = {key: data[key] for key in data.files}
    # This contract is rejected on every supported NumPy version.
    payload["x64"] = payload["x64"].astype(np.float32)
    np.savez_compressed(source, **payload)
    for mode in ("memory", "disk"):
        with pytest.raises(ValueError, match="x64 must use float64"):
            injector.inject_npz(source, tmp_path / (mode + ".npz"), storage_mode=mode, **PARAMS)


def test_sampling_buffer_guard_is_scoped_to_workspace(monkeypatch):
    monkeypatch.setattr(storage, "REBUILD_BYTES", 512)
    with storage.NpzWorkspace():
        with pytest.raises(ValueError, match="sampling buffer.*row budget"):
            storage.materialize_samples(iter(range(100)))
    assert storage.materialize_samples(iter(range(100))) == list(range(100))


def test_disk_storage_rejects_seq_overflow_before_replacement():
    with storage.NpzWorkspace() as workspace:
        arrays = storage.DiskArrays(workspace)
        arrays["seq"] = np.arange(2, dtype=np.uint8)
        with pytest.raises(ValueError, match="seq dtype capacity"):
            arrays.check_size(257)
