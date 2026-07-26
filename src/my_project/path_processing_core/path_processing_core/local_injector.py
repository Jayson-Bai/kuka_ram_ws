"""局部修改已导出 Core NPZ 的现场注入器。

该模块不读取 GCode、STL 或源 NPZ，也不调用 Core exporter。它只在已有
``core_npz_local_injection_v1`` 输出上做数组级变换和标记 block 的局部重建，
最后用同目录临时文件原子替换输出。
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import tempfile
from typing import Any

import numpy as np


_PART_RE = re.compile(r"^(?P<base>.+)_part(?P<part>\d{4})$")
_REQUIRED = {
    "seq", "x", "y", "z", "a", "b", "c", "e", "tool_id", "move_type",
    "event_flag", "event_type", "payload", "trigger_seq", "layer_index",
    "path_id", "path_end_flag", "planned_time_s",
    "core_injection_manifest", "core_injection_block_id", "core_injection_role",
}


@dataclass(frozen=True)
class LocalInjectionParams:
    tool_offset: tuple[float, float, float] | None = None
    resin_z_print_compensation_mm: float | None = None
    tool_change_safe_lift_mm: float | None = None
    cut_lift_mm: float | None = None
    cut_wait_s: float | None = None

    def as_overrides(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        if self.tool_offset is not None:
            if len(self.tool_offset) != 3:
                raise ValueError("tool_offset must contain exactly 3 values")
            out["tool_offset"] = [float(v) for v in self.tool_offset]
        for name in (
            "resin_z_print_compensation_mm",
            "tool_change_safe_lift_mm",
            "cut_lift_mm",
            "cut_wait_s",
        ):
            value = getattr(self, name)
            if value is not None:
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite")
                out[name] = value
        return out


def _part_files(path: Path) -> list[Path]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    if path.is_dir():
        candidates = list(path.glob("*.npz"))
        candidates.sort(key=lambda p: (_part_sort_key(p), p.name))
        return candidates
    match = _PART_RE.match(path.stem)
    if match:
        candidates = list(path.parent.glob(f"{match.group('base')}_part*.npz"))
        candidates.sort(key=lambda p: (_part_sort_key(p), p.name))
        return candidates
    return [path]


def _part_sort_key(path: Path) -> int:
    match = _PART_RE.match(path.stem)
    return int(match.group("part")) if match else -1


def _read_parts(files: list[Path]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    if not files:
        raise ValueError("no NPZ parts found")
    chunks: list[dict[str, np.ndarray]] = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            chunks.append({key: data[key].copy() for key in data.files})
    missing = sorted(_REQUIRED - set(chunks[0]))
    if missing:
        raise ValueError(
            "NPZ is not a core_npz_local_injection_v1 file; missing fields: "
            + ", ".join(missing)
        )
    row_keys = [key for key, value in chunks[0].items() if value.ndim == 1 and len(value) == len(chunks[0]["seq"])]
    arrays: dict[str, np.ndarray] = {}
    for key in row_keys:
        arrays[key] = np.concatenate([chunk[key] for chunk in chunks])
    static = {key: value for key, value in chunks[0].items() if key not in row_keys}
    return arrays, static


def _json_scalar(value: np.ndarray) -> dict[str, Any]:
    if value.shape != ():
        raise ValueError("core_injection_manifest must be a 0-D JSON string")
    try:
        manifest = json.loads(str(value.item()))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("invalid core_injection_manifest JSON") from exc
    if not isinstance(manifest, dict) or manifest.get("format") != "core_npz_local_injection_v1":
        raise ValueError("unsupported core injection manifest format")
    return manifest


def _decode_vocab(keys: np.ndarray, vals: np.ndarray) -> dict[int, str]:
    return {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(keys, vals)
    }


def _finite_and_lengths(arrays: dict[str, np.ndarray]) -> None:
    n = len(arrays["seq"])
    for key, value in arrays.items():
        if value.ndim == 1 and len(value) != n:
            raise ValueError(f"NPZ field length mismatch: {key}")
    for key in ("x", "y", "z", "a", "b", "c", "e", "planned_time_s"):
        if not np.all(np.isfinite(arrays[key])):
            raise ValueError(f"NPZ field contains non-finite values: {key}")
    seq = arrays["seq"].astype(np.int64, copy=False)
    if n and (int(seq[0]) < 0 or not np.all(np.diff(seq) == 1)):
        raise ValueError("NPZ seq is not contiguous")


def _smootherstep(values: np.ndarray) -> np.ndarray:
    """Septic zero-jerk endpoint interpolation used for local linear blocks."""
    u = np.clip(values, 0.0, 1.0)
    return 35.0 * u**4 - 84.0 * u**5 + 70.0 * u**6 - 20.0 * u**7


def _ensure_tool_pre_rows(
    arrays: dict[str, np.ndarray],
    block_id: int,
    target_tool_id: int,
    new_offset: np.ndarray,
    new_safe_lift: float,
    sample_period_s: float,
    default_feed_mm_s: float,
    roles: dict[int, str],
) -> bool:
    role_names = np.vectorize(roles.get)(arrays["core_injection_role"])
    block = arrays["core_injection_block_id"] == block_id
    event = np.flatnonzero(block & (role_names == "tool_change_event"))
    pre = np.flatnonzero(block & (role_names == "tool_change_pre"))
    if event.size == 0 or pre.size:
        return False
    event_idx = int(event[0])
    if event_idx <= 0:
        return False
    previous_idx = event_idx - 1
    start = np.array([arrays[key][previous_idx] for key in ("x", "y", "z")], dtype=np.float64)
    sign = 1.0 if int(target_tool_id) == 1 else -1.0
    target = start + np.array(
        [sign * new_offset[0], sign * new_offset[1], new_safe_lift + sign * new_offset[2]],
        dtype=np.float64,
    )
    speed = max(float(default_feed_mm_s), 1e-9)
    distance = float(np.linalg.norm(target - start))
    steps = max(1, int(math.ceil(distance / max(speed * sample_period_s, 1e-9))))
    row_keys = [key for key, value in arrays.items() if value.ndim == 1]
    old_event_time = float(arrays["planned_time_s"][event_idx])
    previous_time = float(arrays["planned_time_s"][previous_idx])
    inserted: dict[str, np.ndarray] = {}
    u = np.linspace(0.0, 1.0, steps + 1, endpoint=True)[1:]
    poses = start[None, :] + _smootherstep(u)[:, None] * (target - start)[None, :]
    for key in row_keys:
        source = arrays[key][event_idx]
        values = np.repeat(np.asarray(source)[None], steps, axis=0)
        inserted[key] = values
    for column, key in enumerate(("x", "y", "z")):
        inserted[key] = poses[:, column].astype(arrays[key].dtype)
    inserted["tool_id"] = np.full(steps, arrays["tool_id"][previous_idx], dtype=arrays["tool_id"].dtype)
    inserted["move_type"] = np.full(steps, 0, dtype=arrays["move_type"].dtype)
    inserted["event_flag"] = np.zeros(steps, dtype=arrays["event_flag"].dtype)
    inserted["event_type"] = np.zeros(steps, dtype=arrays["event_type"].dtype)
    inserted["payload"] = np.zeros(steps, dtype=arrays["payload"].dtype)
    inserted["trigger_seq"] = np.full(steps, -1, dtype=arrays["trigger_seq"].dtype)
    inserted["path_end_flag"] = np.zeros(steps, dtype=arrays["path_end_flag"].dtype)
    inserted["core_injection_block_id"] = np.full(steps, block_id, dtype=arrays["core_injection_block_id"].dtype)
    # The role vocabulary is stable; the caller supplies the numeric code.
    inserted["core_injection_role"] = np.full(
        steps, next(code for code, name in roles.items() if name == "tool_change_pre"),
        dtype=arrays["core_injection_role"].dtype,
    )
    inserted["planned_time_s"] = np.asarray(
        [previous_time + sample_period_s * (i + 1) for i in range(steps)],
        dtype=arrays["planned_time_s"].dtype,
    )
    inserted["seq"] = np.zeros(steps, dtype=arrays["seq"].dtype)
    # Keep the source line/path metadata, but do not duplicate a path-end marker.
    for key in ("src_line", "path_id", "layer_index", "preview_layer_index", "total_layers", "e"):
        if key in inserted:
            inserted[key] = np.repeat(np.asarray(arrays[key][previous_idx])[None], steps, axis=0)
    extra_time = max(0.0, steps * sample_period_s - max(0.0, old_event_time - previous_time))
    for key in row_keys:
        arrays[key] = np.insert(arrays[key], event_idx, inserted[key], axis=0)
    if extra_time:
        arrays["planned_time_s"][event_idx + steps:] += np.asarray(extra_time, dtype=arrays["planned_time_s"].dtype)
    return True


def _apply_tool_block_rebuild(
    arrays: dict[str, np.ndarray],
    block_id: int,
    target_tool_id: int,
    base_offset: np.ndarray,
    new_offset: np.ndarray,
    new_safe_lift: float,
    roles: dict[int, str],
) -> None:
    block = arrays["core_injection_block_id"] == block_id
    if not np.any(block):
        return
    role = arrays["core_injection_role"]
    pre = np.flatnonzero(block & (np.vectorize(roles.get)(role) == "tool_change_pre"))
    event = np.flatnonzero(block & (np.vectorize(roles.get)(role) == "tool_change_event"))
    anchor = np.flatnonzero(block & (np.vectorize(roles.get)(role) == "post_anchor"))
    if event.size == 0:
        return
    event_idx = int(event[0])
    first_idx = int(np.flatnonzero(block)[0])
    previous_idx = first_idx - 1
    if previous_idx < 0:
        return
    start = np.array([arrays[key][previous_idx] for key in ("x", "y", "z")], dtype=np.float64)
    sign = 1.0 if int(target_tool_id) == 1 else -1.0
    target_event = start + np.array(
        [sign * new_offset[0], sign * new_offset[1], new_safe_lift + sign * new_offset[2]],
        dtype=np.float64,
    )
    if pre.size:
        u = np.linspace(0.0, 1.0, pre.size + 1, endpoint=True)[1:]
        values = start[None, :] + _smootherstep(u)[:, None] * (target_event - start)[None, :]
        for col, key in enumerate(("x", "y", "z")):
            arrays[key][pre] = values[:, col].astype(arrays[key].dtype)
    for key, value in zip(("x", "y", "z"), target_event):
        arrays[key][event_idx] = np.asarray(value, dtype=arrays[key].dtype)
    if anchor.size:
        anchor_idx = int(anchor[0])
        anchor_pose = np.array([arrays[key][anchor_idx] for key in ("x", "y", "z")], dtype=np.float64)
        post = np.flatnonzero(block & (np.arange(len(block)) > event_idx) & (np.arange(len(block)) < anchor_idx))
        if post.size:
            u = np.linspace(0.0, 1.0, post.size + 2, endpoint=True)[1:-1]
            values = target_event[None, :] + _smootherstep(u)[:, None] * (anchor_pose - target_event)[None, :]
            for col, key in enumerate(("x", "y", "z")):
                arrays[key][post] = values[:, col].astype(arrays[key].dtype)


def _apply_cut_block_rebuild(
    arrays: dict[str, np.ndarray],
    block_id: int,
    new_lift: float,
    new_wait: float,
    base_lift: float,
    base_wait: float,
    roles: dict[int, str],
) -> None:
    block = arrays["core_injection_block_id"] == block_id
    if not np.any(block):
        return
    role_names = np.vectorize(roles.get)(arrays["core_injection_role"])
    event = np.flatnonzero(block & (role_names == "cut_event"))
    action = np.flatnonzero(block & (role_names == "cut_action"))
    if event.size == 0 or action.size == 0:
        return
    event_idx = int(event[0])
    low_z = float(arrays["z"][event_idx])
    if base_lift > 1e-9:
        base_high = low_z + base_lift
        ratios = (arrays["z"][action].astype(np.float64) - low_z) / base_lift
        ratios = np.clip(ratios, 0.0, 1.0)
        arrays["z"][action] = (low_z + ratios * new_lift).astype(arrays["z"].dtype)
        # CUT lift extrusion is proportional to the configured lift. Preserve all
        # ordinary path E and only scale the marked safety interval.
        e0 = float(arrays["e"][event_idx])
        e_delta = arrays["e"][action].astype(np.float64) - e0
        scale = float(new_lift) / base_lift
        arrays["e"][action] = (e0 + e_delta * scale).astype(arrays["e"].dtype)
    if base_wait > 1e-9 and new_wait >= 0.0:
        block_rows = np.flatnonzero(block)
        start_time = float(arrays["planned_time_s"][block_rows[0]])
        end_time = float(arrays["planned_time_s"][block_rows[-1]])
        old_duration = max(0.0, end_time - start_time)
        # Keep the block's internal event ordering and shift its tail by the
        # wait delta; downstream rows receive the same cumulative shift.
        delta = float(new_wait - base_wait)
        after = np.arange(len(arrays["planned_time_s"])) > block_rows[-1]
        arrays["planned_time_s"][after] += delta
        arrays["planned_time_s"][block_rows] += np.linspace(0.0, delta, block_rows.size, dtype=np.float32)
        if old_duration > 0.0 and delta:
            arrays["planned_time_s"][block_rows] = np.maximum.accumulate(arrays["planned_time_s"][block_rows])


def _repair_indices(arrays: dict[str, np.ndarray]) -> None:
    arrays["seq"] = np.arange(len(arrays["seq"]), dtype=arrays["seq"].dtype)
    event_mask = arrays["event_flag"] != 0
    arrays["trigger_seq"] = np.where(
        event_mask,
        arrays["seq"],
        np.asarray(-1, dtype=arrays["trigger_seq"].dtype),
    ).astype(arrays["trigger_seq"].dtype)


def _update_manifest(manifest: dict[str, Any], overrides: dict[str, Any]) -> str:
    base = dict(manifest.get("base_parameters", {}))
    base.update(overrides)
    manifest["base_parameters"] = base
    return json.dumps(manifest, ensure_ascii=False, separators=(",", ":"))


def _output_parts(output_path: Path, count: int) -> list[Path]:
    match = _PART_RE.match(output_path.stem)
    if match:
        base = output_path.parent / match.group("base")
    else:
        base = output_path.with_suffix("")
    if count == 1:
        return [output_path if output_path.suffix == ".npz" else output_path.with_suffix(".npz")]
    return [Path(f"{base}_part{i:04d}.npz") for i in range(count)]


def _atomic_write(parts: list[Path], arrays: dict[str, np.ndarray], static: dict[str, np.ndarray], chunk_sizes: list[int]) -> None:
    manifest = static["core_injection_manifest"]
    output_paths = parts
    temporary: list[tuple[Path, Path]] = []
    offset = 0
    try:
        for index, (target, size) in enumerate(zip(output_paths, chunk_sizes)):
            target.parent.mkdir(parents=True, exist_ok=True)
            fd, raw = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
            Path(raw).unlink(missing_ok=True)
            temp = Path(raw)
            payload = {key: value[offset:offset + size] for key, value in arrays.items()}
            payload.update(static)
            with open(temp, "wb") as stream:
                np.savez_compressed(stream, **payload)
            temporary.append((temp, target))
            offset += size
        for temp, target in temporary:
            temp.replace(target)
    finally:
        for temp, _ in temporary:
            temp.unlink(missing_ok=True)


def inject_npz(
    input_path: str | Path,
    output_path: str | Path | None = None,
    *,
    params: LocalInjectionParams | None = None,
    tool_offset: tuple[float, float, float] | None = None,
    resin_z_print_compensation_mm: float | None = None,
    tool_change_safe_lift_mm: float | None = None,
    cut_lift_mm: float | None = None,
    cut_wait_s: float | None = None,
) -> dict[str, Any]:
    """Apply现场参数 to a final Core NPZ and atomically write it."""
    files = _part_files(Path(input_path).expanduser())
    arrays, static = _read_parts(files)
    _finite_and_lengths(arrays)
    manifest = _json_scalar(static["core_injection_manifest"])
    overrides = (params or LocalInjectionParams(
        tool_offset=tool_offset,
        resin_z_print_compensation_mm=resin_z_print_compensation_mm,
        tool_change_safe_lift_mm=tool_change_safe_lift_mm,
        cut_lift_mm=cut_lift_mm,
        cut_wait_s=cut_wait_s,
    )).as_overrides()
    base = manifest.get("base_parameters", {})
    if not overrides:
        raise ValueError("at least one local injection parameter is required")
    current_tool_offset = np.asarray(base.get("tool_offset", [0.0, 0.0, 0.0]), dtype=np.float64)
    new_tool_offset = np.asarray(overrides.get("tool_offset", current_tool_offset), dtype=np.float64)
    delta_tool = new_tool_offset - current_tool_offset
    current_resin_z = float(base.get("resin_z_print_compensation_mm", 0.0))
    new_resin_z = float(overrides.get("resin_z_print_compensation_mm", current_resin_z))
    role_vocab = _decode_vocab(
        static["core_injection_role_vocab_keys"],
        static["core_injection_role_vocab_vals"],
    )
    role_names = np.vectorize(role_vocab.get)(arrays["core_injection_role"])
    base_safe = float(base.get("tool_change_safe_lift_mm", 20.0))
    new_safe = float(overrides.get("tool_change_safe_lift_mm", base_safe))
    sample_period_s = float(manifest.get("sample_period_s", 0.004))
    default_feed_mm_s = float(base.get("default_feed_mm_s", 10.0))
    inserted_rows = False
    if "tool_offset" in overrides or "tool_change_safe_lift_mm" in overrides:
        for block in manifest.get("blocks", []):
            if block.get("kind") == "tool_change":
                inserted_rows = _ensure_tool_pre_rows(
                    arrays,
                    int(block.get("id", -1)),
                    int(block.get("target_tool_id", 1)),
                    new_tool_offset,
                    new_safe,
                    sample_period_s,
                    default_feed_mm_s,
                    role_vocab,
                ) or inserted_rows
        if inserted_rows:
            role_names = np.vectorize(role_vocab.get)(arrays["core_injection_role"])
    if np.any(delta_tool):
        mask = (arrays["tool_id"] == 1) & (role_names != "tool_change_pre")
        for key, delta in zip(("x", "y", "z"), delta_tool):
            arrays[key][mask] += np.asarray(delta, dtype=arrays[key].dtype)
    if abs(new_resin_z - current_resin_z) > 1e-12:
        anchor_candidates = np.flatnonzero(role_names == "post_anchor")
        if anchor_candidates.size:
            anchor = int(anchor_candidates[0])
        else:
            print_mask = arrays["move_type"] == 1
            anchor = int(np.flatnonzero(print_mask)[0]) if np.any(print_mask) else 0
        mask = np.arange(len(arrays["z"])) >= anchor
        mask &= role_names != "resin_z_compensation"
        arrays["z"][mask] += np.asarray(new_resin_z - current_resin_z, dtype=arrays["z"].dtype)
    for block in manifest.get("blocks", []):
        block_id = int(block.get("id", -1))
        kind = block.get("kind")
        if kind == "tool_change" and ("tool_offset" in overrides or "tool_change_safe_lift_mm" in overrides):
            _apply_tool_block_rebuild(
                arrays,
                block_id,
                int(block.get("target_tool_id", 1)),
                current_tool_offset,
                new_tool_offset,
                float(overrides.get("tool_change_safe_lift_mm", base.get("tool_change_safe_lift_mm", 20.0))),
                role_vocab,
            )
        if kind == "cut" and ("cut_lift_mm" in overrides or "cut_wait_s" in overrides):
            _apply_cut_block_rebuild(
                arrays,
                block_id,
                float(overrides.get("cut_lift_mm", base.get("cut_lift_mm", 20.0))),
                float(overrides.get("cut_wait_s", base.get("cut_wait_s", 15.0))),
                float(base.get("cut_lift_mm", 20.0)),
                float(base.get("cut_wait_s", 15.0)),
                role_vocab,
            )
    if inserted_rows:
        _repair_indices(arrays)
    static["core_injection_manifest"] = np.array(_update_manifest(manifest, overrides))
    _finite_and_lengths(arrays)
    if not np.all(arrays["trigger_seq"][arrays["event_flag"] != 0] == arrays["seq"][arrays["event_flag"] != 0]):
        raise ValueError("event trigger_seq no longer matches event seq")
    output = Path(output_path).expanduser() if output_path is not None else Path(input_path).expanduser()
    parts = _output_parts(output, len(files))
    chunk_sizes = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            chunk_sizes.append(len(data["seq"]))
    if len(chunk_sizes) > 1 and sum(chunk_sizes) != len(arrays["seq"]):
        prefix_sizes = chunk_sizes[:-1]
        remaining = len(arrays["seq"]) - sum(prefix_sizes)
        if remaining > 0:
            chunk_sizes = prefix_sizes + [remaining]
        else:
            chunk_sizes = []
            remaining = len(arrays["seq"])
            while remaining > 0:
                size = min(100000, remaining)
                chunk_sizes.append(size)
                remaining -= size
            parts = _output_parts(output, len(chunk_sizes))
    _atomic_write(parts, arrays, static, chunk_sizes)
    return {
        "input_parts": len(files),
        "output_parts": len(parts),
        "rows": len(arrays["seq"]),
        "output_path": str(output),
        "delta_tool_offset": [float(v) for v in delta_tool],
        "delta_resin_z_mm": float(new_resin_z - current_resin_z),
    }
