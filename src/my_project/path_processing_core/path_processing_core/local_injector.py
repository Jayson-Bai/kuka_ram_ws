"""Minimal local Core NPZ injector.

Only marked synthetic blocks are rebuilt. Ordinary print trajectories, E values,
path order and reset/event payloads are preserved. Synthetic segments use the
same Core sampler as the full exporter; malformed or ambiguous blocks fail
closed instead of producing a best-effort trajectory.
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

from .polynomial_interpolator import sample_global_curve_iter
from .types import GlobalCurveCommand, Position

_PART_RE = re.compile(r"^(?P<base>.+)_part(?P<part>\d{4})$")
_REQUIRED = {
    "seq", "x", "y", "z", "a", "b", "c", "e", "tool_id", "move_type",
    "event_flag", "event_type", "payload", "trigger_seq", "layer_index",
    "preview_layer_index", "path_id", "path_end_flag", "planned_time_s",
    "core_injection_manifest", "core_injection_block_id", "core_injection_role",
    "core_injection_role_vocab_keys", "core_injection_role_vocab_vals",
    "move_type_vocab_keys", "move_type_vocab_vals",
    "event_type_vocab_keys", "event_type_vocab_vals",
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
            values = tuple(float(v) for v in self.tool_offset)
            if not all(math.isfinite(v) for v in values):
                raise ValueError("tool_offset must be finite")
            out["tool_offset"] = list(values)
        for name in (
            "resin_z_print_compensation_mm", "tool_change_safe_lift_mm",
            "cut_lift_mm", "cut_wait_s",
        ):
            value = getattr(self, name)
            if value is not None:
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite")
                if name in ("tool_change_safe_lift_mm", "cut_lift_mm", "cut_wait_s") and value < 0:
                    raise ValueError(f"{name} must be >= 0")
                out[name] = value
        return out


def _part_sort_key(path: Path) -> int:
    match = _PART_RE.match(path.stem)
    return int(match.group("part")) if match else -1


def _part_files(path: Path) -> list[Path]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    if path.is_dir():
        files = list(path.glob("*.npz"))
    else:
        match = _PART_RE.match(path.stem)
        files = list(path.parent.glob(f"{match.group('base')}_part*.npz")) if match else [path]
    files.sort(key=lambda p: (_part_sort_key(p), p.name))
    if not files:
        raise ValueError("no NPZ parts found")
    return files


def _read_parts(files: list[Path]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    chunks: list[dict[str, np.ndarray]] = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            chunks.append({key: data[key].copy() for key in data.files})
    first_keys = set(chunks[0])
    missing = sorted(_REQUIRED - first_keys)
    if missing:
        raise ValueError(
            "NPZ is not a core_npz_local_injection_v1 file; missing fields: "
            + ", ".join(missing)
        )
    n = len(chunks[0]["seq"])
    row_keys = [key for key, value in chunks[0].items() if value.ndim == 1 and len(value) == n]
    for part_index, chunk in enumerate(chunks):
        if set(chunk) != first_keys:
            raise ValueError(f"NPZ part {part_index} schema differs from the first part")
        part_n = len(chunk["seq"])
        for key in row_keys:
            if chunk[key].ndim != 1 or len(chunk[key]) != part_n:
                raise ValueError(f"NPZ part {part_index} field length mismatch: {key}")
    static = {key: value for key, value in chunks[0].items() if key not in row_keys}
    for part_index, chunk in enumerate(chunks[1:], start=1):
        for key, value in static.items():
            if not np.array_equal(value, chunk[key]):
                raise ValueError(f"NPZ part {part_index} static field differs: {key}")
    arrays = {key: np.concatenate([chunk[key] for chunk in chunks]) for key in row_keys}
    return arrays, static


def _json_manifest(value: np.ndarray) -> dict[str, Any]:
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
    return {int(value): key.decode("utf-8").rstrip("\x00") for key, value in zip(keys, vals)}


def _code(vocab: dict[int, str], name: str) -> int:
    for value, text in vocab.items():
        if text == name:
            return value
    raise ValueError(f"NPZ vocabulary does not contain {name!r}")


def _row_keys(arrays: dict[str, np.ndarray]) -> list[str]:
    return list(arrays.keys())


def _row(arrays: dict[str, np.ndarray], index: int) -> dict[str, Any]:
    return {key: arrays[key][index].copy() for key in _row_keys(arrays)}


def _rows_array(rows: list[dict[str, Any]], arrays: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    if not rows:
        return {key: np.empty(0, dtype=value.dtype) for key, value in arrays.items()}
    return {key: np.asarray([row[key] for row in rows], dtype=arrays[key].dtype) for key in arrays}


def _replace(arrays: dict[str, np.ndarray], start: int, end: int, rows: list[dict[str, Any]]) -> None:
    replacement = _rows_array(rows, arrays)
    for key in _row_keys(arrays):
        arrays[key] = np.concatenate((arrays[key][:start], replacement[key], arrays[key][end:]))


def _sample_segment(start: np.ndarray, end: np.ndarray, e_start: float, delta_e: float,
                    *, raw: str, feed_mm_s: float, dt: float):
    start_values = [float(v) for v in start]
    end_values = [float(v) for v in end]
    if len(start_values) == 3:
        start_values.extend([0.0, 0.0, 0.0])
        end_values.extend([0.0, 0.0, 0.0])
    start_pos = Position(*start_values)
    end_pos = Position(*end_values)
    curve = GlobalCurveCommand(
        type="TRAVEL", cmd="SPLINE", start_pos=start_pos,
        control_points=[end_pos, end_pos, end_pos],
        e_val=float(e_start + delta_e), delta_e=float(delta_e),
        feedrate=float(feed_mm_s) * 60.0, line=0, raw=raw,
        constraints=[], original_moves=[],
    )
    return list(sample_global_curve_iter(
        curve, dt=float(dt), target_velocity=float(feed_mm_s), t_acc=2.0, t_dec=2.0,
    ))


def _motion_rows(arrays, template, samples, block_id, role_code, move_type_code, tool_id=None):
    rows = []
    for sample in samples:
        row = _row(arrays, template)
        row["x"], row["y"], row["z"] = sample.pos.x, sample.pos.y, sample.pos.z
        row["a"], row["b"], row["c"] = sample.pos.a, sample.pos.b, sample.pos.c
        row["e"] = sample.e
        row["move_type"] = move_type_code
        row["event_flag"] = 0
        row["event_type"] = 0
        row["payload"] = np.asarray(b"", dtype=arrays["payload"].dtype)
        row["trigger_seq"] = -1
        row["path_end_flag"] = 0
        row["planned_time_s"] = 0.0
        row["core_injection_block_id"] = block_id
        row["core_injection_role"] = role_code
        if tool_id is not None:
            row["tool_id"] = tool_id
        rows.append(row)
    return rows


def _wait_rows(arrays, template, pose, e_start, delta_e, duration, *, block_id, role_code,
               print_code, dt, path_end=False):
    count = max(1, int(math.ceil(max(float(duration), dt) / dt)))
    rows = []
    for index in range(1, count + 1):
        row = _row(arrays, template)
        row["x"], row["y"], row["z"] = [float(v) for v in pose[:3]]
        if len(pose) >= 6:
            row["a"], row["b"], row["c"] = [float(v) for v in pose[3:6]]
        row["e"] = float(e_start + delta_e * index / count)
        row["move_type"] = print_code
        row["event_flag"] = 0
        row["event_type"] = 0
        row["payload"] = np.asarray(b"", dtype=arrays["payload"].dtype)
        row["trigger_seq"] = -1
        row["path_end_flag"] = 1 if path_end and index == count else 0
        row["planned_time_s"] = 0.0
        row["core_injection_block_id"] = block_id
        row["core_injection_role"] = role_code
        rows.append(row)
    return rows


def _set_pose(row: dict[str, Any], pose: np.ndarray, e: float | None = None) -> None:
    row["x"], row["y"], row["z"] = [float(v) for v in pose[:3]]
    if len(pose) >= 6:
        row["a"], row["b"], row["c"] = [float(v) for v in pose[3:6]]
    if e is not None:
        row["e"] = float(e)


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


def _rebuild_resin(arrays, manifest, roles, move_types, new_value, base_value, dt, feed):
    block = next((item for item in manifest.get("blocks", []) if item.get("kind") == "resin_z_compensation"), None)
    if block is None:
        raise ValueError("manifest has no resin_z_compensation block")
    bid = int(block["id"])
    mask = arrays["core_injection_block_id"] == bid
    indices = np.flatnonzero(mask)
    if not len(indices):
        raise ValueError("resin compensation block has no anchor rows")
    comp_code = _code(roles, "resin_z_compensation")
    print_codes = {_code(move_types, name) for name in ("PRINT", "PRINT_FIT") if name in move_types.values()}
    comp = np.flatnonzero(mask & (arrays["core_injection_role"] == comp_code))
    effective = np.flatnonzero(mask & (arrays["event_flag"] == 0) & np.isin(arrays["move_type"], list(print_codes)))
    if not len(effective):
        raise ValueError("resin compensation block has no effective print anchor")
    start_index = int(indices[0]) - 1
    if start_index < 0:
        raise ValueError("resin compensation block has no preceding pose")
    insert_at = int(comp[0]) if len(comp) else int(effective[0])
    remove_end = int(comp[-1]) + 1 if len(comp) else insert_at
    start_pose = np.array([arrays[k][start_index] for k in ("x", "y", "z", "a", "b", "c")], dtype=float)
    end_pose = start_pose.copy()
    end_pose[2] += float(new_value)
    template = int(effective[0])
    rows = []
    if abs(float(new_value)) > 1e-9:
        samples = _sample_segment(start_pose, end_pose, 0.0, 0.0, raw="resin_z_print_compensation", feed_mm_s=feed, dt=dt)
        rows = _motion_rows(arrays, template, samples, bid, comp_code, _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index]))
        rows[-1]["path_end_flag"] = 1
    _replace(arrays, insert_at, remove_end, rows)


def _apply_global_transforms(arrays, manifest, roles, move_types, delta_tool, delta_resin):
    role_names = np.array([roles[int(v)] for v in arrays["core_injection_role"]], dtype=object)
    non_event = arrays["event_flag"] == 0
    tool_mask = (arrays["tool_id"] == 1) & non_event & (role_names != "tool_change_pre")
    for key, delta in zip(("x", "y", "z"), delta_tool):
        arrays[key][tool_mask] += np.asarray(delta, dtype=arrays[key].dtype)
    if abs(delta_resin) <= 1e-12:
        return
    resin = next(item for item in manifest.get("blocks", []) if item.get("kind") == "resin_z_compensation")
    bid = int(resin["id"])
    print_codes = {_code(move_types, name) for name in ("PRINT", "PRINT_FIT") if name in move_types.values()}
    candidates = np.flatnonzero(
        (arrays["core_injection_block_id"] == bid)
        & non_event & np.isin(arrays["move_type"], list(print_codes))
    )
    if not len(candidates):
        raise ValueError("cannot locate first effective print path for resin compensation")
    anchor = int(candidates[0])
    index = np.arange(len(arrays["z"]))
    zmask = (index >= anchor) & non_event & (role_names != "resin_z_compensation") & (role_names != "tool_change_pre")
    arrays["z"][zmask] += np.asarray(delta_resin, dtype=arrays["z"].dtype)


def _rebuild_tool(arrays, manifest, roles, move_types, block, new_offset, safe_lift, dt, feed):
    bid = int(block["id"])
    pre_code = _code(roles, "tool_change_pre")
    event_code = _code(roles, "tool_change_event")
    post_code = _code(roles, "tool_change_post")
    mask = arrays["core_injection_block_id"] == bid
    indices = np.flatnonzero(mask)
    event = np.flatnonzero(mask & (arrays["core_injection_role"] == event_code))
    if not len(indices) or not len(event):
        raise ValueError(f"tool block {bid} is incomplete")
    event_index = int(event[0])
    pre = np.flatnonzero(mask & (arrays["core_injection_role"] == pre_code))
    start_index = int(indices[0]) - 1
    if start_index < 0:
        raise ValueError(f"tool block {bid} has no preceding pose")
    start = np.array([arrays[k][start_index] for k in ("x", "y", "z", "a", "b", "c")], dtype=float)
    target_tool = int(block.get("target_tool_id", 1))
    sign = 1.0 if target_tool == 1 else -1.0
    has_offset = bool(np.any(np.abs(new_offset) > 1e-9))
    rows = []
    lifted = start.copy()
    lifted[2] += max(0.0, float(safe_lift))
    target = lifted.copy()
    target[:3] += sign * np.asarray(new_offset, dtype=float)
    if has_offset:
        template = event_index
        if safe_lift > 1e-9:
            samples = _sample_segment(start, lifted, float(arrays["e"][start_index]), 0.0, raw="tool_change_safe_lift", feed_mm_s=feed, dt=dt)
            rows.extend(_motion_rows(arrays, template, samples, bid, pre_code, _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index])))
        samples = _sample_segment(lifted, target, float(arrays["e"][start_index]), 0.0, raw="fallback_linear", feed_mm_s=feed, dt=dt)
        rows.extend(_motion_rows(arrays, template, samples, bid, pre_code, _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index])))
    if len(pre):
        _replace(arrays, int(pre[0]), int(pre[-1]) + 1, rows)
    else:
        _replace(arrays, event_index, event_index, rows)
    mask = arrays["core_injection_block_id"] == bid
    event_index = int(np.flatnonzero(mask & (arrays["core_injection_role"] == event_code))[0])
    _set_pose(_row(arrays, event_index), target)
    for key, value in zip(("x", "y", "z", "a", "b", "c"), target):
        arrays[key][event_index] = value
    anchor_candidates = np.flatnonzero(mask & (arrays["core_injection_role"] == _code(roles, "post_anchor")))
    anchor = int(anchor_candidates[-1]) if len(anchor_candidates) else int(np.flatnonzero(mask)[-1])
    post_indices = [int(i) for i in np.flatnonzero(mask) if event_index < int(i) <= anchor and arrays["event_flag"][i] == 0]
    if post_indices:
        end = np.array([arrays[k][anchor] for k in ("x", "y", "z", "a", "b", "c")], dtype=float)
        connector_samples = _sample_segment(
            target, end, float(arrays["e"][post_indices[0]]), 0.0,
            raw="tool_change_post_connector", feed_mm_s=feed, dt=dt,
        )
        # Keep all original post rows and event rows.  The Core sampler defines
        # the new connector trajectory; its samples are projected onto the
        # existing non-event slots so reset/event payloads and path E remain
        # untouched.
        sample_index = np.linspace(0.0, len(connector_samples) - 1, len(post_indices))
        for rank, index in enumerate(post_indices):
            position = sample_index[rank]
            lo = int(math.floor(position))
            hi = min(lo + 1, len(connector_samples) - 1)
            ratio = position - lo
            p0 = connector_samples[lo].pos
            p1 = connector_samples[hi].pos
            pose = np.array([
                p0.x + ratio * (p1.x - p0.x),
                p0.y + ratio * (p1.y - p0.y),
                p0.z + ratio * (p1.z - p0.z),
                p0.a + ratio * (p1.a - p0.a),
                p0.b + ratio * (p1.b - p0.b),
                p0.c + ratio * (p1.c - p0.c),
            ], dtype=float)
            for key, value in zip(("x", "y", "z", "a", "b", "c"), pose):
                arrays[key][index] = value


def _rebuild_cut(arrays, static, manifest, roles, move_types, block, new_lift, new_wait, dt, feed):
    bid = int(block["id"])
    mask = arrays["core_injection_block_id"] == bid
    indices = np.flatnonzero(mask)
    cut_code = _code(roles, "cut_event")
    action_code = _code(roles, "cut_action")
    cut_event = np.flatnonzero(mask & (arrays["core_injection_role"] == cut_code))
    if not len(indices) or not len(cut_event):
        raise ValueError(f"CUT block {bid} is incomplete")
    event_index = int(cut_event[0])
    event_vocab = _decode_vocab(static["event_type_vocab_keys"], static["event_type_vocab_vals"])
    reset_code = _code(event_vocab, "extrude_reset")
    resets = [int(i) for i in np.flatnonzero(mask & (arrays["event_flag"] != 0) & (arrays["event_type"] == reset_code)) if int(i) > event_index]
    if len(resets) < 3:
        raise ValueError(f"CUT block {bid} does not contain the full reset sequence")
    base_lift = float(manifest.get("base_parameters", {}).get("cut_lift_mm", 20.0))
    old_indices = [int(i) for i in indices]
    pose_index = max(0, event_index - 1)
    low = np.array([arrays[k][pose_index] for k in ("x", "y", "z", "a", "b", "c")], dtype=float)
    low_e = float(arrays["e"][pose_index])
    high = low.copy()
    high[2] += float(new_lift)
    print_code = _code(move_types, "PRINT")
    travel_code = _code(move_types, "TRAVEL")
    rows = [_row(arrays, int(i)) for i in old_indices if int(i) <= event_index]
    template = event_index
    if new_lift > 1e-9:
        samples = _sample_segment(low, high, low_e, float(new_lift), raw="fallback_linear", feed_mm_s=feed, dt=dt)
        rows.extend(_motion_rows(arrays, template, samples, bid, action_code, travel_code, int(arrays["tool_id"][event_index])))
    pose_high = high
    rows.extend(_wait_rows(arrays, template, pose_high, low_e + new_lift, 0.0, 3.0, block_id=bid, role_code=action_code, print_code=print_code, dt=dt))
    rows.append(_row(arrays, resets[0]))
    lift_time = float(new_lift) / max(feed, 1e-9)
    rows.extend(_wait_rows(arrays, template, pose_high, float(new_lift), -2.0 * float(new_lift), lift_time, block_id=bid, role_code=action_code, print_code=print_code, dt=dt))
    rows.extend(_wait_rows(arrays, template, pose_high, -float(new_lift), 0.0, 3.0, block_id=bid, role_code=action_code, print_code=print_code, dt=dt))
    rows.append(_row(arrays, resets[1]))
    remaining = max(0.0, float(new_wait) - 2.0 * lift_time - 6.0)
    rows.extend(_wait_rows(arrays, template, pose_high, 0.0, 0.0, remaining, block_id=bid, role_code=action_code, print_code=print_code, dt=dt))
    rows.append(_row(arrays, resets[2]))
    for extra_reset in resets[3:]:
        rows.append(_row(arrays, extra_reset))
    # Reconnect the post-CUT rows. Their old E/path metadata is retained.
    last_reset = resets[-1]
    post = [int(i) for i in old_indices if int(i) > last_reset]
    anchor_candidates = [i for i in post if int(arrays["core_injection_role"][i]) == _code(roles, "post_anchor")]
    if anchor_candidates:
        anchor_pose = np.array([arrays[k][anchor_candidates[-1]] for k in ("x", "y", "z", "a", "b", "c")], dtype=float)
        connector = post
        if connector:
            connector_samples = _sample_segment(
                pose_high, anchor_pose, 0.0, 0.0, raw="fallback_linear",
                feed_mm_s=feed, dt=dt,
            )
            connector_rows = _motion_rows(
                arrays, connector[0], connector_samples, bid,
                _code(roles, "cut_post"), _code(move_types, "TRAVEL"),
                int(arrays["tool_id"][connector[0]]),
            )
            if any(int(arrays["path_end_flag"][index]) != 0 for index in connector):
                connector_rows[-1]["path_end_flag"] = 1
            if anchor_candidates:
                connector_rows[-1]["core_injection_role"] = _code(roles, "post_anchor")
            rows.extend(connector_rows)
    else:
        for index in post:
            row = _row(arrays, index)
            if arrays["event_flag"][index] == 0:
                _set_pose(row, pose_high)
            rows.append(row)
    if any(int(arrays["path_end_flag"][index]) != 0 for index in old_indices):
        for row in rows:
            row["path_end_flag"] = 0
        for row in reversed(rows):
            if int(row["event_flag"]) == 0:
                row["path_end_flag"] = 1
                break
    _replace(arrays, int(indices[0]), int(indices[-1]) + 1, rows)


def _repair(arrays, dt):
    arrays["seq"] = np.arange(len(arrays["seq"]), dtype=arrays["seq"].dtype)
    event_mask = arrays["event_flag"] != 0
    for index in np.flatnonzero(event_mask):
        if index > 0:
            arrays["x"][index] = arrays["x"][index - 1]
            arrays["y"][index] = arrays["y"][index - 1]
            arrays["z"][index] = arrays["z"][index - 1]
            arrays["a"][index] = arrays["a"][index - 1]
            arrays["b"][index] = arrays["b"][index - 1]
            arrays["c"][index] = arrays["c"][index - 1]
            arrays["e"][index] = arrays["e"][index - 1]
    values = np.zeros(len(arrays["seq"]), dtype=arrays["planned_time_s"].dtype)
    trajectory_count = 0
    for index in range(len(values)):
        if event_mask[index]:
            if index:
                values[index] = values[index - 1]
        else:
            values[index] = float(trajectory_count) * float(dt)
            trajectory_count += 1
    arrays["planned_time_s"] = values
    arrays["trigger_seq"] = np.where(event_mask, arrays["seq"], -1).astype(arrays["trigger_seq"].dtype)
    _finite_and_lengths(arrays)
    if not np.all(np.diff(arrays["planned_time_s"]) >= -1e-6):
        raise ValueError("planned_time_s is not monotonic")
    xyz = np.column_stack((arrays["x"], arrays["y"], arrays["z"]))
    if len(xyz) > 1:
        max_step = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).max())
        if max_step > 0.05 + 1e-9:
            raise ValueError(f"local injection created an unsafe XYZ jump: {max_step:.6f} mm")


def _output_parts(output: Path, count: int) -> list[Path]:
    match = _PART_RE.match(output.stem)
    base = output.parent / match.group("base") if match else output.with_suffix("")
    if count == 1:
        return [output if output.suffix == ".npz" else output.with_suffix(".npz")]
    return [Path(f"{base}_part{i:04d}.npz") for i in range(count)]


def _atomic_write(parts, arrays, static, sizes):
    temporary = []
    offset = 0
    try:
        for target, size in zip(parts, sizes):
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


def _timing_sidecar(path: Path) -> Path:
    stem = _PART_RE.match(path.stem)
    name = stem.group("base") if stem else path.stem
    return path.parent / f"{name}.timing.json"


def _write_timing(path: Path, arrays: dict[str, np.ndarray], dt: float) -> None:
    payload = {
        "format": "rsi_print_timing", "version": 1, "sample_period_s": float(dt),
        "total_planned_time_s": float(arrays["planned_time_s"][-1]) if len(arrays["seq"]) else 0.0,
        "trajectory_rows": int(np.count_nonzero(arrays["event_flag"] == 0)),
        "event_rows_ignored": int(np.count_nonzero(arrays["event_flag"] != 0)),
    }
    target = _timing_sidecar(path)
    fd, raw = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
    Path(raw).unlink(missing_ok=True)
    temp = Path(raw)
    try:
        temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(target)
    finally:
        temp.unlink(missing_ok=True)


def inject_npz(input_path: str | Path, output_path: str | Path | None = None, *,
               params: LocalInjectionParams | None = None,
               tool_offset: tuple[float, float, float] | None = None,
               resin_z_print_compensation_mm: float | None = None,
               tool_change_safe_lift_mm: float | None = None,
               cut_lift_mm: float | None = None,
               cut_wait_s: float | None = None) -> dict[str, Any]:
    files = _part_files(Path(input_path).expanduser())
    arrays, static = _read_parts(files)
    _finite_and_lengths(arrays)
    manifest = _json_manifest(static["core_injection_manifest"])
    overrides = (params or LocalInjectionParams(
        tool_offset=tool_offset,
        resin_z_print_compensation_mm=resin_z_print_compensation_mm,
        tool_change_safe_lift_mm=tool_change_safe_lift_mm,
        cut_lift_mm=cut_lift_mm,
        cut_wait_s=cut_wait_s,
    )).as_overrides()
    if not overrides:
        raise ValueError("at least one local injection parameter is required")
    roles = _decode_vocab(static["core_injection_role_vocab_keys"], static["core_injection_role_vocab_vals"])
    move_types = _decode_vocab(static["move_type_vocab_keys"], static["move_type_vocab_vals"])
    base = manifest.get("base_parameters", {})
    current_offset = np.asarray(base.get("tool_offset", [0.0, 0.0, 0.0]), dtype=float)
    new_offset = np.asarray(overrides.get("tool_offset", current_offset), dtype=float)
    current_resin = float(base.get("resin_z_print_compensation_mm", 0.0))
    new_resin = float(overrides.get("resin_z_print_compensation_mm", current_resin))
    safe = float(overrides.get("tool_change_safe_lift_mm", base.get("tool_change_safe_lift_mm", 20.0)))
    lift = float(overrides.get("cut_lift_mm", base.get("cut_lift_mm", 20.0)))
    wait = float(overrides.get("cut_wait_s", base.get("cut_wait_s", 15.0)))
    dt = float(manifest.get("sample_period_s", 0.004))
    feed = float(base.get("default_feed_mm_s", 10.0))
    if dt <= 0 or feed <= 0:
        raise ValueError("manifest sample_period_s/default_feed_mm_s must be positive")
    if "resin_z_print_compensation_mm" in overrides:
        _rebuild_resin(arrays, manifest, roles, move_types, new_resin, current_resin, dt, feed)
    _apply_global_transforms(arrays, manifest, roles, move_types, new_offset - current_offset, new_resin - current_resin)
    for block in manifest.get("blocks", []):
        if block.get("kind") == "tool_change" and ("tool_offset" in overrides or "tool_change_safe_lift_mm" in overrides):
            _rebuild_tool(arrays, manifest, roles, move_types, block, new_offset, safe, dt, feed)
    for block in manifest.get("blocks", []):
        if block.get("kind") == "cut" and ("cut_lift_mm" in overrides or "cut_wait_s" in overrides):
            _rebuild_cut(arrays, static, manifest, roles, move_types, block, lift, wait, dt, feed)
    _repair(arrays, dt)
    manifest["base_parameters"] = dict(base)
    manifest["base_parameters"].update(overrides)
    static["core_injection_manifest"] = np.array(json.dumps(manifest, ensure_ascii=False, separators=(",", ":")))
    output = Path(output_path).expanduser() if output_path is not None else Path(input_path).expanduser()
    parts = _output_parts(output, len(files))
    sizes = []
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            sizes.append(len(data["seq"]))
    if len(sizes) > 1:
        sizes[-1] = len(arrays["seq"]) - sum(sizes[:-1])
        if sizes[-1] <= 0:
            sizes = []
            remain = len(arrays["seq"])
            while remain:
                size = min(100000, remain)
                sizes.append(size)
                remain -= size
            parts = _output_parts(output, len(sizes))
    _atomic_write(parts, arrays, static, sizes)
    _write_timing(output, arrays, dt)
    return {
        "input_parts": len(files), "output_parts": len(parts), "rows": len(arrays["seq"]),
        "output_path": str(output), "delta_tool_offset": [float(v) for v in new_offset - current_offset],
        "delta_resin_z_mm": float(new_resin - current_resin),
    }
