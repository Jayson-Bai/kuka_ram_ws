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
_MAX_ESTIMATED_ROWS = 4_000_000
_HIGH_PRECISION_MAP = {
    "x": "x64",
    "y": "y64",
    "z": "z64",
    "a": "a64",
    "b": "b64",
    "c": "c64",
    "e": "e64",
}
_HIGH_PRECISION_FIELDS = tuple(_HIGH_PRECISION_MAP.values())
_HIGH_PRECISION_POSE_FIELDS = _HIGH_PRECISION_FIELDS[:6]

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


def _read_parts(files: list[Path]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], list[str]]:
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
    present_precision = [key for key in _HIGH_PRECISION_FIELDS if key in arrays]
    if present_precision and len(present_precision) != len(_HIGH_PRECISION_FIELDS):
        raise ValueError("NPZ high-precision fields must be provided as a complete x64..e64 set")
    for key in present_precision:
        if arrays[key].dtype != np.float64:
            raise ValueError(f"{key} must use float64")
    return arrays, static, list(chunks[0].keys())


def _has_high_precision(arrays: dict[str, np.ndarray]) -> bool:
    return all(key in arrays for key in _HIGH_PRECISION_FIELDS)


def _pose_from_arrays(arrays: dict[str, np.ndarray], index: int) -> np.ndarray:
    keys = _HIGH_PRECISION_POSE_FIELDS if _has_high_precision(arrays) else (
        "x", "y", "z", "a", "b", "c"
    )
    return np.asarray([arrays[key][index] for key in keys], dtype=np.float64)


def _sync_public_pose(arrays: dict[str, np.ndarray]) -> None:
    if not _has_high_precision(arrays):
        return
    for public, precise in _HIGH_PRECISION_MAP.items():
        arrays[public] = arrays[precise].astype(arrays[public].dtype, copy=False)


def _set_array_pose(arrays: dict[str, np.ndarray], index: int, pose: np.ndarray, e: float | None = None) -> None:
    values = tuple(float(value) for value in pose[:6])
    for public, value in zip(("x", "y", "z", "a", "b", "c"), values):
        arrays[public][index] = value
        precise = _HIGH_PRECISION_MAP.get(public)
        if precise is not None and precise in arrays:
            arrays[precise][index] = value
    if e is not None:
        arrays["e"][index] = float(e)
        if "e64" in arrays:
            arrays["e64"][index] = float(e)


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

def _ensure_role_vocab(static: dict[str, np.ndarray], name: str, code: int) -> None:
    roles = _decode_vocab(static["core_injection_role_vocab_keys"], static["core_injection_role_vocab_vals"])
    if name in roles.values():
        return
    if code in roles:
        raise ValueError(f"cannot add role {name!r}: code {code} is already in use")
    keys = static["core_injection_role_vocab_keys"]
    values = static["core_injection_role_vocab_vals"]
    static["core_injection_role_vocab_keys"] = np.append(keys, np.asarray([name.encode("utf-8")], dtype=keys.dtype))
    static["core_injection_role_vocab_vals"] = np.append(values, np.asarray([code], dtype=values.dtype))


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
        _set_pose(
            row,
            np.asarray([
                sample.pos.x, sample.pos.y, sample.pos.z,
                sample.pos.a, sample.pos.b, sample.pos.c,
            ], dtype=np.float64),
            e=sample.e,
        )
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
        ratio = index / count
        row_e = float(e_start + delta_e * ratio)
        _set_pose(row, np.asarray(pose, dtype=np.float64), e=row_e)
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
    values = tuple(float(value) for value in pose[:6])
    for public, value in zip(("x", "y", "z", "a", "b", "c"), values):
        row[public] = value
        precise = _HIGH_PRECISION_MAP.get(public)
        if precise is not None and precise in row:
            row[precise] = np.float64(value)
    if e is not None:
        row["e"] = float(e)
        if "e64" in row:
            row["e64"] = np.float64(e)


def _finite_and_lengths(arrays: dict[str, np.ndarray]) -> None:
    n = len(arrays["seq"])
    for key, value in arrays.items():
        if value.ndim == 1 and len(value) != n:
            raise ValueError(f"NPZ field length mismatch: {key}")
    finite_keys = ("x", "y", "z", "a", "b", "c", "e", "planned_time_s")
    if _has_high_precision(arrays):
        finite_keys += _HIGH_PRECISION_FIELDS
    for key in finite_keys:
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
    comp_code = _code(roles, "resin_z_compensation")
    print_codes = {_code(move_types, name) for name in ("PRINT", "PRINT_FIT") if name in move_types.values()}
    effective = np.flatnonzero(
        (arrays["event_flag"] == 0)
        & np.isin(arrays["move_type"], list(print_codes))
    )
    if not len(effective):
        raise ValueError("cannot locate first effective print path for resin compensation")

    comp = np.flatnonzero(mask & (arrays["core_injection_role"] == comp_code))
    previous_compensation_path_id = int(arrays["path_id"][int(comp[0])]) if len(comp) else None
    if len(indices):
        block_effective = np.flatnonzero(
            mask
            & (arrays["event_flag"] == 0)
            & np.isin(arrays["move_type"], list(print_codes))
        )
        # Normal Core output: use the explicit block rows and replace only
        # the existing compensation travel.
        anchor_index = int(block_effective[0]) if len(block_effective) else int(effective[0])
        # A zero-valued Core resin block may contain only a post_anchor at
        # the first row. Its preceding pose is the effective print anchor.
        # With a zero-valued export Core records the semantic insertion point
        # as the first marker row (usually the next tool event), not as the
        # later first PRINT row.  Its predecessor is therefore the exact
        # `last_pose` Core used for direct full export.
        start_index = int(comp[0]) - 1 if len(comp) else int(indices[0]) - 1
        insert_at = int(comp[0]) if len(comp) else int(indices[0])
        remove_end = int(comp[-1]) + 1 if len(comp) else insert_at
    else:
        # Core deliberately emits no marker rows when the exported resin Z
        # compensation is zero. The manifest still defines the semantic
        # anchor as before_first_effective_print_path, so fall back to the
        # first effective print row and insert before it.
        anchor_index = int(effective[0])
        start_index = anchor_index - 1
        insert_at = anchor_index
        remove_end = anchor_index

    if start_index < 0:
        raise ValueError("resin compensation anchor has no preceding pose")
    start_pose = _pose_from_arrays(arrays, start_index)
    end_pose = start_pose.copy()
    end_pose[2] += float(new_value)
    template = int(start_index)
    rows = []
    if abs(float(new_value)) > 1e-9:
        samples = _sample_segment(start_pose, end_pose, 0.0, 0.0, raw="resin_z_print_compensation", feed_mm_s=10.0, dt=dt)
        rows = _motion_rows(arrays, template, samples, bid, comp_code, _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index]))
        # Core allocates this TRAVEL segment at the insertion point, rather
        # than after every later segment already present in the NPZ.
        compensation_path_id = int(np.max(arrays["path_id"][:insert_at])) + 1
        for row in rows:
            row["path_id"] = compensation_path_id
        rows[-1]["path_end_flag"] = 1
    if not rows and previous_compensation_path_id is not None:
        arrays["path_id"][arrays["path_id"] > previous_compensation_path_id] -= 1
    _replace(arrays, insert_at, remove_end, rows)


def _apply_global_transforms(arrays, manifest, roles, move_types, delta_tool, delta_resin):
    role_names = np.array([roles[int(v)] for v in arrays["core_injection_role"]], dtype=object)
    non_event = arrays["event_flag"] == 0
    tool_mask = (arrays["tool_id"] == 1) & non_event & (role_names != "tool_change_pre")
    for public, delta in zip(("x", "y", "z"), delta_tool):
        precise = _HIGH_PRECISION_MAP.get(public) if _has_high_precision(arrays) else public
        arrays[precise][tool_mask] += float(delta)
    if abs(delta_resin) <= 1e-12:
        _sync_public_pose(arrays)
        return
    resin = next(item for item in manifest.get("blocks", []) if item.get("kind") == "resin_z_compensation")
    bid = int(resin["id"])
    print_codes = {_code(move_types, name) for name in ("PRINT", "PRINT_FIT") if name in move_types.values()}
    candidates = np.flatnonzero(
        (arrays["core_injection_block_id"] == bid)
        & non_event & np.isin(arrays["move_type"], list(print_codes))
    )
    if not len(candidates):
        # Zero exported compensation has no row marker. Use the manifest's
        # semantic anchor instead of requiring a synthetic marker row.
        candidates = np.flatnonzero(
            non_event & np.isin(arrays["move_type"], list(print_codes))
        )
    if not len(candidates):
        raise ValueError("cannot locate first effective print path for resin compensation")
    anchor = int(candidates[0])
    index = np.arange(len(arrays["z"]))
    # Resin Z compensation is a world/print-coordinate correction. It must
    # also cover tool-change-pre rows: leaving those rows in the uncorrected
    # frame creates a full compensation-sized jump at a fiber->resin switch.
    # The only rows excluded here are the synthetic resin compensation travel
    # itself, whose endpoint already contains the requested offset.
    zmask = (index >= anchor) & non_event & (role_names != "resin_z_compensation")
    z_key = "z64" if _has_high_precision(arrays) else "z"
    arrays[z_key][zmask] += float(delta_resin)
    _sync_public_pose(arrays)


def _rebuild_tool(arrays, manifest, roles, move_types, block, new_offset, safe_lift, dt, feed):
    """Rebuild the Core-generated pre-tool rows and post-event RSI bridge."""
    bid = int(block["id"])
    pre_code = _code(roles, "tool_change_pre")
    event_code = _code(roles, "tool_change_event")
    mask = arrays["core_injection_block_id"] == bid
    indices = np.flatnonzero(mask)
    events = np.flatnonzero(mask & (arrays["core_injection_role"] == event_code))
    if not len(indices) or not len(events):
        raise ValueError(f"tool block {bid} is incomplete")
    event_index = int(events[0])
    pre = np.flatnonzero(mask & (arrays["core_injection_role"] == pre_code))
    start_index = int(indices[0]) - 1
    if start_index < 0:
        raise ValueError(f"tool block {bid} has no preceding pose")
    start = _pose_from_arrays(arrays, start_index)
    start_e = float(arrays["e64"][start_index] if "e64" in arrays else arrays["e"][start_index])
    # Core manifests record tool changes as ``from_tool`` / ``to_tool``.
    # ``target_tool_id`` is accepted only for compatibility with older
    # experimental manifests. Never guess here: defaulting every block to
    # tool 1 makes a fiber->resin change use the fiber sign and applies the
    # head offset twice.
    target_tool_raw = block.get("to_tool", block.get("target_tool_id"))
    if target_tool_raw is None:
        raise ValueError(f"tool block {bid} has no target tool")
    target_tool = int(target_tool_raw)
    sign = 1.0 if target_tool == 1 else -1.0
    has_offset = bool(np.any(np.abs(new_offset) > 1e-9))
    lifted = start.copy()
    lifted[2] += max(0.0, float(safe_lift))
    target = lifted.copy()
    target[:3] += sign * np.asarray(new_offset, dtype=float)
    rows = []
    if has_offset or safe_lift > 1e-9:
        template = event_index
        if safe_lift > 1e-9:
            samples = _sample_segment(
                start, lifted, start_e, 0.0,
                raw="tool_change_safe_lift", feed_mm_s=feed, dt=dt,
            )
            rows.extend(_motion_rows(
                arrays, template, samples, bid, pre_code,
                _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index]),
            ))
        if has_offset:
            samples = _sample_segment(
                lifted, target, start_e, 0.0,
                raw="fallback_linear", feed_mm_s=feed, dt=dt,
            )
            rows.extend(_motion_rows(
                arrays, template, samples, bid, pre_code,
                _code(move_types, "TRAVEL"), int(arrays["tool_id"][start_index]),
            ))
    if rows:
        rows[-1]["path_end_flag"] = 1
    if len(pre):
        _replace(arrays, int(pre[0]), int(pre[-1]) + 1, rows)
    else:
        _replace(arrays, event_index, event_index, rows)

    # Rebuild the event-to-RSI bridge as a separate synthetic region.  Older
    # Core files have no bridge marker; insert it before their first ordinary
    # post-tool motion so they become safe on the first local injection too.
    mask = arrays["core_injection_block_id"] == bid
    event_index = int(np.flatnonzero(mask & (arrays["core_injection_role"] == event_code))[0])
    bridge_code = _code(roles, "tool_change_bridge")
    bridge = np.flatnonzero(mask & (arrays["core_injection_role"] == bridge_code))
    scan_from = int(bridge[-1]) + 1 if len(bridge) else event_index + 1
    while scan_from < len(arrays["seq"]) and arrays["event_flag"][scan_from] != 0:
        scan_from += 1
    travel_code = _code(move_types, "TRAVEL")
    bridge_rows = []
    if scan_from < len(arrays["seq"]):
        end = _pose_from_arrays(arrays, scan_from)
        e_start = float(arrays["e64"][event_index] if "e64" in arrays else arrays["e"][event_index])
        samples = _sample_segment(target, end, e_start, 0.0, raw="tool_change_post_bridge", feed_mm_s=feed, dt=dt)
        bridge_rows = _motion_rows(
            arrays, scan_from, samples, bid, bridge_code, travel_code,
            tool_id=int(arrays["tool_id"][event_index]),
        )
        if bridge_rows:
            bridge_rows[-1]["path_end_flag"] = 0
    if len(bridge):
        _replace(arrays, int(bridge[0]), int(bridge[-1]) + 1, bridge_rows)
    elif bridge_rows:
        insert_at = event_index + 1
        while insert_at < len(arrays["seq"]) and arrays["event_flag"][insert_at] != 0:
            insert_at += 1
        _replace(arrays, insert_at, insert_at, bridge_rows)

    mask = arrays["core_injection_block_id"] == bid
    event_index = int(np.flatnonzero(mask & (arrays["core_injection_role"] == event_code))[0])
    _set_pose(_row(arrays, event_index), target)
    _set_array_pose(arrays, event_index, target)


def _rebuild_cut(arrays, static, manifest, roles, move_types, block, new_lift, new_wait, dt, feed):
    """Rebuild the CUT handshake with the canonical Core row contract."""
    bid = int(block["id"])
    mask = arrays["core_injection_block_id"] == bid
    indices = np.flatnonzero(mask)
    cut_code = _code(roles, "cut_event")
    action_code = _code(roles, "cut_action")
    post_code = _code(roles, "cut_post")
    cut_events = np.flatnonzero(mask & (arrays["core_injection_role"] == cut_code))
    if not len(indices) or not len(cut_events):
        raise ValueError(f"CUT block {bid} is incomplete")
    event_index = int(cut_events[0])
    event_vocab = _decode_vocab(static["event_type_vocab_keys"], static["event_type_vocab_vals"])
    reset_code = _code(event_vocab, "extrude_reset")
    resets = [
        int(i) for i in np.flatnonzero(
            mask & (arrays["event_flag"] != 0)
            & (arrays["event_type"] == reset_code)
        ) if int(i) > event_index
    ]
    if len(resets) < 2:
        raise ValueError(f"CUT block {bid} does not contain the full post-CUT reset sequence")

    old_indices = [int(i) for i in indices]
    cut_line = str(int(block.get("source_line", -1))).encode("ascii")
    anchor_code = _code(roles, "post_anchor")
    # The first two resets are emitted by Core's CUT post sequence.  Later
    # resets belong to the following external source command and must be kept.
    last_reset = resets[1]
    region_start = int(indices[0])
    region_end = int(indices[-1])
    tail = [int(i) for i in old_indices if int(i) > last_reset]
    tail.extend(
        int(i) for i in range(region_start, region_end + 1)
        if int(i) > last_reset and int(arrays["core_injection_block_id"][i]) != bid
    )
    tail.sort()

    def _line(index):
        value = arrays["src_line"][index]
        return bytes(value).split(b"\x00", 1)[0]

    source_action_path_end = any(
        int(arrays["path_end_flag"][index]) != 0
        and int(arrays["core_injection_role"][index]) == action_code
        for index in old_indices
    )
    preserved = [
        index for index in tail
        if _line(index) != cut_line
        or int(arrays["core_injection_role"][index]) == anchor_code
    ]
    connector = [
        index for index in preserved
        if int(arrays["core_injection_role"][index]) == post_code
    ]
    connector_set = set(connector)
    connector_ordinal = {index: ordinal for ordinal, index in enumerate(connector)}
    connector_positions = np.asarray([_pose_from_arrays(arrays, index)[:3] for index in connector], dtype=np.float64)
    connector_distances = np.linalg.norm(np.diff(connector_positions, axis=0), axis=1) if len(connector) > 1 else np.empty(0)
    connector_total = float(np.sum(connector_distances))
    connector_progress = np.concatenate(([0.0], np.cumsum(connector_distances) / connector_total)) if connector_total > 1e-12 else np.zeros(len(connector))
    base_lift = float(manifest.get("base_parameters", {}).get("cut_lift_mm", 20.0))
    lift_delta = float(new_lift) - base_lift

    low = _pose_from_arrays(arrays, event_index - 1)
    low_e = float(arrays["e64"][event_index - 1] if "e64" in arrays else arrays["e"][event_index - 1])
    high = low.copy()
    high[2] += float(new_lift)
    template = event_index
    print_code = _code(move_types, "PRINT")
    travel_code = _code(move_types, "TRAVEL")
    rows = [_row(arrays, index) for index in old_indices if index <= event_index]

    if new_lift > 1e-9:
        samples = _sample_segment(
            low, high, low_e, float(new_lift), raw="cut_lift_feed",
            feed_mm_s=feed, dt=dt,
        )
        rows.extend(_motion_rows(
            arrays, template, samples, bid, post_code, travel_code,
            int(arrays["tool_id"][event_index]),
        ))

    settle_s = 3.0
    rows.extend(_wait_rows(
        arrays, template, high, float(new_lift), 0.0, settle_s,
        block_id=bid, role_code=action_code, print_code=print_code, dt=dt,
    ))

    def _reset_row(index, e_value):
        row = _row(arrays, index)
        _set_pose(row, high, e=float(e_value))
        row["core_injection_block_id"] = bid
        row["core_injection_role"] = action_code
        return row

    rows.append(_reset_row(resets[0], float(new_lift)))
    rows.extend(_wait_rows(
        arrays, template, high, 0.0, 0.0, dt,
        block_id=bid, role_code=action_code, print_code=print_code, dt=dt,
    ))
    lift_duration_s = float(new_lift) / max(feed, 1e-9)
    rows.extend(_wait_rows(
        arrays, template, high, 0.0, -float(new_lift), lift_duration_s,
        block_id=bid, role_code=action_code, print_code=print_code, dt=dt,
    ))
    rows.extend(_wait_rows(
        arrays, template, high, -float(new_lift), 0.0, settle_s,
        block_id=bid, role_code=action_code, print_code=print_code, dt=dt,
    ))
    rows.append(_reset_row(resets[1], -float(new_lift)))
    rows.extend(_wait_rows(
        arrays, template, high, 0.0, 0.0, dt,
        block_id=bid, role_code=action_code, print_code=print_code, dt=dt,
    ))
    retract_duration_s = float(new_lift) / max(feed, 1e-9)
    remaining = max(
        0.0,
        float(new_wait)
        - lift_duration_s
        - settle_s
        - retract_duration_s
        - settle_s,
    )
    rows.extend(_wait_rows(
        arrays, template, high, 0.0, 0.0, remaining,
        block_id=bid, role_code=action_code, print_code=print_code,
        dt=dt,
    ))

    if source_action_path_end:
        for row in reversed(rows):
            if int(row["event_flag"]) == 0 and int(row["core_injection_role"]) == action_code:
                row["path_end_flag"] = 1
                break
    generated_connector = []
    if connector:
        connector_end_pose = _pose_from_arrays(arrays, connector[-1])
        connector_samples = _sample_segment(
            high, connector_end_pose, 0.0, 0.0, raw="cut_post_connector",
            feed_mm_s=20.0, dt=dt,
        )
        generated_connector = _motion_rows(
            arrays, connector[0], connector_samples, bid, post_code,
            travel_code, int(arrays["tool_id"][connector[0]]),
        )
        if any(int(arrays["path_end_flag"][index]) != 0 for index in connector):
            generated_connector[-1]["path_end_flag"] = 1
    connector_inserted = False
    for index in preserved:
        if index in connector_set:
            if not connector_inserted:
                rows.extend(generated_connector)
                connector_inserted = True
            continue
        row = _row(arrays, index)
        if int(row["event_flag"]) == 0 and int(row["core_injection_role"]) != anchor_code:
            _set_pose(row, high)
        rows.append(row)

    _replace(arrays, int(indices[0]), int(indices[-1]) + 1, rows)



def _reassign_path_ids(arrays, roles, move_types, base_resin, new_resin):
    """Recreate Core path-id allocation after local block replacement."""
    print_codes = {
        _code(move_types, name)
        for name in ("PRINT", "PRINT_FIT")
        if name in move_types.values()
    }
    none_code = _code(roles, "none") if "none" in roles.values() else None
    normal = np.flatnonzero(
        (arrays["event_flag"] == 0)
        & np.isin(arrays["move_type"], list(print_codes))
        & ((arrays["core_injection_role"] == none_code) if none_code is not None else True)
    )
    if not len(normal):
        normal = np.flatnonzero(
            (arrays["event_flag"] == 0)
            & np.isin(arrays["move_type"], list(print_codes))
            & (arrays["core_injection_role"] != _code(roles, "resin_z_compensation"))
        )
    if not len(normal):
        return
    base_has_resin = abs(float(base_resin)) > 1e-9
    new_has_resin = abs(float(new_resin)) > 1e-9
    if base_has_resin == new_has_resin:
        for index in np.flatnonzero(arrays["path_end_flag"] != 0):
            if np.any((arrays["event_flag"][index + 1:] == 0) & (arrays["path_id"][index + 1:] == arrays["path_id"][index])):
                arrays["path_end_flag"][index] = 0
        return
    comp_code = _code(roles, "resin_z_compensation")
    comp_rows = np.flatnonzero(arrays["core_injection_role"] == comp_code)
    if not new_has_resin:
        for index in np.flatnonzero(arrays["path_end_flag"] != 0):
            if np.any((arrays["event_flag"][index + 1:] == 0) & (arrays["path_id"][index + 1:] == arrays["path_id"][index])):
                arrays["path_end_flag"][index] = 0
        return
    if not len(comp_rows):
        raise ValueError("rebuilt resin compensation rows are missing")
    compensation_path_id = int(arrays["path_id"][int(comp_rows[0])])
    for index in range(len(arrays["path_id"])):
        if int(arrays["core_injection_role"][index]) != comp_code and int(arrays["path_id"][index]) >= compensation_path_id and not (int(arrays["core_injection_role"][index]) in (_code(roles, "tool_change_pre"), _code(roles, "tool_change_event")) and int(arrays["path_id"][index]) == compensation_path_id):
            arrays["path_id"][index] += 1
    for index in np.flatnonzero(arrays["path_end_flag"] != 0):
        if np.any((arrays["event_flag"][index + 1:] == 0) & (arrays["path_id"][index + 1:] == arrays["path_id"][index])):
            arrays["path_end_flag"][index] = 0
    return



def _repair(arrays, dt):
    arrays["seq"] = np.arange(len(arrays["seq"]), dtype=arrays["seq"].dtype)
    event_mask = arrays["event_flag"] != 0
    for index in np.flatnonzero(event_mask):
        if index > 0:
            _set_array_pose(arrays, index, _pose_from_arrays(arrays, index - 1), e=float(
                arrays["e64"][index - 1] if "e64" in arrays else arrays["e"][index - 1]
            ))
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
    _sync_public_pose(arrays)
    _finite_and_lengths(arrays)
    if not np.all(np.diff(arrays["planned_time_s"]) >= -1e-6):
        raise ValueError("planned_time_s is not monotonic")


def _output_parts(output: Path, count: int) -> list[Path]:
    match = _PART_RE.match(output.stem)
    base = output.parent / match.group("base") if match else output.with_suffix("")
    if count == 1:
        return [output if output.suffix == ".npz" else output.with_suffix(".npz")]
    return [Path(f"{base}_part{i:04d}.npz") for i in range(count)]


def _atomic_write(parts, arrays, static, sizes, key_order):
    temporary = []
    offset = 0
    try:
        for target, size in zip(parts, sizes):
            target.parent.mkdir(parents=True, exist_ok=True)
            fd, raw = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
            Path(raw).unlink(missing_ok=True)
            temp = Path(raw)
            row_payload = {key: value[offset:offset + size] for key, value in arrays.items()}
            payload = {}
            for key in key_order:
                if key in row_payload:
                    payload[key] = row_payload[key]
                elif key in static:
                    payload[key] = static[key]
            for key, value in row_payload.items():
                payload.setdefault(key, value)
            for key, value in static.items():
                payload.setdefault(key, value)
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


def _estimate_injected_rows(
    arrays, manifest, *, tool_changed, resin_changed, delta_resin,
    safe, lift, wait, new_offset, feed, dt,
):
    estimated = int(len(arrays["seq"]))
    blocks = manifest.get("blocks", [])
    if resin_changed:
        resin_rows = int(math.ceil(
            max(abs(float(delta_resin)), 0.0) / max(feed * dt, 1e-9)
        ))
        estimated += resin_rows * 4
    if tool_changed:
        tool_blocks = sum(block.get("kind") == "tool_change" for block in blocks)
        travel_mm = max(float(safe), 0.0) + float(
            np.linalg.norm(np.asarray(new_offset, dtype=float))
        )
        tool_rows = int(math.ceil(max(travel_mm / max(feed * dt, 1e-9), 1.0) * 4.0))
        estimated += tool_blocks * tool_rows
    base_parameters = manifest.get("base_parameters", {})
    if (
        lift != float(base_parameters.get("cut_lift_mm", 20.0))
        or wait != float(base_parameters.get("cut_wait_s", 15.0))
    ):
        cut_blocks = sum(block.get("kind") == "cut" for block in blocks)
        phase_s = max(
            float(wait), 2.0 * max(float(lift), 0.0) / max(feed, 1e-9) + 9.0
        )
        cut_rows = int(math.ceil(max(phase_s, dt) / dt))
        estimated += cut_blocks * cut_rows
    return estimated


def inject_npz(input_path: str | Path, output_path: str | Path | None = None, *,
               params: LocalInjectionParams | None = None,
               tool_offset: tuple[float, float, float] | None = None,
               resin_z_print_compensation_mm: float | None = None,
               tool_change_safe_lift_mm: float | None = None,
               cut_lift_mm: float | None = None,
               cut_wait_s: float | None = None) -> dict[str, Any]:
    files = _part_files(Path(input_path).expanduser())
    arrays, static, key_order = _read_parts(files)
    _sync_public_pose(arrays)
    _finite_and_lengths(arrays)
    manifest = _json_manifest(static["core_injection_manifest"])
    _ensure_role_vocab(static, "tool_change_bridge", 9)
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
    if not math.isclose(dt, 0.004, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("local injection requires a fixed 4 ms RSI period (sample_period_s=0.004)")

    # A Core NPZ already contains the canonical synthetic CUT/tool-change
    # sequence. Rebuild a block only when its effective parameter changes;
    # passing the same UI value must preserve the Core rows byte-for-byte in
    # their ordering, timing, reset count, and E phases.
    base_safe = float(base.get("tool_change_safe_lift_mm", 20.0))
    base_lift = float(base.get("cut_lift_mm", 20.0))
    base_wait = float(base.get("cut_wait_s", 15.0))
    tool_offset_changed = not np.allclose(new_offset, current_offset, rtol=0.0, atol=1e-9)
    resin_changed = abs(new_resin - current_resin) > 1e-9
    safe_changed = (
        "tool_change_safe_lift_mm" in overrides
        and abs(safe - base_safe) > 1e-9
    )
    cut_changed = (
        ("cut_lift_mm" in overrides and abs(lift - base_lift) > 1e-9)
        or ("cut_wait_s" in overrides and abs(wait - base_wait) > 1e-9)
    )
    estimated_rows = _estimate_injected_rows(
        arrays, manifest,
        tool_changed=tool_offset_changed or safe_changed,
        resin_changed=resin_changed, delta_resin=new_resin - current_resin,
        safe=safe, lift=lift, wait=wait,
        new_offset=new_offset, feed=feed, dt=dt,
    )
    if estimated_rows > _MAX_ESTIMATED_ROWS:
        raise ValueError(
            "local injection exceeds the safe in-memory row budget: "
            f"estimated {estimated_rows:,} rows > {_MAX_ESTIMATED_ROWS:,}; "
            "use Core full export for this parameter magnitude"
        )
    if resin_changed:
        _rebuild_resin(arrays, manifest, roles, move_types, new_resin, current_resin, dt, feed)
    _apply_global_transforms(arrays, manifest, roles, move_types, new_offset - current_offset, new_resin - current_resin)
    # A CUT can alter the predecessor pose of a later tool change.
    # Rebuild CUT first, then rebuild tool-change pre/bridge rows from that
    # final pose, matching direct Core full-export ordering.
    for block in manifest.get("blocks", []):
        if block.get("kind") == "cut" and cut_changed:
            _rebuild_cut(arrays, static, manifest, roles, move_types, block, lift, wait, dt, feed)
    for block in manifest.get("blocks", []):
        if block.get("kind") == "tool_change" and (tool_offset_changed or safe_changed):
            _rebuild_tool(arrays, manifest, roles, move_types, block, new_offset, safe, dt, feed)
    _reassign_path_ids(arrays, roles, move_types, current_resin, new_resin)
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
    if len(sizes) == 1:
        # A local block may add/remove rows even when the source is one
        # chunk; never truncate the rebuilt tail to the source length.
        sizes = [len(arrays["seq"])]
    elif len(sizes) > 1:
        sizes[-1] = len(arrays["seq"]) - sum(sizes[:-1])
        if sizes[-1] <= 0:
            sizes = []
            remain = len(arrays["seq"])
            while remain:
                size = min(100000, remain)
                sizes.append(size)
                remain -= size
            parts = _output_parts(output, len(sizes))
    _atomic_write(parts, arrays, static, sizes, key_order)
    _write_timing(output, arrays, dt)
    return {
        "input_parts": len(files), "output_parts": len(parts), "rows": len(arrays["seq"]),
        "output_path": str(output), "delta_tool_offset": [float(v) for v in new_offset - current_offset],
        "delta_resin_z_mm": float(new_resin - current_resin),
    }
