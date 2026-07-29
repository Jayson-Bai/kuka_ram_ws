"""Final-NPZ RSI continuity and TCP-floor validation.

Events are semantic rows, not RSI Cartesian samples.  Validation therefore
removes event rows and checks the remaining RSI samples as one global sequence;
an event can never hide a Cartesian discontinuity.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
from typing import Iterable

import numpy as np


_PART_RE = re.compile(r"^(?P<base>.+)_part(?P<part>\d{4})$")
_RSI_PERIOD_S = 0.004
_MAX_TCP_SPEED_MM_S = 25.0
_FLOAT_TOLERANCE_MM = 2e-5
_MAX_TCP_STEP_MM = _MAX_TCP_SPEED_MM_S * _RSI_PERIOD_S + _FLOAT_TOLERANCE_MM


class RsiContinuityError(ValueError):
    """Raised when final Cartesian RSI samples contain a discontinuity."""


def _normalise_paths(paths: str | Path | Iterable[str | Path]) -> list[Path]:
    requested = [paths] if isinstance(paths, (str, Path)) else list(paths)
    expanded: list[Path] = []
    for item in requested:
        path = Path(item)
        if path.is_dir():
            expanded.extend(path.glob("*.npz"))
            continue
        if path.exists():
            match = _PART_RE.match(path.stem)
            if match:
                expanded.extend(path.parent.glob(f"{match.group('base')}_part*.npz"))
            else:
                expanded.append(path)
            continue
        # A multi-part Core export is addressed by its base path without the
        # part suffix.  Keep this branch independent of event/path metadata.
        expanded.extend(path.parent.glob(f"{path.stem}_part*.npz"))
    unique = {item.resolve() for item in expanded if item.suffix.lower() == ".npz"}
    if not unique:
        raise FileNotFoundError("no final NPZ file found for RSI validation")

    def first_seq(path: Path) -> int:
        with np.load(path, allow_pickle=False) as data:
            return int(data["seq"][0]) if len(data["seq"]) else -1

    return sorted(unique, key=lambda item: (first_seq(item), str(item)))


def _decode_vocab(keys: np.ndarray, values: np.ndarray) -> dict[int, str]:
    return {
        int(value): key.decode("utf-8").rstrip("\x00")
        for key, value in zip(keys, values)
    }


def _write_report(path: Path | None, report: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


def validate_final_npz(
    paths: str | Path | Iterable[str | Path], *, report_path: str | Path | None = None,
) -> dict:
    """Validate final Cartesian RSI continuity and report TCP floor warnings.

    Validation is stateless: it accepts only the fixed 4 ms timestamp sequence
    and a fixed 25 mm/s TCP speed limit.  It uses no source-file baseline,
    historical result, path-local threshold, or adaptive quantile.  Event rows
    are removed only for geometry; the remaining global RSI sequence stays
    joined across every event.
    """
    files = _normalise_paths(paths)
    chunks = []
    static = None
    for path in files:
        with np.load(path, allow_pickle=False) as data:
            required = {"seq", "x", "y", "z", "event_flag", "planned_time_s", "tool_id", "move_type", "layer_index", "move_type_vocab_keys", "move_type_vocab_vals"}
            missing = sorted(required - set(data.files))
            if missing:
                raise ValueError(f"{path} missing NPZ fields: {', '.join(missing)}")
            current_static = {
                key: data[key].copy()
                for key in ("move_type_vocab_keys", "move_type_vocab_vals")
            }
            if static is None:
                static = current_static
            elif any(not np.array_equal(static[key], current_static[key]) for key in static):
                raise ValueError("NPZ parts have incompatible move-type vocabularies")
            # Validate the exact consumer payload sent to RSI.  x64/y64/z64
            # are injector-only mirrors and are never the execution contract.
            pose_keys = ("x", "y", "z")
            chunks.append({
                "seq": data["seq"].astype(np.int64, copy=True),
                "xyz": np.column_stack([data[key].astype(np.float64) for key in pose_keys]),
                "event": data["event_flag"].astype(np.uint8, copy=True),
                "time": data["planned_time_s"].copy(),
                "tool": data["tool_id"].astype(np.int64, copy=True),
                "move": data["move_type"].astype(np.int64, copy=True),
                "layer": data["layer_index"].astype(np.int64, copy=True),
            })

    values = {key: np.concatenate([chunk[key] for chunk in chunks]) for key in chunks[0]}
    seq = values["seq"]
    if len(seq) < 2 or not np.all(np.diff(seq) == 1):
        raise RsiContinuityError("final NPZ seq must be globally contiguous")
    rsi = values["event"] == 0
    rsi_seq = seq[rsi]
    rsi_xyz = values["xyz"][rsi]
    rsi_time = values["time"][rsi]
    if len(rsi_xyz) < 2:
        raise RsiContinuityError("final NPZ has fewer than two RSI Cartesian samples")
    if not np.all(np.isfinite(rsi_xyz)):
        raise RsiContinuityError("final NPZ RSI Cartesian samples contain non-finite values")
    if not np.all(np.diff(rsi_seq) > 0) or not np.all(np.diff(rsi_time) > 0.0):
        raise RsiContinuityError("final NPZ RSI sequence/time is not strictly increasing")

    expected_time = (np.arange(len(rsi_time), dtype=np.float64) * _RSI_PERIOD_S).astype(rsi_time.dtype).astype(np.float64)
    time_error = np.abs(rsi_time.astype(np.float64) - expected_time)
    timing_bad = np.flatnonzero(time_error > 0.0)
    if len(timing_bad):
        index = int(timing_bad[np.argmax(time_error[timing_bad])])
        raise RsiContinuityError(
            "final NPZ RSI timestamps do not equal the fixed 4 ms sequence: "
            f"sample {index}, error={time_error[index]:.9f} s"
        )
    deltas = np.diff(rsi_xyz, axis=0)
    steps = np.linalg.norm(deltas, axis=1)
    step_limit = _MAX_TCP_STEP_MM
    bad = np.flatnonzero(steps > step_limit)

    report = {
        "format": "rsi_continuity_validation",
        "version": 1,
        "files": [str(item) for item in files],
        "rows": int(len(seq)),
        "rsi_rows": int(np.count_nonzero(rsi)),
        "event_rows_excluded_from_geometry": int(np.count_nonzero(~rsi)),
        "rsi_period_s": _RSI_PERIOD_S,
        "max_timestamp_quantization_error_s": float(np.max(time_error)) if len(time_error) else 0.0,
        "max_tcp_speed_mm_s": _MAX_TCP_SPEED_MM_S,
        "rsi_step_limit_mm": float(step_limit),
        "max_rsi_step_mm": float(np.max(steps)),
        "step_continuity_ok": not len(bad),
        "continuity_ok": not len(bad),
        "tcp_floor_ok": True,
        "warnings": [],
    }
    if len(bad):
        index = int(bad[np.argmax(steps[bad])])
        report["continuity_error"] = {
            "from_seq": int(rsi_seq[index]),
            "to_seq": int(rsi_seq[index + 1]),
            "seq_gap": int(rsi_seq[index + 1] - rsi_seq[index]),
            "step_mm": float(steps[index]),
            "limit_mm": float(step_limit),
            "from_xyz": rsi_xyz[index].tolist(),
            "to_xyz": rsi_xyz[index + 1].tolist(),
        }

    move_vocab = _decode_vocab(static["move_type_vocab_keys"], static["move_type_vocab_vals"])
    print_codes = [code for code, name in move_vocab.items() if name in ("PRINT", "PRINT_FIT")]
    resin_print = rsi & (values["tool"] == 2) & np.isin(values["move"], print_codes)
    if not np.any(resin_print):
        report["tcp_floor_ok"] = None
        report["warnings"].append("未找到树脂首层 PRINT/PRINT_FIT RSI 点，无法建立 TCP Z 安全阈值")
    else:
        first_layer = int(np.min(values["layer"][resin_print]))
        first_mask = resin_print & (values["layer"] == first_layer)
        floor_z = float(np.min(values["xyz"][first_mask, 2]))
        first_seq = int(np.min(seq[first_mask]))
        later = rsi & (seq > first_seq)
        later_z = values["xyz"][later, 2]
        report["tcp_floor"] = {
            "first_resin_layer": first_layer,
            "first_resin_z_min_mm": floor_z,
            "subsequent_rsi_z_min_mm": float(np.min(later_z)) if len(later_z) else floor_z,
        }
        below = np.flatnonzero(later & (values["xyz"][:, 2] < floor_z - _FLOAT_TOLERANCE_MM))
        if len(below):
            index = int(below[np.argmin(values["xyz"][below, 2])])
            report["warnings"].append(
                "后续 RSI TCP Z 低于首层树脂最低点："
                f"seq={int(seq[index])}, z={float(values['xyz'][index, 2]):.6f} mm, "
                f"安全阈值={floor_z:.6f} mm"
            )
            report["tcp_floor_ok"] = False

    report["ok"] = bool(report["continuity_ok"]) and report["tcp_floor_ok"] is not False
    _write_report(Path(report_path) if report_path is not None else None, report)
    if bad.size:
        error = report["continuity_error"]
        raise RsiContinuityError(
            "final NPZ RSI discontinuity: "
            f"seq {error['from_seq']} -> {error['to_seq']}, "
            f"{error['step_mm']:.6f} mm > {error['limit_mm']:.6f} mm"
        )
    if report["tcp_floor_ok"] is False:
        raise RsiContinuityError("final NPZ TCP Z falls below the first-resin-layer safety floor")
    return report


def _main() -> int:
    parser = argparse.ArgumentParser(description="Validate final NPZ RSI continuity and TCP Z floor")
    parser.add_argument("npz", nargs="+", help="final NPZ file, part base, or output directory")
    parser.add_argument("--report", help="optional JSON report path")
    args = parser.parse_args()
    try:
        report = validate_final_npz(args.npz, report_path=args.report)
    except (OSError, ValueError) as exc:
        print(f"RSI validation failed: {exc}")
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
