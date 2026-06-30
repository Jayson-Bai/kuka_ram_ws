"""Persistent print parameter JSON config."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

from gcode_planner.head_calibration import DEFAULT_DATA_ROOT

from .process_params import FiberProcessParams, ProcessParams, ResinProcessParams


_CONFIG_DIR_NAME = "external_npz_preprocessor"
_CONFIG_FILE_NAME = "print_params.json"
_CONFIG_VERSION = 1


def default_print_params_path(data_root: str | Path | None = None) -> Path:
    root = Path(data_root) if data_root is not None else DEFAULT_DATA_ROOT
    return root / _CONFIG_DIR_NAME / _CONFIG_FILE_NAME


def save_print_params(params: ProcessParams, path: str | Path | None = None) -> Path:
    target = Path(path).expanduser() if path is not None else default_print_params_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": "external_npz_preprocessor_print_params",
        "version": _CONFIG_VERSION,
        "params": asdict(params),
    }
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return target


def load_print_params(path: str | Path | None = None) -> ProcessParams:
    source = Path(path).expanduser() if path is not None else default_print_params_path()
    if not source.is_file():
        return ProcessParams()
    raw = json.loads(source.read_text(encoding="utf-8"))
    params = raw.get("params", raw)
    if not isinstance(params, dict):
        raise ValueError("print parameter JSON must contain an object")
    return process_params_from_dict(params)


def process_params_from_dict(data: dict[str, Any]) -> ProcessParams:
    defaults = asdict(ProcessParams())
    merged = _deep_merge(defaults, data)
    return ProcessParams(
        resin=ResinProcessParams(**_known_fields(ResinProcessParams, merged.get("resin", {}))),
        fiber=FiberProcessParams(**_known_fields(FiberProcessParams, merged.get("fiber", {}))),
        travel_feed_mm_s=float(merged.get("travel_feed_mm_s", defaults["travel_feed_mm_s"])),
        default_a=float(merged.get("default_a", defaults["default_a"])),
        default_b=float(merged.get("default_b", defaults["default_b"])),
        default_c=float(merged.get("default_c", defaults["default_c"])),
        dt=float(merged.get("dt", defaults["dt"])),
        corner_angle_deg=float(merged.get("corner_angle_deg", defaults["corner_angle_deg"])),
        corner_retreat_ratio=float(merged.get("corner_retreat_ratio", defaults["corner_retreat_ratio"])),
        density=int(merged.get("density", defaults["density"])),
        degree=int(merged.get("degree", defaults["degree"])),
        max_fit_points_per_segment=int(
            merged.get("max_fit_points_per_segment", defaults["max_fit_points_per_segment"])
        ),
    )


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def _known_fields(cls, data: dict[str, Any]) -> dict[str, Any]:
    names = set(cls.__dataclass_fields__.keys())
    return {key: value for key, value in data.items() if key in names}
