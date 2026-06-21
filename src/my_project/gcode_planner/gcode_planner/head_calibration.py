from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Mapping


DEFAULT_HEAD_CALIBRATION_PATH = Path(
    "/home/jayson/kuka_ram_ws/data/head_calibration_offsets/head_offsets.json"
)

DEFAULT_HEAD_CALIBRATION = {
    "resin": {"z_print_compensation_mm": 0.0},
    "fiber": {
        "x_print_compensation_mm": 0.0,
        "y_print_compensation_mm": 0.0,
        "z_print_compensation_mm": 0.0,
    },
}


@dataclass(frozen=True)
class HeadCalibration:
    resin_z_print_compensation_mm: float = 0.0
    fiber_x_print_compensation_mm: float = 0.0
    fiber_y_print_compensation_mm: float = 0.0
    fiber_z_print_compensation_mm: float = 0.0


def _as_float(data: Mapping[str, object], key: str, default: float) -> float:
    value = data.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def load_head_calibration(
    path: str | Path = DEFAULT_HEAD_CALIBRATION_PATH,
) -> HeadCalibration:
    p = Path(path)
    if not p.is_file():
        return HeadCalibration()
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return HeadCalibration()
    resin = data.get("resin", {}) if isinstance(data, dict) else {}
    fiber = data.get("fiber", {}) if isinstance(data, dict) else {}
    if not isinstance(resin, dict):
        resin = {}
    if not isinstance(fiber, dict):
        fiber = {}
    return HeadCalibration(
        resin_z_print_compensation_mm=_as_float(
            resin, "z_print_compensation_mm", 0.0
        ),
        fiber_x_print_compensation_mm=_as_float(
            fiber, "x_print_compensation_mm", 0.0
        ),
        fiber_y_print_compensation_mm=_as_float(
            fiber, "y_print_compensation_mm", 0.0
        ),
        fiber_z_print_compensation_mm=_as_float(
            fiber, "z_print_compensation_mm", 0.0
        ),
    )


def save_head_calibration(
    path: str | Path,
    calibration: HeadCalibration,
) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "updated_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "resin": {
            "z_print_compensation_mm": float(
                calibration.resin_z_print_compensation_mm
            )
        },
        "fiber": {
            "x_print_compensation_mm": float(
                calibration.fiber_x_print_compensation_mm
            ),
            "y_print_compensation_mm": float(
                calibration.fiber_y_print_compensation_mm
            ),
            "z_print_compensation_mm": float(
                calibration.fiber_z_print_compensation_mm
            ),
        },
    }
    p.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def calibration_relative_offsets(
    calibration: HeadCalibration,
    *,
    from_tool: str,
    to_tool: str,
) -> tuple[float, float, float]:
    resin = (0.0, 0.0, float(calibration.resin_z_print_compensation_mm))
    fiber = (
        float(calibration.fiber_x_print_compensation_mm),
        float(calibration.fiber_y_print_compensation_mm),
        float(calibration.fiber_z_print_compensation_mm),
    )
    tools = {"resin": resin, "fiber": fiber}
    if from_tool not in tools or to_tool not in tools:
        raise ValueError("from_tool and to_tool must be 'resin' or 'fiber'")
    src = tools[from_tool]
    dst = tools[to_tool]
    return tuple(dst[i] - src[i] for i in range(3))
