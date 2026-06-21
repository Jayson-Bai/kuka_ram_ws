import json
from datetime import datetime

from gcode_planner.head_calibration import (
    DEFAULT_HEAD_CALIBRATION,
    HeadCalibration,
    calibration_relative_offsets,
    load_head_calibration,
    save_head_calibration,
)


def test_load_head_calibration_returns_defaults_when_file_missing(tmp_path):
    path = tmp_path / "head_offsets.json"

    cal = load_head_calibration(path)

    assert cal.resin_z_print_compensation_mm == 0.0
    assert cal.fiber_x_print_compensation_mm == 0.0
    assert cal.fiber_y_print_compensation_mm == 0.0
    assert cal.fiber_z_print_compensation_mm == 0.0
    assert DEFAULT_HEAD_CALIBRATION["resin"]["z_print_compensation_mm"] == 0.0


def test_load_head_calibration_returns_defaults_for_malformed_json(tmp_path):
    path = tmp_path / "head_offsets.json"
    path.write_text("{not valid json", encoding="utf-8")

    assert load_head_calibration(path) == HeadCalibration()


def test_load_head_calibration_preserves_present_fields_and_defaults_missing_fields(
    tmp_path,
):
    path = tmp_path / "head_offsets.json"
    path.write_text(
        json.dumps(
            {
                "resin": {"z_print_compensation_mm": "-2.5"},
                "fiber": {"x_print_compensation_mm": 3.5},
            }
        ),
        encoding="utf-8",
    )

    cal = load_head_calibration(path)

    assert cal == HeadCalibration(
        resin_z_print_compensation_mm=-2.5,
        fiber_x_print_compensation_mm=3.5,
        fiber_y_print_compensation_mm=0.0,
        fiber_z_print_compensation_mm=0.0,
    )


def test_save_head_calibration_overwrites_current_file_with_timestamp(tmp_path):
    path = tmp_path / "head_offsets.json"
    save_head_calibration(
        HeadCalibration(
            resin_z_print_compensation_mm=-20.0,
            fiber_x_print_compensation_mm=5.0,
            fiber_y_print_compensation_mm=4.0,
            fiber_z_print_compensation_mm=-25.0,
        ),
        path,
    )

    data = json.loads(path.read_text(encoding="utf-8"))

    assert datetime.fromisoformat(data["updated_at"])
    assert data["resin"] == {"z_print_compensation_mm": -20.0}
    assert data["fiber"] == {
        "x_print_compensation_mm": 5.0,
        "y_print_compensation_mm": 4.0,
        "z_print_compensation_mm": -25.0,
    }

    save_head_calibration(HeadCalibration(resin_z_print_compensation_mm=-1.0), path)
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["resin"]["z_print_compensation_mm"] == -1.0
    assert data["fiber"]["x_print_compensation_mm"] == 0.0


def test_save_head_calibration_round_trips_through_load_head_calibration(tmp_path):
    path = tmp_path / "head_offsets.json"
    expected = HeadCalibration(
        resin_z_print_compensation_mm=-20.0,
        fiber_x_print_compensation_mm=5.0,
        fiber_y_print_compensation_mm=4.0,
        fiber_z_print_compensation_mm=-25.0,
    )

    save_head_calibration(expected, path)

    assert load_head_calibration(path) == expected


def test_calibration_relative_offsets_use_target_minus_current_head():
    cal = HeadCalibration(
        resin_z_print_compensation_mm=-20.0,
        fiber_x_print_compensation_mm=5.0,
        fiber_y_print_compensation_mm=4.0,
        fiber_z_print_compensation_mm=-25.0,
    )

    assert calibration_relative_offsets(cal, from_tool="resin", to_tool="fiber") == (
        5.0,
        4.0,
        -5.0,
    )
    assert calibration_relative_offsets(cal, from_tool="fiber", to_tool="resin") == (
        -5.0,
        -4.0,
        5.0,
    )
