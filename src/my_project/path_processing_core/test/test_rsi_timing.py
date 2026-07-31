import json

import numpy as np
import pytest

from path_processing_core.npz_exporter import export_npz
from path_processing_core.rsi_timing import RsiTimingAccumulator
from path_processing_core.types import ExtrudeWait, MoveCommand, Position


def test_trajectory_rows_advance_by_dt_but_event_rows_do_not():
    timing = RsiTimingAccumulator(dt=0.004)
    assert timing.trajectory_time() == 0.0
    assert timing.append_trajectory_time() == 0.0
    assert timing.append_event_time() == 0.0
    assert timing.append_trajectory_time() == 0.004
    assert timing.append_trajectory_time() == 0.008
    assert timing.trajectory_rows == 3
    assert timing.event_rows_ignored == 1


def test_segment_metadata_preserves_acc_flat_dec_and_sequence_range():
    timing = RsiTimingAccumulator(dt=0.1)
    timing.start_segment(path_id=7, move_type="TRAVEL", start_seq=10)
    timing.append_trajectory_time()
    timing.append_trajectory_time()
    timing.finish_segment(t_acc_s=2.0, t_flat_s=3.5, t_dec_s=2.0, end_seq=11)

    assert timing.segments == [{
        "path_id": 7,
        "move_type": "TRAVEL",
        "start_seq": 10,
        "end_seq": 11,
        "duration_s": 0.1,
        "t_acc_s": 2.0,
        "t_flat_s": 3.5,
        "t_dec_s": 2.0,
    }]


def test_summary_is_json_serializable_and_reports_total_time():
    timing = RsiTimingAccumulator(dt=0.25)
    timing.append_trajectory_time("travel")
    timing.append_trajectory_time("wait")
    timing.append_event_time("cut")
    timing.append_event_time("tool_change_cf")
    timing.append_event_time("heat_cf")
    summary = timing.summary()

    assert summary["format"] == "rsi_print_timing"
    assert summary["version"] == 2
    assert summary["sample_period_s"] == 0.25
    assert summary["total_planned_time_s"] == 0.25
    assert summary["trajectory_rows"] == 2
    assert summary["planned_travel_time_s"] == 0.0
    assert summary["planned_wait_time_s"] == 0.25
    assert summary["planned_cut_count"] == 1
    assert summary["planned_tool_change_count"] == 1
    assert summary["planned_unquantified_event_count"] == 1


def test_all_rsi_points_are_counted_once_and_events_never_advance_time():
    timing = RsiTimingAccumulator(dt=0.004)
    for category in ("print", "travel", "wait", "cut", "print"):
        timing.append_trajectory_time(category)
    for event_type in ("cut", "tool_change_cf", "heat_cf", "extrude_reset"):
        timing.append_event_time(event_type)

    summary = timing.summary()
    category_total = sum(summary["trajectory_time_breakdown_s"].values())

    assert summary["total_planned_time_s"] == 4 * 0.004
    assert category_total == summary["total_planned_time_s"]
    assert summary["event_rows_ignored"] == 4


def test_stationary_prime_or_retract_uses_length_over_speed_as_rsi_time(tmp_path):
    start = Position(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    end = Position(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    commands = [
        MoveCommand(
            type="TRAVEL", cmd="G0", start_pos=start, pos=end,
            e_val=0.0, delta_e=0.0, feedrate=600.0,
            line=1, layer=0, subtype="TRAVEL", raw="timing_anchor",
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT", wait_sec=10.0 / 5.0,
            delta_e=-10.0, feedrate=5.0 * 60.0,
            line=2, layer=0, subtype="FIBER_PRINT",
            raw="stationary_retract",
        ),
    ]
    output = tmp_path / "stationary_retract.npz"

    export_npz(commands, str(output), dt=0.004, enable_extrude_wait=True)

    summary = json.loads(
        output.with_suffix(".timing.json").read_text(encoding="utf-8"))
    with np.load(output, allow_pickle=False) as data:
        wait_rows = data["timing_category"] == 2
        assert np.count_nonzero(wait_rows) == 500
        assert np.ptp(data["x"][wait_rows]) == 0.0
        assert np.ptp(data["y"][wait_rows]) == 0.0
        assert np.ptp(data["z"][wait_rows]) == 0.0
        assert data["e"][wait_rows][-1] == -10.0

    assert summary["planned_wait_time_s"] == pytest.approx(2.0)
