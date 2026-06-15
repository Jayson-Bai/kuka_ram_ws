import math

from gcode_planner.gcode_parser import parse_gcode_lines
from gcode_planner.print_test_generator import (
    EXTRUSION_PER_MM3,
    RESIN_TOOL_ID,
    expand_test_values,
    generate_test_matrix_gcode,
    generate_test_line_gcode,
    generate_z_adjust_gcode,
)
from gcode_planner.types import ExtrudeWait, MoveCommand, ToolChangeCommand


def _moves(lines):
    return [cmd for cmd in parse_gcode_lines(lines) if isinstance(cmd, MoveCommand)]


def test_z_adjust_gcode_starts_from_current_rsi_correction_and_uses_resin_tool():
    lines = generate_z_adjust_gcode(
        start_pose=(0.0, 0.0, 1.5, 0.0, 0.0, 0.0),
        target_z=1.6,
        speed_mm_s=5.0,
    )
    parsed = parse_gcode_lines(lines)
    moves = [cmd for cmd in parsed if isinstance(cmd, MoveCommand)]
    tools = [cmd for cmd in parsed if isinstance(cmd, ToolChangeCommand)]

    assert tools[-1].tool == 1  # GCode T1 maps to internal resin tool id 2.
    assert moves[0].start_pos.z == 1.5
    assert moves[0].pos.z == 1.6
    assert moves[0].feedrate == 300.0
    assert moves[0].delta_e == 0.0


def test_test_line_gcode_uses_fixed_resin_line_width_and_finish_lift():
    lines = generate_test_line_gcode(
        start_pose=(1.0, 2.0, 0.4, 0.0, 0.0, 0.0),
        layer_height_mm=0.5,
        speed_mm_s=10.0,
        line_length_mm=200.0,
        finish_lift_mm=10.0,
    )
    parsed = parse_gcode_lines(lines)
    moves = [cmd for cmd in parsed if isinstance(cmd, MoveCommand)]
    tools = [cmd for cmd in parsed if isinstance(cmd, ToolChangeCommand)]

    assert RESIN_TOOL_ID == 2
    assert tools[-1].tool == 1
    print_move = next(cmd for cmd in moves if cmd.type == "PRINT")
    assert print_move.start_pos.x == 1.0
    assert print_move.pos.x == 201.0
    assert math.isclose(print_move.delta_e, 200.0 * 2.0 * 0.5 * EXTRUSION_PER_MM3)
    lift_move = moves[-1]
    assert lift_move.type == "TRAVEL"
    assert lift_move.pos.z == 10.4
    assert lift_move.delta_e == 0.0


def test_expand_test_values_accepts_single_value_and_inclusive_range():
    assert expand_test_values("0.5") == [0.5]
    assert expand_test_values("0.5-0.7") == [0.5, 0.6, 0.7]


def test_expand_test_values_rejects_descending_range():
    try:
        expand_test_values("1.0-0.8")
    except ValueError as exc:
        assert "不能小于" in str(exc)
    else:
        raise AssertionError("expected descending range to be rejected")


def test_test_matrix_gcode_generates_full_combinations_with_y_spacing_and_scaled_e():
    lines = generate_test_matrix_gcode(
        start_pose=(1.0, 2.0, 0.4, 0.0, 0.0, 0.0),
        layer_heights_mm=[0.5, 0.6],
        extrusion_scales=[0.8, 1.0],
        speed_mm_s=10.0,
        line_length_mm=300.0,
        y_spacing_mm=10.0,
        finish_lift_mm=10.0,
    )

    moves = _moves(lines)
    print_moves = [cmd for cmd in moves if cmd.type == "PRINT"]
    travel_moves = [cmd for cmd in moves if cmd.type == "TRAVEL"]

    assert len(print_moves) == 4
    assert [(cmd.start_pos.x, cmd.start_pos.y) for cmd in print_moves] == [
        (1.0, 2.0),
        (301.0, 12.0),
        (1.0, 22.0),
        (301.0, 32.0),
    ]
    assert [cmd.pos.x for cmd in print_moves] == [301.0, 1.0, 301.0, 1.0]
    assert [cmd.pos.z for cmd in print_moves] == [0.9, 0.9, 1.0, 1.0]

    expected_deltas = [
        300.0 * 2.0 * 0.5 * EXTRUSION_PER_MM3 * 0.8,
        300.0 * 2.0 * 0.5 * EXTRUSION_PER_MM3 * 1.0,
        300.0 * 2.0 * 0.6 * EXTRUSION_PER_MM3 * 0.8,
        300.0 * 2.0 * 0.6 * EXTRUSION_PER_MM3 * 1.0,
    ]
    for move, expected_delta in zip(print_moves, expected_deltas):
        assert math.isclose(move.delta_e, expected_delta)

    assert any(
        cmd.start_pos.x == 301.0
        and cmd.start_pos.y == 2.0
        and cmd.start_pos.z == 10.9
        and cmd.pos.x == 301.0
        and cmd.pos.y == 12.0
        and cmd.pos.z == 10.9
        and cmd.delta_e == 0.0
        for cmd in travel_moves
    )
    assert any(
        cmd.start_pos.x == 301.0
        and cmd.start_pos.y == 12.0
        and cmd.start_pos.z == 10.9
        and cmd.pos.x == 301.0
        and cmd.pos.y == 12.0
        and cmd.pos.z == 0.9
        and cmd.delta_e == 0.0
        for cmd in travel_moves
    )
    assert moves[-1].type == "TRAVEL"
    assert moves[-1].pos.x == 1.0
    assert moves[-1].pos.z == 11.0


def test_test_matrix_gcode_rejects_more_than_45_lines():
    try:
        generate_test_matrix_gcode(
            start_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            layer_heights_mm=[0.1, 0.2, 0.3, 0.4, 0.5],
            extrusion_scales=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
            speed_mm_s=10.0,
        )
    except ValueError as exc:
        assert "45" in str(exc)
    else:
        raise AssertionError("expected matrix line limit to be enforced")


def test_test_matrix_gcode_adds_prime_and_retract_from_equivalent_lengths():
    lines = generate_test_matrix_gcode(
        start_pose=(1.0, 2.0, 0.4, 0.0, 0.0, 0.0),
        layer_heights_mm=[0.5],
        extrusion_scales=[1.0],
        speed_mm_s=10.0,
        line_length_mm=300.0,
        prime_length_mm=5.0,
        retract_length_mm=3.0,
        prime_speed_mm_s=2.0,
        retract_speed_mm_s=8.0,
        finish_lift_mm=10.0,
    )

    parsed = parse_gcode_lines(lines)
    waits = [cmd for cmd in parsed if isinstance(cmd, ExtrudeWait)]
    print_move = next(
        cmd for cmd in parsed if isinstance(cmd, MoveCommand) and cmd.type == "PRINT"
    )

    e_per_path_mm = 2.0 * 0.5 * EXTRUSION_PER_MM3
    assert len(waits) == 2
    assert math.isclose(waits[0].delta_e, 5.0 * e_per_path_mm)
    assert math.isclose(waits[0].feedrate, 2.0 * 60.0)
    assert math.isclose(print_move.delta_e, 300.0 * e_per_path_mm)
    assert math.isclose(waits[1].delta_e, -3.0 * e_per_path_mm)
    assert math.isclose(waits[1].feedrate, 8.0 * 60.0)
