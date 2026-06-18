from gcode_planner.gcode_parser import parse_gcode_lines
from gcode_planner.types import MoveCommand


def test_g92_xyzabc_initializes_pose_without_motion():
    parsed = parse_gcode_lines([
        "G90",
        "M82",
        "G92 X1.0 Y2.0 Z3.0 A4.0 B5.0 C6.0 E0",
        "G1 Z4.0 F60",
    ])

    moves = [cmd for cmd in parsed if isinstance(cmd, MoveCommand)]

    assert len(moves) == 1
    assert moves[0].start_pos.x == 1.0
    assert moves[0].start_pos.y == 2.0
    assert moves[0].start_pos.z == 3.0
    assert moves[0].start_pos.a == 4.0
    assert moves[0].start_pos.b == 5.0
    assert moves[0].start_pos.c == 6.0
    assert moves[0].pos.z == 4.0
