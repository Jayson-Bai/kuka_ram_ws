"""Insert a resin prime line into parsed GCode commands."""

from __future__ import annotations

import copy
import math

from path_processing_core.types import (
    CurveCommand,
    GlobalCurveCommand,
    MoveCommand,
    ParsedCommandList,
    Position,
    ResetECommand,
)

_EPS = 1e-9


def insert_resin_primeline(
    commands: ParsedCommandList,
    *,
    length_mm: float = 100.0,
    y_offset_mm: float = 10.0,
) -> ParsedCommandList:
    """Return commands with a 100 mm resin prime line before the first print move.

    The line reuses the first print move's inferred E/mm and feedrate so formal GCode
    exports keep their existing material allocation. Subsequent absolute E payloads are
    shifted by the added prime-line extrusion until the next explicit G92 reset.
    """
    if length_mm <= _EPS:
        return list(commands)
    first_index, first_print = _first_print_command(commands)
    if first_index is None or first_print is None:
        return list(commands)

    first_len = _command_length(first_print)
    first_delta_e = float(getattr(first_print, "delta_e", 0.0))
    if first_len <= _EPS or first_delta_e <= _EPS:
        return list(commands)

    e_per_mm = first_delta_e / first_len
    added_e = length_mm * e_per_mm
    e_start = float(first_print.e_val) - first_delta_e
    line_start = Position(
        x=float(first_print.start_pos.x),
        y=float(first_print.start_pos.y) - float(y_offset_mm),
        z=float(first_print.start_pos.z),
        a=float(first_print.start_pos.a),
        b=float(first_print.start_pos.b),
        c=float(first_print.start_pos.c),
    )
    line_end = Position(
        x=line_start.x + float(length_mm),
        y=line_start.y,
        z=line_start.z,
        a=line_start.a,
        b=line_start.b,
        c=line_start.c,
    )
    primeline = MoveCommand(
        type="PRINT",
        cmd="G1",
        start_pos=line_start,
        pos=line_end,
        e_val=e_start + added_e,
        delta_e=added_e,
        feedrate=float(first_print.feedrate),
        line=int(first_print.line),
        layer=int(first_print.layer),
        subtype="RESIN_PRINT",
        raw="gcode_primeline",
    )
    return_travel = MoveCommand(
        type="TRAVEL",
        cmd="G0",
        start_pos=line_end,
        pos=copy.deepcopy(first_print.start_pos),
        e_val=e_start + added_e,
        delta_e=0.0,
        feedrate=float(first_print.feedrate),
        line=int(first_print.line),
        layer=int(first_print.layer),
        subtype="TRAVEL",
        raw="gcode_primeline_return_travel",
    )

    out: ParsedCommandList = list(commands[:first_index])
    out.extend([primeline, return_travel])
    e_offset = added_e
    for cmd in commands[first_index:]:
        cloned = copy.deepcopy(cmd)
        if isinstance(cloned, ResetECommand):
            e_offset = 0.0
        elif isinstance(cloned, (MoveCommand, CurveCommand, GlobalCurveCommand)):
            cloned.e_val = float(cloned.e_val) + e_offset
        out.append(cloned)
    return out


def _first_print_command(commands: ParsedCommandList):
    for index, cmd in enumerate(commands):
        if isinstance(cmd, (MoveCommand, CurveCommand, GlobalCurveCommand)):
            if cmd.type == "PRINT" and float(cmd.delta_e) > _EPS:
                return index, cmd
    return None, None


def _command_length(cmd) -> float:
    points = [cmd.start_pos]
    if isinstance(cmd, MoveCommand):
        points.append(cmd.pos)
    else:
        points.extend(cmd.control_points)
    return sum(_distance(start, end) for start, end in zip(points, points[1:]))


def _distance(a: Position, b: Position) -> float:
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
