"""Convert external source paths into gcode_planner parsed commands."""

from __future__ import annotations

import math

import numpy as np

from path_processing_core.bspline_approximation import GlobalSplinePlanner
from path_processing_core.polynomial_interpolator import (
    _eval_bspline_point,
    _make_open_uniform_knots,
    _split_ctrl_components,
)
from path_processing_core.types import (
    ExtrudeWait,
    GlobalCurveCommand,
    MCommand,
    MoveCommand,
    ParsedCommandList,
    Position,
    ResetECommand,
    ToolChangeCommand,
)

from .process_params import ProcessParams
from .source_npz import MaterialPath, SourceJob


_RESIN_GCODE_TOOL = 1
_FIBER_GCODE_TOOL = 0
_EPS = 1e-9


def source_job_to_parsed_commands(job: SourceJob, params: ProcessParams) -> ParsedCommandList:
    commands: ParsedCommandList = []
    current_pose: Position | None = None
    current_tool: int | None = None
    current_e = 0.0
    line = 0

    line = _append_startup_head_events(commands, params, line)

    initial_travel_added = False

    for layer in job.layers:
        ordered_paths: list[MaterialPath] = []
        ordered_paths.extend(layer.resin_paths)
        ordered_paths.extend(layer.fiber_paths)
        for material_path in ordered_paths:
            tool = _tool_for_material(material_path.material)
            subtype = _subtype_for_material(material_path.material)
            first_pose = _offset_source_position(
                _position_from_row(material_path.points[0]), params
            )
            if not initial_travel_added:
                line = _append_initial_start_xy_travel(
                    commands, params, first_pose, line, layer.index
                )
                initial_travel_added = True
                current_pose = first_pose
            if current_tool != tool:
                commands.append(
                    ToolChangeCommand(
                        type="TOOL_CHANGE",
                        tool=tool,
                        line=line,
                        layer=layer.index,
                        subtype=subtype,
                        raw=f"T{tool}",
                    )
                )
                line += 1
                commands.append(
                    ResetECommand(
                        type="RESET_E",
                        val=0.0,
                        line=line,
                        layer=layer.index,
                        subtype=subtype,
                        raw="G92 E0",
                        pose=first_pose,
                    )
                )
                line += 1
                current_tool = tool
                current_e = 0.0

            if current_pose is not None and _distance(current_pose, first_pose) > _EPS:
                commands.append(
                    MoveCommand(
                        type="TRAVEL",
                        cmd="G0",
                        start_pos=current_pose,
                        pos=first_pose,
                        e_val=current_e,
                        delta_e=0.0,
                        feedrate=float(params.travel_feed_mm_s) * 60.0,
                        line=line,
                        layer=layer.index,
                        subtype="TRAVEL",
                        raw="external_npz_travel",
                    )
                )
                line += 1

            e_per_mm = _e_per_mm_for_material(material_path.material, params)
            feedrate = _feed_mm_s_for_material(material_path.material, params) * 60.0
            source_positions = [
                _offset_source_position(_position_from_row(row), params)
                for row in material_path.points
            ]
            previous_pose = source_positions[-1]

            for wait in _path_retract_prime_waits(material_path.material, params, line, layer.index, subtype):
                commands.append(wait)
                current_e += wait.delta_e
                line += 1

            print_moves, current_e = _print_moves_from_positions(
                source_positions=source_positions,
                e_start=current_e,
                e_per_mm=e_per_mm,
                feedrate=feedrate,
                line=line,
                layer=layer.index,
                subtype=subtype,
            )
            curve = _validated_spline_or_polyline(print_moves, source_positions, params)
            curve.layer = layer.index
            curve.subtype = subtype
            commands.append(curve)
            line += 1

            if material_path.material == "F":
                commands.append(
                    MCommand(
                        type="M_COMMAND",
                        code="CUT",
                        params={"P": 1.0},
                        line=line,
                        layer=layer.index,
                        subtype=subtype,
                        raw="external_npz_cut",
                        tool=tool,
                    )
                )
                line += 1

            for wait in _path_retract_prime_waits(material_path.material, params, line, layer.index, subtype):
                commands.append(wait)
                current_e += wait.delta_e
                line += 1

            current_pose = previous_pose

    return commands


def _path_retract_prime_waits(
    material: str,
    params: ProcessParams,
    line: int,
    layer: int,
    subtype: str,
) -> list[ExtrudeWait]:
    if material == "R":
        process = params.resin
    elif material == "F":
        process = params.fiber
    else:
        raise ValueError(f"unknown material: {material}")

    waits: list[ExtrudeWait] = []
    retract = _make_extrude_wait(
        delta_e=-float(process.retract_length_mm),
        speed_mm_s=float(process.retract_speed_mm_s),
        line=line,
        layer=layer,
        subtype=subtype,
        raw="external_npz_retract",
    )
    if retract is not None:
        waits.append(retract)
        line += 1

    prime = _make_extrude_wait(
        delta_e=float(process.prime_length_mm),
        speed_mm_s=float(process.prime_speed_mm_s),
        line=line,
        layer=layer,
        subtype=subtype,
        raw="external_npz_prime",
    )
    if prime is not None:
        waits.append(prime)
    return waits


def _make_extrude_wait(
    *,
    delta_e: float,
    speed_mm_s: float,
    line: int,
    layer: int,
    subtype: str,
    raw: str,
) -> ExtrudeWait | None:
    if abs(delta_e) <= _EPS:
        return None
    if speed_mm_s <= 0.0:
        raise ValueError("extrude wait speed must be > 0")
    return ExtrudeWait(
        type="EXTRUDE_WAIT",
        wait_sec=abs(delta_e) / speed_mm_s,
        delta_e=delta_e,
        feedrate=speed_mm_s * 60.0,
        line=line,
        layer=layer,
        subtype=subtype,
        raw=raw,
    )


def _print_moves_from_positions(
    *,
    source_positions: list[Position],
    e_start: float,
    e_per_mm: float,
    feedrate: float,
    line: int,
    layer: int,
    subtype: str,
) -> tuple[list[MoveCommand], float]:
    moves: list[MoveCommand] = []
    current_e = e_start
    previous = source_positions[0]
    for offset, next_pos in enumerate(source_positions[1:]):
        delta_e = _distance(previous, next_pos) * e_per_mm
        current_e += delta_e
        moves.append(
            MoveCommand(
                type="PRINT",
                cmd="G1",
                start_pos=previous,
                pos=next_pos,
                e_val=current_e,
                delta_e=delta_e,
                feedrate=feedrate,
                line=line + offset,
                layer=layer,
                subtype=subtype,
                raw="external_npz_print_source",
            )
        )
        previous = next_pos
    return moves, current_e


def _validated_spline_or_polyline(
    moves: list[MoveCommand],
    source_positions: list[Position],
    params: ProcessParams,
) -> GlobalCurveCommand:
    if not moves:
        return GlobalCurveCommand(
            type="PRINT",
            cmd="POLYLINE",
            start_pos=source_positions[0],
            control_points=[],
            e_val=0.0,
            delta_e=0.0,
            feedrate=0.0,
            line=0,
            raw="external_npz_empty_polyline",
            original_moves=[],
        )

    smoothed_positions = _prepare_external_smoothing_positions(source_positions, params)
    if not _positions_changed(source_positions, smoothed_positions):
        return _make_polyline_curve(moves, raw="external_npz_polyline")

    smoothed_moves, _ = _print_moves_from_positions(
        source_positions=smoothed_positions,
        e_start=moves[0].e_val - moves[0].delta_e,
        e_per_mm=_e_per_mm_from_moves(moves),
        feedrate=moves[0].feedrate,
        line=moves[0].line,
        layer=moves[0].layer,
        subtype=moves[0].subtype,
    )
    return _make_polyline_curve(smoothed_moves, raw="external_npz_smoothed_polyline")


def _fit_validated_spline(
    moves: list[MoveCommand],
    source_positions: list[Position],
    params: ProcessParams,
) -> GlobalCurveCommand | None:
    curve = GlobalSplinePlanner().fit_global_curve(
        moves,
        corner_angle_deg=float(params.corner_angle_deg),
        corner_retreat_ratio=float(params.corner_retreat_ratio),
        density=max(0, int(params.density)),
        degree=max(1, int(params.degree)),
        max_fit_points=max(2, int(params.max_fit_points_per_segment)),
    )
    if curve is None or (curve.cmd or "").upper() != "SPLINE":
        return None

    max_error = max(0.0, float(params.spline_max_error_mm))
    max_angle = max(0.0, float(params.spline_max_angle_deg))
    if _curve_max_bidirectional_error_mm(curve, source_positions) > max_error:
        return None
    if _curve_max_turn_angle_deg(
        curve,
        source_positions,
        min_segment_mm=max(_EPS, float(params.source_merge_distance_mm)),
    ) > max_angle:
        return None

    curve.type = "PRINT"
    curve.raw = "external_npz_validated_bspline"
    curve.original_moves = list(moves)
    return curve


def _make_polyline_curve(moves: list[MoveCommand], raw: str) -> GlobalCurveCommand:
    first = moves[0]
    last = moves[-1]
    return GlobalCurveCommand(
        type="PRINT",
        cmd="POLYLINE",
        start_pos=first.start_pos,
        control_points=[move.pos for move in moves],
        e_val=last.e_val,
        delta_e=sum(move.delta_e for move in moves),
        feedrate=first.feedrate,
        line=first.line,
        raw=raw,
        original_moves=list(moves),
    )


def _curve_max_bidirectional_error_mm(
    curve: GlobalCurveCommand,
    source_positions: list[Position],
) -> float:
    curve_points = _sample_spline_positions(curve, source_positions)
    if len(curve_points) < 2 or len(source_positions) < 2:
        return 0.0

    max_curve_to_source = max(
        _point_to_polyline_distance(point, source_positions)
        for point in curve_points
    )
    max_source_to_curve = max(
        _point_to_polyline_distance(point, curve_points)
        for point in source_positions
    )
    return max(max_curve_to_source, max_source_to_curve)


def _curve_max_turn_angle_deg(
    curve: GlobalCurveCommand,
    source_positions: list[Position],
    *,
    min_segment_mm: float,
) -> float:
    points = _sample_spline_positions(
        curve,
        source_positions,
        sample_count=max(128, min(1000, len(source_positions) * 4)),
    )
    return _polyline_max_turn_angle_deg(points, min_segment_mm=min_segment_mm)


def _prepare_external_smoothing_positions(
    positions: list[Position],
    params: ProcessParams,
) -> list[Position]:
    if len(positions) < 3:
        return positions
    merged = _merge_short_source_segments(
        positions, min_distance=max(0.0, float(params.source_merge_distance_mm))
    )
    if len(merged) < 3:
        return merged
    return _blend_sharp_corners(
        merged,
        angle_threshold_deg=max(0.0, float(params.corner_angle_deg)),
        retreat_ratio=max(0.0, min(1.0, float(params.corner_retreat_ratio))),
        retreat_max_mm=max(0.0, float(params.corner_retreat_max_mm)),
        blend_segments=max(2, int(params.corner_blend_segments)),
    )


def _merge_short_source_segments(
    positions: list[Position],
    *,
    min_distance: float,
) -> list[Position]:
    if len(positions) <= 2 or min_distance <= _EPS:
        return positions
    merged = [positions[0]]
    for point in positions[1:-1]:
        if _distance(merged[-1], point) >= min_distance:
            merged.append(point)
    if _distance(merged[-1], positions[-1]) > _EPS or len(merged) == 1:
        merged.append(positions[-1])
    else:
        merged[-1] = positions[-1]
    return merged


def _blend_sharp_corners(
    positions: list[Position],
    *,
    angle_threshold_deg: float,
    retreat_ratio: float,
    retreat_max_mm: float,
    blend_segments: int,
) -> list[Position]:
    retreats = [0.0] * len(positions)
    for index in range(1, len(positions) - 1):
        prev_pos = positions[index - 1]
        corner = positions[index]
        next_pos = positions[index + 1]
        in_len = _distance(prev_pos, corner)
        out_len = _distance(corner, next_pos)
        turn_angle = _turn_angle_deg(prev_pos, corner, next_pos)
        if (
            in_len > _EPS
            and out_len > _EPS
            and turn_angle >= angle_threshold_deg
            and retreat_ratio > _EPS
        ):
            retreat = min(in_len * retreat_ratio, out_len * retreat_ratio)
            if retreat_max_mm > _EPS:
                retreat = min(retreat, retreat_max_mm)
            retreats[index] = retreat

    for seg_index, (start, end) in enumerate(zip(positions, positions[1:])):
        seg_len = _distance(start, end)
        if seg_len <= _EPS:
            continue
        left = retreats[seg_index] if 0 < seg_index < len(positions) - 1 else 0.0
        right_index = seg_index + 1
        right = retreats[right_index] if 0 < right_index < len(positions) - 1 else 0.0
        total = left + right
        limit = seg_len * 0.9
        if total > limit and total > _EPS:
            scale = limit / total
            if left > 0.0:
                retreats[seg_index] *= scale
            if right > 0.0:
                retreats[right_index] *= scale

    out = [positions[0]]
    for index in range(1, len(positions) - 1):
        prev_pos = positions[index - 1]
        corner = positions[index]
        next_pos = positions[index + 1]
        retreat = retreats[index]
        if retreat <= _EPS:
            _append_distinct_position(out, corner)
            continue

        in_len = _distance(prev_pos, corner)
        out_len = _distance(corner, next_pos)
        entry = _position_along(corner, prev_pos, retreat / in_len)
        exit_pos = _position_along(corner, next_pos, retreat / out_len)
        _append_distinct_position(out, entry)
        for step in range(1, blend_segments):
            t = step / blend_segments
            _append_distinct_position(out, _quadratic_position(entry, corner, exit_pos, t))
        _append_distinct_position(out, exit_pos)

    _append_distinct_position(out, positions[-1])
    return out


def _polyline_max_turn_angle_deg(points: list[Position], *, min_segment_mm: float) -> float:
    max_angle = 0.0
    for prev_pos, corner, next_pos in zip(points, points[1:], points[2:]):
        if (
            _distance(prev_pos, corner) < min_segment_mm
            or _distance(corner, next_pos) < min_segment_mm
        ):
            continue
        max_angle = max(max_angle, _turn_angle_deg(prev_pos, corner, next_pos))
    return max_angle


def _turn_angle_deg(prev_pos: Position, corner: Position, next_pos: Position) -> float:
    ax = corner.x - prev_pos.x
    ay = corner.y - prev_pos.y
    az = corner.z - prev_pos.z
    bx = next_pos.x - corner.x
    by = next_pos.y - corner.y
    bz = next_pos.z - corner.z
    a_len = math.sqrt(ax * ax + ay * ay + az * az)
    b_len = math.sqrt(bx * bx + by * by + bz * bz)
    if a_len <= _EPS or b_len <= _EPS:
        return 0.0
    dot = max(-1.0, min(1.0, (ax * bx + ay * by + az * bz) / (a_len * b_len)))
    return math.degrees(math.acos(dot))


def _position_along(start: Position, end: Position, ratio: float) -> Position:
    return Position(
        x=start.x + (end.x - start.x) * ratio,
        y=start.y + (end.y - start.y) * ratio,
        z=start.z + (end.z - start.z) * ratio,
        a=start.a + (end.a - start.a) * ratio,
        b=start.b + (end.b - start.b) * ratio,
        c=start.c + (end.c - start.c) * ratio,
    )


def _quadratic_position(
    start: Position,
    control: Position,
    end: Position,
    t: float,
) -> Position:
    omt = 1.0 - t
    return Position(
        x=omt * omt * start.x + 2.0 * omt * t * control.x + t * t * end.x,
        y=omt * omt * start.y + 2.0 * omt * t * control.y + t * t * end.y,
        z=omt * omt * start.z + 2.0 * omt * t * control.z + t * t * end.z,
        a=omt * omt * start.a + 2.0 * omt * t * control.a + t * t * end.a,
        b=omt * omt * start.b + 2.0 * omt * t * control.b + t * t * end.b,
        c=omt * omt * start.c + 2.0 * omt * t * control.c + t * t * end.c,
    )


def _append_distinct_position(points: list[Position], point: Position) -> None:
    if not points or _distance(points[-1], point) > _EPS:
        points.append(point)


def _positions_changed(before: list[Position], after: list[Position]) -> bool:
    if len(before) != len(after):
        return True
    return any(_distance(left, right) > _EPS for left, right in zip(before, after))


def _e_per_mm_from_moves(moves: list[MoveCommand]) -> float:
    for move in moves:
        length = _distance(move.start_pos, move.pos)
        if length > _EPS:
            return move.delta_e / length
    return 0.0


def _sample_spline_positions(
    curve: GlobalCurveCommand,
    source_positions: list[Position],
    sample_count: int | None = None,
) -> list[Position]:
    ctrl = [curve.start_pos] + list(curve.control_points)
    degree = min(3, len(ctrl) - 1)
    if len(ctrl) < 2 or degree < 1:
        return ctrl

    if sample_count is None:
        sample_count = max(64, min(800, len(source_positions) * 2))
    knots = _make_open_uniform_knots(len(ctrl), degree)
    u_min = knots[degree]
    u_max = knots[len(ctrl)]
    ctrl_xyzabc = _split_ctrl_components(ctrl)
    span = degree
    samples: list[Position] = []
    for idx in range(sample_count + 1):
        u = u_min + (u_max - u_min) * idx / sample_count
        point, span = _eval_bspline_point(u, degree, knots, ctrl_xyzabc, len(ctrl), span)
        samples.append(point)
    return samples


def _point_to_polyline_distance(point: Position, polyline: list[Position]) -> float:
    if len(polyline) == 1:
        return _distance(point, polyline[0])
    return min(
        _point_to_segment_distance(point, start, end)
        for start, end in zip(polyline, polyline[1:])
    )


def _point_to_segment_distance(point: Position, start: Position, end: Position) -> float:
    vx = end.x - start.x
    vy = end.y - start.y
    vz = end.z - start.z
    wx = point.x - start.x
    wy = point.y - start.y
    wz = point.z - start.z
    denom = vx * vx + vy * vy + vz * vz
    if denom <= _EPS:
        return _distance(point, start)
    t = max(0.0, min(1.0, (wx * vx + wy * vy + wz * vz) / denom))
    projection = Position(
        x=start.x + vx * t,
        y=start.y + vy * t,
        z=start.z + vz * t,
        a=0.0,
        b=0.0,
        c=0.0,
    )
    return _distance(point, projection)


def _polyline_length(points: list[Position]) -> float:
    return sum(_distance(start, end) for start, end in zip(points, points[1:]))


def _offset_source_position(position: Position, params: ProcessParams) -> Position:
    return Position(
        x=position.x + float(params.start_x_mm),
        y=position.y + float(params.start_y_mm),
        z=position.z,
        a=position.a,
        b=position.b,
        c=position.c,
    )


def _append_initial_start_xy_travel(
    commands: ParsedCommandList,
    params: ProcessParams,
    first_pose: Position,
    line: int,
    layer: int,
) -> int:
    if abs(float(params.start_x_mm)) <= _EPS and abs(float(params.start_y_mm)) <= _EPS:
        return line
    start_pose = Position(
        x=0.0,
        y=0.0,
        z=first_pose.z,
        a=float(params.default_a),
        b=float(params.default_b),
        c=float(params.default_c),
    )
    if _distance(start_pose, first_pose) <= _EPS:
        return line
    commands.append(
        MoveCommand(
            type="TRAVEL",
            cmd="G0",
            start_pos=start_pose,
            pos=first_pose,
            e_val=0.0,
            delta_e=0.0,
            feedrate=float(params.travel_feed_mm_s) * 60.0,
            line=line,
            layer=layer,
            subtype="TRAVEL",
            raw="external_npz_start_xy_travel",
        )
    )
    return line + 1


def _position_from_row(row: np.ndarray) -> Position:
    return Position(
        x=float(row[0]),
        y=float(row[1]),
        z=float(row[2]),
        a=float(row[3]),
        b=float(row[4]),
        c=float(row[5]),
    )


def _distance(a: Position, b: Position) -> float:
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)


def _tool_for_material(material: str) -> int:
    if material == "R":
        return _RESIN_GCODE_TOOL
    if material == "F":
        return _FIBER_GCODE_TOOL
    raise ValueError(f"unknown material: {material}")


def _subtype_for_material(material: str) -> str:
    if material == "R":
        return "RESIN_PRINT"
    if material == "F":
        return "FIBER_PRINT"
    raise ValueError(f"unknown material: {material}")


def _e_per_mm_for_material(material: str, params: ProcessParams) -> float:
    if material == "R":
        return params.resin.e_per_mm()
    if material == "F":
        return params.fiber.e_per_mm()
    raise ValueError(f"unknown material: {material}")


def _feed_mm_s_for_material(material: str, params: ProcessParams) -> float:
    if material == "R":
        return float(params.resin.feed_mm_s)
    if material == "F":
        return float(params.fiber.feed_mm_s)
    raise ValueError(f"unknown material: {material}")


def _append_startup_head_events(commands: ParsedCommandList, params: ProcessParams, line: int) -> int:
    for material, code in (("R", "M106"), ("F", "M106"), ("R", "M104"), ("F", "M104")):
        process = params.resin if material == "R" else params.fiber
        gcode_tool = _tool_for_material(material)
        subtype = _subtype_for_material(material)
        if code == "M104":
            if process.temperature_c <= 0:
                continue
            commands.append(
                MCommand(
                    type="M_COMMAND",
                    code="M104",
                    params={"S": float(process.temperature_c), "T": float(gcode_tool)},
                    line=line,
                    layer=0,
                    subtype=subtype,
                    raw=f"M104 T{gcode_tool} S{process.temperature_c}",
                    tool=gcode_tool,
                )
            )
        else:
            commands.append(
                MCommand(
                    type="M_COMMAND",
                    code="M106" if process.fan_enabled else "M107",
                    params={"T": float(gcode_tool)},
                    line=line,
                    layer=0,
                    subtype=subtype,
                    raw=("M106" if process.fan_enabled else "M107") + f" T{gcode_tool}",
                    tool=gcode_tool,
                )
            )
        line += 1
    return line
