"""Convert external source paths into gcode_planner parsed commands."""

from __future__ import annotations

import math

import numpy as np

from path_processing_core.types import (
    ExtrudeWait,
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
            previous_pose = first_pose

            for wait in _path_retract_prime_waits(material_path.material, params, line, layer.index, subtype):
                commands.append(wait)
                current_e += wait.delta_e
                line += 1

            for row in material_path.points[1:]:
                next_pose = _offset_source_position(_position_from_row(row), params)
                segment_length = _distance(previous_pose, next_pose)
                delta_e = segment_length * e_per_mm
                current_e += delta_e
                commands.append(
                    MoveCommand(
                        type="PRINT",
                        cmd="G1",
                        start_pos=previous_pose,
                        pos=next_pose,
                        e_val=current_e,
                        delta_e=delta_e,
                        feedrate=feedrate,
                        line=line,
                        layer=layer.index,
                        subtype=subtype,
                        raw="external_npz_print",
                    )
                )
                line += 1
                previous_pose = next_pose

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
