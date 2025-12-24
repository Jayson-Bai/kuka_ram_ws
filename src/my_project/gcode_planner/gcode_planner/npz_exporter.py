"""
gcode_planner 的 npz 导出器（分片）：
- 解析后的指令 + 插值采样点，输出 npz 分片（二进制列存），与 RSI/uart 消费逻辑对齐。
- 事件来自 ToolChangeCommand、指定的 MCommand，以及 G92 重置挤出（ResetECommand）。
- 事件行复用事件发生前的上一帧位姿（保持不动），event_flag=1。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional
import os

from .types import (
    ParsedCommandList,
    MoveCommand,
    GlobalCurveCommand,
    ToolChangeCommand,
    MCommand,
    ResetECommand,
)
from .bspline_approximation import GlobalSplinePlanner
from .polynomial_interpolator import sample_global_curve


@dataclass
class CsvRow:
    seq: int
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float
    e: float
    tool_id: int
    move_type: str
    src_line: str
    event_flag: int
    event_type: str
    payload: str
    trigger_seq: Optional[int]


@dataclass
class _PendingEvent:
    event_type: str
    payload: str
    src_line: int
    tool_id: int


def export_npz(
    parsed_commands: ParsedCommandList,
    output_path: str,
    dt: float = 0.004,
    chunk_size: int = 100000,
) -> None:
    """
    导出 npz（分片）。
    - 按 4ms 采样（要求上游或本函数已将运动转换为 GlobalCurveCommand）。
    - 事件对齐：事件指令出现后，标记落在随后的第一个采样点行。
    - 速度规划由 sample_global_curve 内部的七阶多项式完成，此处不做额外处理。
    """
    rows: List[CsvRow] = []
    current_tool = 2  # 默认工具号（树脂，T1）
    seq = 0
    planner = GlobalSplinePlanner()

    buffer: List[MoveCommand] = []
    current_type: Optional[str] = None
    last_pose: Optional[CsvRow] = None

    def _append_sample(gc: GlobalCurveCommand):
        nonlocal seq, last_pose, rows
        samples = sample_global_curve(gc, dt=dt)
        if not samples:
            return
        move_lines: List[int] = [m.line for m in gc.original_moves] if gc.original_moves else [gc.line]
        if len(move_lines) > 1:
            src_lines = f"{move_lines[0]}-{move_lines[-1]}"
        else:
            src_lines = str(move_lines[0])

        for pt in samples:
            row = CsvRow(
                seq=seq,
                x=pt.pos.x,
                y=pt.pos.y,
                z=pt.pos.z,
                a=pt.pos.a,
                b=pt.pos.b,
                c=pt.pos.c,
                e=pt.e,
                tool_id=gc.type == "TRAVEL" and current_tool or gc.type == "PRINT" and current_tool or current_tool,
                move_type=gc.type,
                src_line=src_lines,
                event_flag=0,
                event_type="",
                payload="",
                trigger_seq=None,
            )
            rows.append(row)
            seq += 1
            last_pose = row

    def flush_moves():
        nonlocal buffer, current_type
        if not buffer:
            return

        def _make_gc(move):
            return GlobalCurveCommand(
                type=move.type,
                cmd="SPLINE",
                start_pos=move.start_pos,
                control_points=[move.pos, move.pos, move.pos],  # 起点+重复终点，表示直线
                e_val=move.e_val,
                delta_e=move.delta_e,
                feedrate=move.feedrate,
                line=move.line,
                raw=move.raw or "fallback_linear",
                constraints=[],
                original_moves=[move],
            )

        if len(buffer) == 1:
            gc_list = [_make_gc(buffer[0])]
        elif len(buffer) == 2:
            gc_list = [_make_gc(buffer[0]), _make_gc(buffer[1])]
        else:
            gc = planner.fit_global_curve(buffer)
            if gc is None:
                raise ValueError("B 样条拟合失败，无法导出 npz（段类型: %s, 段长度: %d)" % (current_type, len(buffer)))
            gc_list = [gc]

        for gc in gc_list:
            _append_sample(gc)
        buffer = []
        current_type = None

    def _emit_event(ev: _PendingEvent):
        nonlocal seq, last_pose, rows
        hold_row = last_pose or CsvRow(
            seq=seq,
            x=0.0,
            y=0.0,
            z=0.0,
            a=0.0,
            b=0.0,
            c=0.0,
            e=0.0,
            tool_id=ev.tool_id,
            move_type="TRAVEL",
            src_line=str(ev.src_line),
            event_flag=0,
            event_type="",
            payload="",
            trigger_seq=None,
        )
        rows.append(
            CsvRow(
                seq=seq,
                x=hold_row.x,
                y=hold_row.y,
                z=hold_row.z,
                a=hold_row.a,
                b=hold_row.b,
                c=hold_row.c,
                e=hold_row.e,
                tool_id=ev.tool_id,
                move_type="EVENT",
                src_line=str(ev.src_line),
                event_flag=1,
                event_type=ev.event_type,
                payload=ev.payload,
                trigger_seq=seq,
            )
        )
        seq += 1
        last_pose = hold_row

    for cmd in parsed_commands:
        # 事件收集：遇到事件前先冲掉当前轨迹段，保证事件贴在后续采样点
        if isinstance(cmd, (ToolChangeCommand, MCommand, ResetECommand)):
            flush_moves()
            if isinstance(cmd, ToolChangeCommand):
                mapped_tool = _map_gcode_tool(cmd.tool)
                current_tool = mapped_tool
                _emit_event(_PendingEvent(
                    event_type="tool_change_cf" if mapped_tool == 1 else "tool_change_resin",
                    payload=str(mapped_tool),
                    src_line=cmd.line,
                    tool_id=mapped_tool,
                ))
            elif isinstance(cmd, ResetECommand):
                _emit_event(_PendingEvent(
                    event_type="extrude_reset",
                    payload=str(cmd.val),
                    src_line=cmd.line,
                    tool_id=current_tool,
                ))
            else:
                ev = _mcommand_to_event(cmd, current_tool)
                if ev:
                    _emit_event(ev)
            continue

        # 轨迹分段收集
        if isinstance(cmd, MoveCommand):
            if cmd.is_pure_state_change:
                continue
            if current_type is None:
                current_type = cmd.type
            if cmd.type != current_type:
                flush_moves()
                current_type = cmd.type
            buffer.append(cmd)
            continue

        if isinstance(cmd, GlobalCurveCommand):
            # 如果上游已提供曲线，先冲掉当前 Move 段，再直接采样
            flush_moves()
            _append_sample(cmd)
            continue

    # 文件末尾冲掉残余 Move 段
    flush_moves()

    _npz_exporter(output_path, rows, chunk_size)


def _npz_exporter(output_path: str, rows: List[CsvRow], chunk_size: int) -> None:
    import numpy as np
    base, ext = os.path.splitext(output_path)
    single_path = output_path if ext.lower() == ".npz" else base + ".npz"
    part_base = base if ext.lower() == ".npz" else output_path

    move_type_map = {
        "TRAVEL": 0,
        "PRINT": 1,
        "TRAVEL_FIT": 2,
        "PRINT_FIT": 3,
        "EVENT": 4,
    }
    event_type_map = {
        "": 0,
        "heat_cf": 1,
        "heat_resin": 2,
        "fan_cf": 3,
        "fan_resin": 4,
        "extrude_reset": 5,
        "tool_change_cf": 6,
        "tool_change_resin": 7,
    }
    # 使用定长字节串，便于 C++ 侧 cnpy 读取
    move_type_keys = np.array(list(move_type_map.keys()), dtype="S32")
    move_type_vals = np.array(list(move_type_map.values()), dtype=np.uint8)
    event_type_keys = np.array(list(event_type_map.keys()), dtype="S32")
    event_type_vals = np.array(list(event_type_map.values()), dtype=np.uint8)

    n = len(rows)
    start = 0
    part = 0
    while start < n:
        end = min(start + chunk_size, n)
        chunk = rows[start:end]
        seq = np.array([r.seq for r in chunk], dtype=np.uint32)
        x = np.array([r.x for r in chunk], dtype=np.float32)
        y = np.array([r.y for r in chunk], dtype=np.float32)
        z = np.array([r.z for r in chunk], dtype=np.float32)
        a = np.array([r.a for r in chunk], dtype=np.float32)
        b = np.array([r.b for r in chunk], dtype=np.float32)
        c = np.array([r.c for r in chunk], dtype=np.float32)
        e = np.array([r.e for r in chunk], dtype=np.float32)
        tool_id = np.array([r.tool_id for r in chunk], dtype=np.uint8)
        move_type = np.array([move_type_map.get(r.move_type, 255) for r in chunk], dtype=np.uint8)
        src_line = np.array([r.src_line for r in chunk], dtype="S32")
        event_flag = np.array([r.event_flag for r in chunk], dtype=np.uint8)
        event_type = np.array([event_type_map.get(r.event_type, 255) for r in chunk], dtype=np.uint8)
        payload = np.array([str(r.payload) for r in chunk], dtype="S32")
        trigger_seq = np.array([r.trigger_seq if r.trigger_seq is not None else -1 for r in chunk], dtype=np.int32)

        out_path = (
            f"{part_base}_part{part:04d}.npz"
            if n > chunk_size
            else single_path
        )
        np.savez_compressed(
            out_path,
            seq=seq,
            x=x,
            y=y,
            z=z,
            a=a,
            b=b,
            c=c,
            e=e,
            tool_id=tool_id,
            move_type=move_type,
            src_line=src_line,
            event_flag=event_flag,
            event_type=event_type,
            payload=payload,
            trigger_seq=trigger_seq,
            move_type_vocab_keys=move_type_keys,
            move_type_vocab_vals=move_type_vals,
            event_type_vocab_keys=event_type_keys,
            event_type_vocab_vals=event_type_vals,
        )
        start = end
        part += 1


def _map_gcode_tool(gcode_tool: int) -> int:
    """将 GCode 中的 T0/T1 映射到内部工具号：1=纤维(T0)，2=树脂(T1)。"""
    if gcode_tool == 0:
        return 1
    if gcode_tool == 1:
        return 2
    return gcode_tool


def _mcommand_to_event(cmd: MCommand, current_tool: int) -> Optional[_PendingEvent]:
    """M 指令映射到事件名/负载；未覆盖的返回 None。"""
    code = cmd.code.upper()
    params = cmd.params or {}

    def _get_tool_from_params():
        if "T" in params:
            try:
                return _map_gcode_tool(int(params["T"]))
            except Exception:
                return current_tool
        return current_tool

    tool_id = _get_tool_from_params()
    src_line = cmd.line if cmd.line is not None else -1

    if code in ("M104", "M109"):  # 加热
        if "S" not in params:
            return None
        temp_val = params["S"]
        ev_type = "heat_cf" if tool_id == 1 else "heat_resin"
        return _PendingEvent(ev_type, str(temp_val), src_line, tool_id)

    if code == "M106":  # 风扇
        ev_type = "fan_cf" if tool_id == 1 else "fan_resin"
        return _PendingEvent(ev_type, "", src_line, tool_id)

    # 其他 M 指令：忽略（热床等）
    return None
