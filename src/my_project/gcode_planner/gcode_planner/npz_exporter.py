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
import time
import json
import re

from .types import (
    ParsedCommandList,
    MoveCommand,
    GlobalCurveCommand,
    ToolChangeCommand,
    MCommand,
    ResetECommand,
)
from .bspline_approximation import GlobalSplinePlanner
from .polynomial_interpolator import sample_global_curve_iter


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
    corner_angle_deg: float = 10.0,
    corner_retreat_ratio: float = 0.2,
    density: int = 0,
    degree: int = 3,
    default_feed_mm_s: float = 10.0,
    export_sleep_ms: int = 0,
    export_yield_every: int = 0,
    split_by_layer_type: bool = False,
    plot_layer_xy: bool = False,
    plot_stride: int = 5,
) -> None:
    """
    导出 npz（分片）。
    - 按 4ms 采样（要求上游或本函数已将运动转换为 GlobalCurveCommand）。
    - 事件对齐：事件指令出现后，标记落在随后的第一个采样点行。
    - 速度规划由 sample_global_curve 内部的七阶多项式完成，此处不做额外处理。
    """
    last_pose_map = {}
    current_tool = 2  # 默认工具号（树脂，T1）
    seq = 0
    planner = GlobalSplinePlanner()

    buffer: List[MoveCommand] = []
    current_type: Optional[str] = None
    current_layer: Optional[int] = None
    current_subtype: Optional[str] = None
    last_pose: Optional[CsvRow] = None
    last_feedrate_mm_min: Optional[float] = None

    # 预先定义 vocab，确保分片一致
    import numpy as np
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
    move_type_keys = np.array(list(move_type_map.keys()), dtype="S32")
    move_type_vals = np.array(list(move_type_map.values()), dtype=np.uint8)
    event_type_keys = np.array(list(event_type_map.keys()), dtype="S32")
    event_type_vals = np.array(list(event_type_map.values()), dtype=np.uint8)

    def _sanitize(s: str) -> str:
        out = []
        for ch in s.strip():
            if ch.isalnum():
                out.append(ch)
            elif ch in (" ", "-", "_"):
                out.append("_" if ch == " " else ch)
            elif ch == "/":
                out.append("-")
        return "".join(out) or "UNKNOWN"

    def _normalize_subtype(s: str) -> str:
        return "TRAVEL" if (not s or s == "UNKNOWN") else s

    base, ext = os.path.splitext(output_path)
    base_no_ext = base if ext.lower() == ".npz" else output_path
    base_dir = os.path.dirname(base_no_ext)
    base_name = os.path.basename(base_no_ext)
    base_name = re.sub(r"_layer_\\d{4}$", "", base_name)
    base_root = os.path.join(base_dir, base_name) if base_dir else base_name

    class _Writer:
        def __init__(self, base_path: str):
            self.base_path = base_path
            self.part = 0
            self.wrote_any = False
            self.rows: List[CsvRow] = []
            self.last_seq: Optional[int] = None

        def add(self, row: CsvRow):
            self.rows.append(row)
            self.last_seq = row.seq
            if len(self.rows) >= chunk_size:
                self.flush()

        def flush(self):
            if not self.rows:
                return
            out_dir = os.path.dirname(self.base_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            out_path = f"{self.base_path}_part{self.part:04d}.npz"
            chunk = self.rows
            self.rows = []
            seq_arr = np.array([r.seq for r in chunk], dtype=np.uint32)
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

            np.savez_compressed(
                out_path,
                seq=seq_arr,
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
            self.part += 1
            self.wrote_any = True

        def finalize(self):
            self.flush()
            if self.wrote_any and self.part == 1:
                only_part = f"{self.base_path}_part0000.npz"
                final_path = f"{self.base_path}.npz"
                if os.path.exists(only_part):
                    os.replace(only_part, final_path)

    writers = {}
    manifest = []
    manifest_by_key = {}
    occ_counters = {}

    def _writer_for(layer: int, subtype: str, occ: int) -> _Writer:
        if not split_by_layer_type:
            key = ("_all_", "_all_")
            if key not in writers:
                writers[key] = _Writer(base_no_ext)
            return writers[key]
        subtype = _normalize_subtype(subtype)
        key = (layer, subtype, occ)
        if key not in writers:
            safe_subtype = _sanitize(subtype)
            layer_dir = os.path.join(base_root, f"layer_{layer:04d}")
            base_path = os.path.join(layer_dir, f"{base_name}_layer_{layer:04d}_type_{safe_subtype}_occ_{occ:04d}")
            writers[key] = _Writer(base_path)
            entry = {
                "layer": layer,
                "type": subtype,
                "occ": occ,
                "base_path": base_path,
                "start_seq": seq,
                "end_seq": None,
            }
            manifest.append(entry)
            manifest_by_key[key] = entry
        return writers[key]

    def _ensure_segment(layer: int, subtype: str) -> int:
        key = (layer, subtype)
        occ = occ_counters.get(key, 0) + 1
        occ_counters[key] = occ
        _writer_for(layer, subtype, occ)
        return occ

    processed_rows = 0

    def _maybe_yield():
        nonlocal processed_rows
        if export_yield_every <= 0 or export_sleep_ms <= 0:
            return
        if processed_rows % export_yield_every == 0:
            time.sleep(export_sleep_ms / 1000.0)

    def _append_sample(gc: GlobalCurveCommand, layer: int, subtype: str, occ: int):
        nonlocal seq, last_feedrate_mm_min, processed_rows, last_pose_map
        feed_mm_min = gc.feedrate if (gc.feedrate is not None and gc.feedrate > 0) else last_feedrate_mm_min
        if feed_mm_min is None or feed_mm_min <= 0:
            target_velocity = default_feed_mm_s
        else:
            target_velocity = feed_mm_min / 60.0
        if gc.feedrate is not None and gc.feedrate > 0:
            last_feedrate_mm_min = gc.feedrate
        has_any = False
        move_lines: List[int] = [m.line for m in gc.original_moves] if gc.original_moves else [gc.line]
        if len(move_lines) > 1:
            src_lines = f"{move_lines[0]}-{move_lines[-1]}"
        else:
            src_lines = str(move_lines[0])

        for pt in sample_global_curve_iter(gc, dt=dt, target_velocity=target_velocity):
            has_any = True
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
            _writer_for(layer, subtype, occ).add(row)
            processed_rows += 1
            _maybe_yield()
            seq += 1
            last_pose_map[(layer, subtype)] = row
        if not has_any:
            return

    current_occ: Optional[int] = None

    def flush_moves():
        nonlocal buffer, current_type, current_layer, current_subtype, current_occ
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
            gc = planner.fit_global_curve(
                buffer,
                corner_angle_deg=corner_angle_deg,
                corner_retreat_ratio=corner_retreat_ratio,
                density=density,
                degree=degree,
            )
            if gc is None:
                raise ValueError("B 样条拟合失败，无法导出 npz（段类型: %s, 段长度: %d)" % (current_type, len(buffer)))
            gc_list = [gc]

        layer = buffer[0].layer if buffer else 0
        subtype = buffer[0].subtype if buffer else "UNKNOWN"
        occ = current_occ if current_occ is not None else _ensure_segment(layer, subtype)
        for gc in gc_list:
            _append_sample(gc, layer, subtype, occ)
        buffer = []
        current_type = None
        current_layer = None
        current_subtype = None
        current_occ = None

    def _emit_event(ev: _PendingEvent, layer: int, subtype: str, occ: int):
        nonlocal seq, processed_rows, last_pose_map
        hold_row = last_pose_map.get((layer, subtype)) or CsvRow(
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
        _writer_for(layer, subtype, occ).add(
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
        processed_rows += 1
        _maybe_yield()
        seq += 1
        last_pose_map[(layer, subtype)] = hold_row

    for cmd in parsed_commands:
        # 事件收集：遇到事件前先冲掉当前轨迹段，保证事件贴在后续采样点
        if isinstance(cmd, (ToolChangeCommand, MCommand, ResetECommand)):
            flush_moves()
            if isinstance(cmd, ToolChangeCommand):
                mapped_tool = _map_gcode_tool(cmd.tool)
                current_tool = mapped_tool
                occ = occ_counters.get((cmd.layer, cmd.subtype), 0)
                if occ == 0:
                    occ = _ensure_segment(cmd.layer, cmd.subtype)
                _emit_event(_PendingEvent(
                    event_type="tool_change_cf" if mapped_tool == 1 else "tool_change_resin",
                    payload=str(mapped_tool),
                    src_line=cmd.line,
                    tool_id=mapped_tool,
                ), cmd.layer, cmd.subtype, occ)
            elif isinstance(cmd, ResetECommand):
                occ = occ_counters.get((cmd.layer, cmd.subtype), 0)
                if occ == 0:
                    occ = _ensure_segment(cmd.layer, cmd.subtype)
                _emit_event(_PendingEvent(
                    event_type="extrude_reset",
                    payload=str(cmd.val),
                    src_line=cmd.line,
                    tool_id=current_tool,
                ), cmd.layer, cmd.subtype, occ)
            else:
                ev = _mcommand_to_event(cmd, current_tool)
                if ev:
                    occ = occ_counters.get((cmd.layer, cmd.subtype), 0)
                    if occ == 0:
                        occ = _ensure_segment(cmd.layer, cmd.subtype)
                    _emit_event(ev, cmd.layer, cmd.subtype, occ)
            continue

        # 轨迹分段收集
        if isinstance(cmd, MoveCommand):
            if cmd.is_pure_state_change:
                continue
            if current_type is None:
                current_type = cmd.type
                current_layer = cmd.layer
                current_subtype = cmd.subtype
                current_occ = _ensure_segment(cmd.layer, cmd.subtype)
            if (cmd.type != current_type) or (cmd.layer != current_layer) or (cmd.subtype != current_subtype):
                flush_moves()
                current_type = cmd.type
                current_layer = cmd.layer
                current_subtype = cmd.subtype
                current_occ = _ensure_segment(cmd.layer, cmd.subtype)
            buffer.append(cmd)
            continue

        if isinstance(cmd, GlobalCurveCommand):
            # 如果上游已提供曲线，先冲掉当前 Move 段，再直接采样
            flush_moves()
            layer = getattr(cmd, "layer", 0)
            subtype = getattr(cmd, "subtype", "UNKNOWN")
            occ = occ_counters.get((layer, subtype), 0)
            if occ == 0:
                occ = _ensure_segment(layer, subtype)
            _append_sample(cmd, layer, subtype, occ)
            continue

    # 文件末尾冲掉残余 Move 段
    flush_moves()

    for w in writers.values():
        w.finalize()

    if split_by_layer_type and manifest:
        for key, entry in manifest_by_key.items():
            writer = writers.get(key)
            if writer and writer.last_seq is not None:
                entry["end_seq"] = writer.last_seq
        manifest_path = os.path.join(base_root, f"{base_name}_manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, ensure_ascii=False, indent=2)

        if plot_layer_xy:
            _plot_layers_from_manifest(
                manifest,
                base_root,
                stride=max(1, int(plot_stride)),
            )


def _plot_layers_from_manifest(manifest, base_root: str, stride: int = 5) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return

    import numpy as np
    from pathlib import Path

    def _resolve_npz_files(base_path: Path):
        if base_path.suffix != ".npz":
            base_path = base_path.with_suffix(".npz")
        if base_path.exists():
            return [base_path]
        prefix = base_path.stem + "_part"
        return sorted([p for p in base_path.parent.glob("*.npz") if p.stem.startswith(prefix)])

    by_layer = {}
    for it in manifest:
        layer = int(it.get("layer", 0))
        by_layer.setdefault(layer, []).append(it)

        for layer, segs in sorted(by_layer.items()):
            xs_all = []
            ys_all = []
            first_seg = True
            for seg in segs:
                base = Path(seg["base_path"]).expanduser().resolve()
                files = _resolve_npz_files(base)
                if not files:
                    continue
                for f in files:
                    z = np.load(str(f))
                    if "x" not in z or "y" not in z or "e" not in z or "move_type" not in z:
                        continue
                    x = z["x"]
                    y = z["y"]
                    e = z["e"]
                    mt = z["move_type"]
                    is_print = (mt == 1) | (mt == 3)
                    de = np.diff(e, prepend=e[0])
                    is_deposit = is_print & (de > 1e-6)
                    x = np.where(is_deposit, x, np.nan)
                    y = np.where(is_deposit, y, np.nan)
                    if stride > 1:
                        x = x[::stride]
                        y = y[::stride]
                    if not first_seg:
                        xs_all.append(np.array([np.nan], dtype=np.float32))
                        ys_all.append(np.array([np.nan], dtype=np.float32))
                    first_seg = False
                    xs_all.append(x)
                    ys_all.append(y)
        if not xs_all:
            continue
        x = np.concatenate(xs_all)
        y = np.concatenate(ys_all)
        fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
        ax.plot(x, y, linewidth=0.6, color="#2b2b2b")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title(f"Layer {layer:04d} XY Path")
        ax.grid(True, linewidth=0.3, alpha=0.5)
        out_dir = Path(base_root) / f"layer_{layer:04d}"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"layer_{layer:04d}.png"
        fig.savefig(str(out_path), bbox_inches="tight")
        plt.close(fig)


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
        return _PendingEvent(ev_type, "1", src_line, tool_id)

    if code == "M107":  # 关风扇
        ev_type = "fan_cf" if tool_id == 1 else "fan_resin"
        return _PendingEvent(ev_type, "0", src_line, tool_id)

    # 其他 M 指令：忽略（热床等）
    return None
