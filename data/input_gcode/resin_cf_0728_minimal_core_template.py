"""最小 Core/NPZ 注入等价性测试源。

覆盖：两条纤维路径、两次外部 CUT、CUT 后回缩/预挤出、一次纤维到树脂换头。
这个文件返回 Core exporter 使用的 ParsedCommandList，不依赖完整业务 G-code。
"""

from path_processing_core.types import (
    ExtrudeWait,
    GlobalCurveCommand,
    MCommand,
    MoveCommand,
    Position,
    ToolChangeCommand,
)


def _p(x, y, z):
    return Position(x, y, z, 0.0, 0.0, 0.0)


def _travel(line, start, end):
    return MoveCommand(
        type="TRAVEL",
        cmd="G0",
        start_pos=start,
        pos=end,
        e_val=0.0,
        delta_e=0.0,
        feedrate=1200.0,
        line=line,
        layer=0,
        subtype="TRAVEL",
        raw=f"minimal_travel_{line}",
    )


def _fiber(line, start, end, delta_e):
    return GlobalCurveCommand(
        type="PRINT",
        cmd="POLYLINE",
        start_pos=start,
        control_points=[end],
        e_val=delta_e,
        delta_e=delta_e,
        feedrate=1200.0,
        line=line,
        raw=f"minimal_fiber_path_{line}",
    )


def commands():
    origin = _p(0.0, 0.0, 0.5)
    fiber_start = _p(5.0, 5.0, 0.5)
    fiber_end_1 = _p(30.0, 0.0, 0.5)
    fiber_end_2 = _p(30.0, 30.0, 0.5)
    resin_end = _p(0.0, 30.0, 0.5)
    return [
        _travel(1, origin, fiber_start),
        ToolChangeCommand(type="TOOL_CHANGE", tool=0, line=2, subtype="TRAVEL"),
        _fiber(3, fiber_start, fiber_end_1, 12.0),
        MCommand(
            type="M_COMMAND", code="CUT", params={"P": 1.0}, line=4,
            subtype="FIBER", raw="external_npz_cut", tool=1,
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT", wait_sec=0.4, delta_e=-3.0,
            feedrate=300.0, line=5, subtype="FIBER",
            raw="external_npz_retract",
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT", wait_sec=0.4, delta_e=3.0,
            feedrate=300.0, line=6, subtype="FIBER",
            raw="external_npz_prime",
        ),
        _travel(7, fiber_end_1, fiber_end_2),
        _fiber(8, fiber_end_1, fiber_end_2, 9.0),
        MCommand(
            type="M_COMMAND", code="CUT", params={"P": 1.0}, line=9,
            subtype="FIBER", raw="external_npz_cut", tool=1,
        ),
        _travel(10, fiber_end_2, resin_end),
        ToolChangeCommand(type="TOOL_CHANGE", tool=1, line=11, subtype="TRAVEL"),
        _fiber(12, fiber_end_2, resin_end, 6.0),
    ]


if __name__ == "__main__":
    print(f"minimal commands: {len(commands())}")
