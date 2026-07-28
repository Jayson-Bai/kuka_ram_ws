"""最小树脂/纤维 Core 输入模板。

这个模板按当前外部 NPZ 的命令约定缩小为：树脂路径、纤维路径、CUT、
CUT 后回缩/预挤出、纤维间移动，以及最后换回树脂。用于比较 Core 全额
导出和本地 NPZ 注入的事件与边界逻辑。

调用 :func:`export_npz` 时必须设置 ``external_npz_cut_absolute_e=True``，
这样 CUT 的 E 复位和抬升/等待序列才与上位机导出的 NPZ 一致。
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


def _travel(line, start, end, *, raw=None):
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
        raw=raw or f"minimal_travel_{line}",
    )


def _path(line, start, end, delta_e, material):
    return GlobalCurveCommand(
        type="PRINT",
        cmd="POLYLINE",
        start_pos=start,
        control_points=[end],
        e_val=delta_e,
        delta_e=delta_e,
        feedrate=1200.0,
        line=line,
        raw=f"minimal_{material.lower()}_path_{line}",
    )


def _cut(line):
    return MCommand(
        type="M_COMMAND",
        code="CUT",
        params={"P": 1.0},
        line=line,
        layer=0,
        subtype="FIBER",
        raw="external_npz_cut",
        tool=1,
    )


def commands():
    """Return a minimal mixed-material ParsedCommandList."""

    a = _p(0.0, 0.0, 0.5)
    b = _p(20.0, 0.0, 0.5)
    c = _p(20.0, 20.0, 0.5)
    d = _p(0.0, 20.0, 0.5)
    e = _p(0.0, 0.0, 0.5)
    return [
        # This raw marker is the external NPZ convention for the first XY
        # travel. It is where Core applies the resin-Z print compensation.
        _travel(1, a, a, raw="external_npz_start_xy_travel"),
        _path(2, a, b, 6.0, "RESIN"),
        ToolChangeCommand(type="TOOL_CHANGE", tool=0, line=3, subtype="TRAVEL"),
        _path(4, b, c, 12.0, "FIBER"),
        _cut(5),
        ExtrudeWait(
            type="EXTRUDE_WAIT", wait_sec=0.4, delta_e=-3.0,
            feedrate=300.0, line=6, subtype="FIBER",
            raw="external_npz_retract",
        ),
        ExtrudeWait(
            type="EXTRUDE_WAIT", wait_sec=0.4, delta_e=3.0,
            feedrate=300.0, line=7, subtype="FIBER",
            raw="external_npz_prime",
        ),
        _travel(8, c, d),
        _path(9, c, d, 9.0, "FIBER"),
        _cut(10),
        _travel(11, d, e),
        ToolChangeCommand(type="TOOL_CHANGE", tool=1, line=12, subtype="TRAVEL"),
        _path(13, e, a, 6.0, "RESIN"),
    ]


if __name__ == "__main__":
    print(f"minimal mixed-material commands: {len(commands())}")
