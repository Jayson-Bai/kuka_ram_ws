import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# 设置路径：指向 gcode_planner 包所在目录
ROOT = Path(__file__).resolve().parent
PKG_ROOT = ROOT / 'kuka_ram_ws' / 'src' / 'my_project' / 'gcode_planner'
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from gcode_planner.types import MoveCommand, Position
from gcode_planner.bspline_approximation import GlobalSplinePlanner
from gcode_planner.bspline import bspline_curve as bc

def create_move(start, end):
    return MoveCommand(
        type="PRINT",
        cmd="G1",
        start_pos=start,
        pos=end,
        e_val=0,
        delta_e=0,
        feedrate=1000,
        line=0
    )


def _build_knot(num_ctrl: int, degree: int):
    """生成用于绘制的简单均匀节点向量。"""
    knot = [0.0] * (degree + 1)
    if num_ctrl > degree + 1:
        num_internal = num_ctrl - (degree + 1)
        step = 1.0 / (num_internal + 1)
        for i in range(num_internal):
            knot.append(step * (i + 1))
    knot.extend([1.0] * (degree + 1))
    return knot


def _plot_cmd(cmd, u_eval, degree, style, label):
    H = len(cmd.control_points) + 1
    ctrl_pts = [cmd.start_pos] + cmd.control_points
    P = [
        [p.x for p in ctrl_pts],
        [p.y for p in ctrl_pts],
        [p.z for p in ctrl_pts],
    ]

    knot = _build_knot(H, degree)
    curve_points = bc.curve(P, H, degree, u_eval, knot)
    plt.plot(curve_points[0], curve_points[1], style, linewidth=2, label=f"{label} (H={H})")
    print(f"{label}: control points H={H}")


def main():
    # 构造 Z 字型轨迹
    p1 = Position(0, 0, 0, 0, 0, 0)
    p2 = Position(10, 0, 0, 0, 0, 0)
    p3 = Position(0, 1.5, 0, 0, 0, 0)
    p4 = Position(10, 1.5, 0, 0, 0, 0)

    moves = [
        create_move(p1, p2),
        create_move(p2, p3),
        create_move(p3, p4)
    ]

    planner = GlobalSplinePlanner()
    degree = 3
    u_eval = np.linspace(0, 1, 200).tolist()
    
    plt.figure(figsize=(10, 8))
    
    # 绘制原始路径
    orig_x = [p1.x, p2.x, p3.x, p4.x]
    orig_y = [p1.y, p2.y, p3.y, p4.y]
    plt.plot(orig_x, orig_y, 'r--', label='Original Path', marker='o', alpha=0.5)

    # 通过 density 调整数据点加密，控制点数量随数据量按 1/5 自动增长
    densities = [3, 4, 5]
    styles = ['b-', 'g-.', 'm:']

    for idx, density in enumerate(densities):
        style = styles[idx % len(styles)]
        print(f"\n--- Generating density={density} ---")
        cmd = planner.fit_global_curve(moves, degree=degree, density=density)
        if cmd is None:
            print(f"density={density} failed to generate curve")
            continue
        _plot_cmd(cmd, u_eval, degree, style, label=f"density={density}")

    plt.title('Z-Curve Approximation (density control: H ≈ N/5)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    output_path = ROOT / 'z_curve_density_test.png'
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")

if __name__ == "__main__":
    main()
