#!/usr/bin/env python3
"""
生成笛卡尔多点轨迹：
1) 使用 gcode_planner.GlobalSplinePlanner 进行全局 B 样条平滑（支持任意多路点）；
2) 按 7 阶 S 曲线时间律，以 4ms 周期离散采样；
3) 输出 CSV，并生成 XY 平面的轨迹示意图（忽略姿态，无 UDP 通讯）。
"""

import csv
import math
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
PKG_ROOT = ROOT / "kuka_ram_ws" / "src" / "my_project" / "gcode_planner"
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from gcode_planner.bspline.BaseFunction import BaseFunction
from gcode_planner.bspline_approximation import GlobalSplinePlanner
from gcode_planner.types import MoveCommand, Position

# --- 基础常量 ---

CYCLE_TIME_SEC = 0.004  # 4ms 周期
OUTPUT_CSV = "cartesian_trajectory_bspline.csv"
PLOT_OUTPUT = "trajectory_preview.png"
DECIMAL_PRECISION = 6

# 样条平滑/采样设置（参考 test_z_curve）
SPLINE_DEGREE = 3
SPLINE_DENSITY = 3
CORNER_ANGLE_DEG = 10.0
CORNER_RETREAT_RATIO = 0.2
ARC_LENGTH_SAMPLES = 2000

# 全局可调参数（在 main 中统一设置）
WAYPOINT_POSES: List[List[float]] = []
TARGET_VELOCITY: float = 60.0
ACC_DURATION: float = 1.5
DEC_DURATION: float = 1.5


# --- 7 阶时间律工具 ---

def calc_sept_poly_val(tau: float) -> float:
    """7 阶多项式在归一化时间 tau∈[0,1] 的位置值。"""
    return (35.0 * tau**4) - (84.0 * tau**5) + (70.0 * tau**6) - (20.0 * tau**7)


def three_stage_sept_poly(t: float, total: float, t_acc: float, t_dec: float) -> float:
    """
    三段式 7 阶 S 曲线：加速-匀速-减速，输出归一化路程 s(t)∈[0,1]。
    若无匀速段，则退化为三角形速度曲线。
    """
    if t <= 0:
        return 0.0
    if t >= total:
        return 1.0

    t_flat = total - t_acc - t_dec
    if t_flat < 0:
        return t / total

    k = 2.1875  # tau=0.5 处一阶导峰值
    denom = t_flat + (t_acc + t_dec) / k
    if denom == 0:
        return 0.0
    v_flat = 1.0 / denom

    if t < t_acc:
        tau = 0.5 * (t / t_acc)
        return (2.0 * v_flat * t_acc / k) * calc_sept_poly_val(tau)
    elif t < (total - t_dec):
        s_acc = v_flat * t_acc / k
        return s_acc + v_flat * (t - t_acc)
    else:
        t_rem = total - t
        tau = 0.5 * (t_rem / t_dec)
        s_rem = (2.0 * v_flat * t_dec / k) * calc_sept_poly_val(tau)
        return 1.0 - s_rem


# --- B 样条工具 ---

def build_uniform_knot(num_ctrl: int, degree: int) -> List[float]:
    """生成用于平滑/评估的简单均匀节点向量。"""
    knot = [0.0] * (degree + 1)
    if num_ctrl > degree + 1:
        num_internal = num_ctrl - (degree + 1)
        step = 1.0 / (num_internal + 1)
        for i in range(num_internal):
            knot.append(step * (i + 1))
    knot.extend([1.0] * (degree + 1))
    return knot


def evaluate_bspline_points(ctrl_points: List[Position], degree: int, u_values: np.ndarray) -> np.ndarray:
    """
    使用 BaseFunction 评估 B 样条，返回 XYZ+ABC（六维）。
    返回 shape=(len(u_values), 6) 的数组。
    """
    if len(ctrl_points) < degree + 1:
        raise ValueError("控制点数量不足以构成 B 样条。")

    u_arr = np.asarray(u_values, dtype=float).reshape(-1)
    num_ctrl = len(ctrl_points)
    knot = build_uniform_knot(num_ctrl, degree)

    nik = np.zeros((u_arr.shape[0], num_ctrl))
    for i, u in enumerate(u_arr):
        u_clamped = float(min(max(u, 0.0), 1.0))
        for j in range(num_ctrl):
            nik[i, j] = BaseFunction(j, degree + 1, u_clamped, knot)
    if nik.shape[0] > 0:
        nik[-1, -1] = 1.0  # 确保末端严格落在最后一个控制点

    ctrl_matrix = np.array([[p.x, p.y, p.z, p.a, p.b, p.c] for p in ctrl_points], dtype=float)
    return nik @ ctrl_matrix


def compute_arc_length_profile(curve_pts: np.ndarray) -> Tuple[np.ndarray, float]:
    """根据样条评估点计算归一化弧长分布，返回 (s_samples, total_length)。"""
    if curve_pts.shape[0] < 2:
        raise RuntimeError("样条评估点不足，无法计算弧长。")
    diffs = np.diff(curve_pts[:, :3], axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total_len = float(cum_len[-1])
    if total_len <= 0:
        raise RuntimeError("样条弧长为 0，请检查输入路点。")
    return cum_len / total_len, total_len


def waypoints_to_moves(waypoints: Sequence[Sequence[float]], feedrate_mm_per_min: float) -> List[MoveCommand]:
    """把任意数量的路点转换为连续的 MoveCommand 序列供 GlobalSplinePlanner 使用，缺省姿态补零。"""
    poses = np.array(waypoints, dtype=float)
    if poses.shape[0] < 2 or poses.shape[1] < 3:
        raise ValueError("至少需要 2 个路点，且每个包含 XYZ。")

    # 补齐姿态到 6 维
    if poses.shape[1] < 6:
        zeros = np.zeros((poses.shape[0], 6 - poses.shape[1]))
        poses = np.hstack([poses, zeros])

    moves: List[MoveCommand] = []
    for i in range(poses.shape[0] - 1):
        start = Position(*poses[i, :6])
        end = Position(*poses[i + 1, :6])
        moves.append(
            MoveCommand(
                type="PRINT",
                cmd="G1",
                start_pos=start,
                pos=end,
                e_val=0.0,
                delta_e=0.0,
                feedrate=feedrate_mm_per_min,
                line=i + 1,
            )
        )
    return moves


def compute_motion_time(
    path_length: float, target_velocity: float, t_acc: float, t_dec: float
) -> Tuple[float, float]:
    """根据路径长度、目标速度、加减速时间计算总时长和匀速段时长。"""
    if path_length <= 0 or target_velocity <= 0:
        return 0.0, 0.0

    k = 2.1875
    effective_acc_dec = (t_acc + t_dec) / k
    nominal_time = path_length / target_velocity
    t_flat = nominal_time - effective_acc_dec
    if t_flat < 0:
        t_flat = 0.0
    total_time = t_acc + t_flat + t_dec
    return total_time, t_flat


def generate_trajectory() -> Tuple[List[List[float]], np.ndarray, float]:
    """
    生成整条轨迹（XYZ+ABC）的点序列，返回 (list 版轨迹, numpy 版轨迹, 路径总长)。
    """
    moves = waypoints_to_moves(WAYPOINT_POSES, TARGET_VELOCITY * 60.0)  # feedrate: mm/min
    planner = GlobalSplinePlanner()
    curve_cmd = planner.fit_global_curve(
        moves,
        corner_angle_deg=CORNER_ANGLE_DEG,
        corner_retreat_ratio=CORNER_RETREAT_RATIO,
        density=SPLINE_DENSITY,
        degree=SPLINE_DEGREE,
    )
    if curve_cmd is None:
        raise RuntimeError("B 样条平滑拟合失败，请检查路点。")

    ctrl_points = [curve_cmd.start_pos] + curve_cmd.control_points
    u_samples = np.linspace(0.0, 1.0, ARC_LENGTH_SAMPLES)
    curve_samples = evaluate_bspline_points(ctrl_points, SPLINE_DEGREE, u_samples)
    s_samples, path_length = compute_arc_length_profile(curve_samples)
    total_time, t_flat = compute_motion_time(path_length, TARGET_VELOCITY, ACC_DURATION, DEC_DURATION)
    if total_time <= 0:
        raise RuntimeError("Computed total motion time is zero; check inputs.")

    num_steps = int(math.ceil(total_time / CYCLE_TIME_SEC))
    corrected_total_time = num_steps * CYCLE_TIME_SEC
    time_samples = np.arange(0.0, corrected_total_time + 1e-9, CYCLE_TIME_SEC)
    s_profile = np.array([three_stage_sept_poly(t, corrected_total_time, ACC_DURATION, DEC_DURATION) for t in time_samples])
    u_profile = np.clip(np.interp(s_profile, s_samples, u_samples), 0.0, 1.0)

    eval_points = evaluate_bspline_points(ctrl_points, SPLINE_DEGREE, u_profile)
    trajectory_list = [[float(v) for v in row] for row in eval_points]

    print(f"路点数: {len(WAYPOINT_POSES)} -> 样条控制点 {len(ctrl_points)} (阶次 {SPLINE_DEGREE})")
    print(f"样条路径长度: {path_length:.3f} mm")
    print(f"总时长: {corrected_total_time:.3f}s（匀速段 {t_flat:.3f}s）")
    print(f"平滑设置: density={SPLINE_DENSITY}, corner_angle={CORNER_ANGLE_DEG}°, retreat_ratio={CORNER_RETREAT_RATIO}")

    return trajectory_list, eval_points, path_length


# --- 可视化 ---

def plot_trajectory(curve_points: np.ndarray, waypoints: np.ndarray, filename: str) -> None:
    """生成仅 XY 平面的轨迹预览图。"""
    if curve_points.size == 0 or waypoints.size == 0:
        return

    plt.figure(figsize=(6, 5))
    plt.plot(waypoints[:, 0], waypoints[:, 1], "o--", label="Waypoints", alpha=0.6)
    plt.plot(curve_points[:, 0], curve_points[:, 1], "-", label="B-spline", linewidth=2)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.title("B-spline Trajectory (XY Plane)")
    plt.tight_layout()
    plt.savefig(filename, dpi=160)
    plt.close()
    print(f"轨迹预览图已保存: {filename}")


# --- CSV 工具 ---

def save_to_csv(trajectory: Sequence[Sequence[float]], filename: str) -> None:
    """写入 CSV，格式化到固定小数位。"""
    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for row in trajectory:
            writer.writerow([f"{v:.{DECIMAL_PRECISION}f}" for v in row])


def main():
    """
    集中配置所有参数，便于测试：
      - 路径点、加减速时间、匀速速度、输出文件名
      - 平滑相关参数在文件顶部（SPLINE_*）统一配置
    """
    global WAYPOINT_POSES, ACC_DURATION, DEC_DURATION, TARGET_VELOCITY, OUTPUT_CSV

    # 1) 路径点（可修改）：至少包含 XYZ，可附带 ABC
    WAYPOINT_POSES = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 4.0, 0.0, 0.0, 0.0, 0.0],
        [100.0, 4.0, 0.0, 0.0, 0.0, 0.0],
    ]

    # 2) 速度与时间参数（可修改）
    TARGET_VELOCITY = 10.0  # 匀速段线速度 mm/s
    ACC_DURATION = 3      # 加速时间 s
    DEC_DURATION = 3      # 减速时间 s
    OUTPUT_CSV = "cartesian_trajectory_bspline.csv"

    # 3) 生成轨迹并写入 CSV
    try:
        trajectory, trajectory_np, path_length = generate_trajectory()
    except Exception as exc:
        print(f"生成轨迹失败: {exc}")
        sys.exit(1)

    plot_trajectory(trajectory_np, np.array(WAYPOINT_POSES, dtype=float), PLOT_OUTPUT)
    save_to_csv(trajectory, OUTPUT_CSV)
    print(f"已写入 {len(trajectory)} 个点到 {OUTPUT_CSV}（4ms 采样）。")


if __name__ == "__main__":
    main()
