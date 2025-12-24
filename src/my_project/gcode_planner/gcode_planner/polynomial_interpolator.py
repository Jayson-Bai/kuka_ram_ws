"""
时间参数化器（七阶 S 曲线 + 球面插值）：
- 几何由 bspline_approximation 生成的 GlobalCurveCommand 提供，只负责沿弧长采样。
- 目标匀速暂写死为 10 mm/s，入口/出口速度、加速度、jerk 全 0，对称加速/减速时间固定 2 s。
- 若弧长太短无法形成匀速段，则自动降低峰值速度（等价于把位移缩放进同样的时间，保证不超速且 jerk 连续）。
- 挤出量 E 按 4 ms 采样的路程比例分配，保持绝对挤出。
"""

from dataclasses import dataclass
from typing import List, Optional
import math
import bisect

from .types import Position, GlobalCurveCommand


# -------------------------- 基础工具 --------------------------

@dataclass
class InterpolatedPoint:
    t: float
    pos: Position
    e: float               # 绝对挤出量
    extrude_speed: float   # dE/dt (mm/s) 供调试
    feedrate_mm_min: float
    cmd_type: str
    line: Optional[int]
    raw: Optional[str]


def _euler_xyz_to_quat(roll: float, pitch: float, yaw: float):
    """欧拉角(弧度) -> 四元数，顺序 XYZ。"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def _quat_to_euler_xyz(q):
    """四元数 -> 欧拉角(弧度)，顺序 XYZ。"""
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _quat_slerp(q0, q1, t: float):
    """球面插值，q0/q1 均为 (w,x,y,z)。"""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    dot = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    if dot < 0.0:
        w1, x1, y1, z1 = -w1, -x1, -y1, -z1
        dot = -dot
    if dot > 0.9995:
        w = w0 + t * (w1 - w0)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        return (w / norm, x / norm, y / norm, z / norm)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    w = s0 * w0 + s1 * w1
    x = s0 * x0 + s1 * x1
    y = s0 * y0 + s1 * y1
    z = s0 * z0 + s1 * z1
    return (w, x, y, z)


# -------------------------- B 样条评估（仅用于已生成的控制点） --------------------------

def _make_open_uniform_knots(n_ctrl: int, degree: int = 3) -> List[float]:
    knots: List[float] = []
    for i in range(n_ctrl + degree + 1):
        if i <= degree:
            knots.append(0.0)
        elif i >= n_ctrl:
            knots.append(n_ctrl - degree)
        else:
            knots.append(i - degree)
    return knots


def _find_span(u: float, knots: List[float], degree: int, n_ctrl: int) -> int:
    if abs(u - knots[n_ctrl]) < 1e-9:
        return n_ctrl - 1
    low = degree
    high = n_ctrl
    mid = (low + high) // 2
    while not (knots[mid] <= u < knots[mid + 1]):
        if u < knots[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def _de_boor(u: float, degree: int, knots: List[float], ctrl: List[Position]) -> Position:
    n_ctrl = len(ctrl)
    span = _find_span(u, knots, degree, n_ctrl)
    d = [Position(**vars(ctrl[span - degree + i])) for i in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = span - degree + j
            denom = knots[i + degree - r + 1] - knots[i]
            alpha = 0.0 if abs(denom) < 1e-12 else (u - knots[i]) / denom
            prev = d[j - 1]
            curr = d[j]
            d[j] = Position(
                x=(1 - alpha) * prev.x + alpha * curr.x,
                y=(1 - alpha) * prev.y + alpha * curr.y,
                z=(1 - alpha) * prev.z + alpha * curr.z,
                a=(1 - alpha) * prev.a + alpha * curr.a,
                b=(1 - alpha) * prev.b + alpha * curr.b,
                c=(1 - alpha) * prev.c + alpha * curr.c,
            )
    return d[degree]


def _build_arc_length_map(ctrl: List[Position], degree: int = 3, samples: int = 400):
    knots = _make_open_uniform_knots(len(ctrl), degree)
    u_min = knots[degree]
    u_max = knots[len(ctrl)]

    u_list: List[float] = []
    len_list: List[float] = []

    prev_pos = _de_boor(u_min, degree, knots, ctrl)
    u_list.append(u_min)
    len_list.append(0.0)

    current_len = 0.0
    for i in range(1, samples + 1):
        u = u_min + (u_max - u_min) * i / samples
        curr_pos = _de_boor(u, degree, knots, ctrl)
        dist = math.sqrt(
            (curr_pos.x - prev_pos.x) ** 2
            + (curr_pos.y - prev_pos.y) ** 2
            + (curr_pos.z - prev_pos.z) ** 2
        )
        current_len += dist
        u_list.append(u)
        len_list.append(current_len)
        prev_pos = curr_pos

    total_length = len_list[-1]
    return u_list, len_list, total_length, knots


def _lookup_u_from_s(s_norm: float, u_list: List[float], len_list: List[float], total_length: float) -> float:
    """给定归一化弧长 s_norm (0~1)，返回对应的 B 样条参数 u。"""
    target_len = s_norm * total_length
    if target_len <= 1e-9:
        return u_list[0]
    if target_len >= total_length - 1e-9:
        return u_list[-1]

    idx = bisect.bisect_right(len_list, target_len)
    if idx == 0:
        return u_list[0]
    if idx >= len(len_list):
        return u_list[-1]

    l0 = len_list[idx - 1]
    l1 = len_list[idx]
    u0 = u_list[idx - 1]
    u1 = u_list[idx]
    if abs(l1 - l0) < 1e-12:
        return u0
    ratio = (target_len - l0) / (l1 - l0)
    return u0 + ratio * (u1 - u0)


# -------------------------- 七阶 S 曲线剖面 --------------------------

def _sept_poly_base(tau: float) -> float:
    """归一化 0->1 七阶位置曲线，v/a/jerk 在两端为 0。"""
    return (35.0 * tau**4) - (84.0 * tau**5) + (70.0 * tau**6) - (20.0 * tau**7)


def _three_stage_sept_poly(t: float, total: float, t_acc: float, t_dec: float) -> float:
    """
    三段式七阶 S 曲线：加速-匀速-减速，输出归一化路程 s(t)∈[0,1]。
    若无匀速段，则退化为对称 S 曲线（总时长 t_acc+t_dec）。
    """
    if t <= 0.0:
        return 0.0
    if t >= total:
        return 1.0

    t_flat = total - t_acc - t_dec
    if t_flat < 0:
        # 时间不足形成匀速段，保持对称 S 曲线
        return t / total

    k = 2.1875  # 基函数导数峰值
    denom = t_flat + (t_acc + t_dec) / k
    if denom <= 0:
        return t / total
    v_flat = 1.0 / denom  # 归一化匀速速度

    if t < t_acc:
        tau = 0.5 * (t / t_acc)
        return (2.0 * v_flat * t_acc / k) * _sept_poly_base(tau)
    elif t < (total - t_dec):
        s_acc = v_flat * t_acc / k
        return s_acc + v_flat * (t - t_acc)
    else:
        t_rem = total - t
        tau = 0.5 * (t_rem / t_dec)
        s_rem = (2.0 * v_flat * t_dec / k) * _sept_poly_base(tau)
        return 1.0 - s_rem


def _compute_time_profile(length: float, target_v: float, t_acc: float, t_dec: float):
    """
    根据弧长和目标匀速计算总时间与匀速时间。
    若长度过短，则匀速段为 0，总时间=t_acc+t_dec。
    """
    if length <= 0.0 or target_v <= 0.0:
        return 0.0, 0.0
    k = 2.1875
    nominal_time = length / target_v
    effective_acc_dec = (t_acc + t_dec) / k
    t_flat = nominal_time - effective_acc_dec
    if t_flat < 0:
        t_flat = 0.0
    total_time = t_acc + t_flat + t_dec
    return total_time, t_flat


# -------------------------- 采样主逻辑 --------------------------

def sample_global_curve(
    curve: GlobalCurveCommand,
    dt: float = 0.004,
    target_velocity: float = 10.0,  # mm/s
    t_acc: float = 2.0,
    t_dec: float = 2.0,
) -> List[InterpolatedPoint]:
    """
    对一条全局 B 样条进行时间参数化并采样。
    - 入口/出口 v/a/jerk 均为 0，对称加/减速时间固定。
    - 匀速段无法满足时自动退化为对称 S 曲线（速度整体下降，不超速）。
    - 挤出按弧长比例分配，保持绝对挤出量不变。
    """
    if curve is None:
        return []

    ctrl = [curve.start_pos] + curve.control_points
    degree = 3

    # 构建弧长映射
    u_list, len_list, total_length, knots = _build_arc_length_map(ctrl, degree=degree, samples=max(400, len(ctrl) * 10))
    if total_length <= 1e-9:
        # 退化：零长度，直接返回终点
        return [
            InterpolatedPoint(
                t=0.0,
                pos=curve.start_pos,
                e=curve.e_val,
                extrude_speed=0.0,
                feedrate_mm_min=curve.feedrate,
                cmd_type=curve.type,
                line=curve.line,
                raw=curve.raw,
            )
        ]

    # 时间规划
    total_time, t_flat = _compute_time_profile(total_length, target_velocity, t_acc, t_dec)
    if total_time <= 0.0:
        return []

    num_steps = int(math.ceil(total_time / dt))
    corrected_total_time = num_steps * dt
    time_samples = [i * dt for i in range(num_steps + 1)]
    s_profile = [_three_stage_sept_poly(t, corrected_total_time, t_acc, t_dec) for t in time_samples]

    start_e = curve.e_val - curve.delta_e
    current_e = start_e
    samples: List[InterpolatedPoint] = []

    # 姿态：仅用起点/终点做 slerp
    end_pos = ctrl[-1]
    start_q = _euler_xyz_to_quat(
        math.radians(curve.start_pos.a),
        math.radians(curve.start_pos.b),
        math.radians(curve.start_pos.c),
    )
    end_q = _euler_xyz_to_quat(
        math.radians(end_pos.a),
        math.radians(end_pos.b),
        math.radians(end_pos.c),
    )

    prev_s = 0.0
    for t, s_norm in zip(time_samples, s_profile):
        s_norm_clamped = max(0.0, min(1.0, s_norm))
        if t == time_samples[-1]:
            s_norm_clamped = 1.0  # 确保最后一点落在终点

        u = _lookup_u_from_s(s_norm_clamped, u_list, len_list, total_length)
        p = _de_boor(u, degree, knots, ctrl)

        # 姿态插值
        qs = _quat_slerp(start_q, end_q, s_norm_clamped)
        a_rad, b_rad, c_rad = _quat_to_euler_xyz(qs)
        p.a = math.degrees(a_rad)
        p.b = math.degrees(b_rad)
        p.c = math.degrees(c_rad)

        # 挤出分配（按弧长比例）
        curr_s = s_norm_clamped * total_length
        delta_s = curr_s - prev_s
        delta_e = curve.delta_e * (delta_s / total_length)
        current_e += delta_e
        prev_s = curr_s

        # 速度估计：用前一帧差分
        feed_mm_s = delta_s / dt if dt > 0 else 0.0
        feed_mm_min = feed_mm_s * 60.0
        extrude_speed = delta_e / dt if dt > 0 else 0.0

        samples.append(
            InterpolatedPoint(
                t=t,
                pos=p,
                e=current_e,
                extrude_speed=extrude_speed,
                feedrate_mm_min=feed_mm_min,
                cmd_type=curve.type,
                line=curve.line,
                raw=curve.raw,
            )
        )

    return samples


__all__ = [
    "InterpolatedPoint",
    "sample_global_curve",
]
