#!/usr/bin/env python3
import socket
import sys
import re
import time
import math
import csv
import gc
import numpy as np
try:
    from scipy.spatial.transform import Rotation as R, Slerp
except ImportError:
    print(" 缺少 'scipy' 库。")
    sys.exit(1)


# 1. RSI 通信配置

LOCAL_IP = "192.168.1.1" 
LOCAL_PORT = 49152
KUKA_ADDRESS = (LOCAL_IP, LOCAL_PORT)
BUFFER_SIZE = 4096

# KUKA RSI 周期 (4ms)
RSI_CYCLE_TIME_SEC = 0.004 


# 2. 轨迹和 CSV 配置


TRAJECTORY_CSV_FILE = "cartesian_trajectory.csv"

# KUKA XML <RECEIVE> 中定义的标签名 
XML_POSE_TAG = "PosCorr"
XML_STOP_TAG = "Stop"


DECIMAL_SEPARATOR = "."
DECIMAL_PRECISION = 6 # 默认精度，可在 main 中修改


# 3. 通信
# ipoc_regex = re.compile(r"<IPOC>(\d+)</IPOC>") # 优化掉正则


# 4. 状态/时间控制全局变量 

start_time = None
packet_count = 0
csv_data = [] # 预读取的 CSV 数据
current_csv_row = [0.0] * 6 # 存储 [X, Y, Z, A, B, C]


# 5. 轨迹插值 (分段七阶多项式)


def calc_sept_poly_val(tau: float) -> float:
    """计算七阶多项式的值 (tau in [0, 1])"""
    return (35.0 * tau**4) - (84.0 * tau**5) + (70.0 * tau**6) - (20.0 * tau**7)

def three_stage_sept_poly(t: float, T_total: float, T_acc: float, T_dec: float) -> float:
    """
    三阶段七阶多项式插值 (加速 -> 匀速 -> 减速)
    加速/减速段使用七阶多项式的形状，中间使用线性插值
    """
    if t <= 0: return 0.0
    if t >= T_total: return 1.0

    T_flat = T_total - T_acc - T_dec
    if T_flat < 0:
        # 如果时间配置错误(加速+减速>总时间)，打印警告并按比例缩放(简单起见此处直接返回线性)
        print(f"[警告] 加减速时间总和 ({T_acc + T_dec}) 超过总时间 ({T_total})")
        return t / T_total

    # 七阶多项式中点 (tau=0.5) 的一阶导数 P'(0.5)
    # v(tau) = 140*tau^3 - 420*tau^4 + 420*tau^5 - 140*tau^6
    # v(0.5) = 2.1875
    K = 2.1875 

    # 计算中间匀速段的速度 (归一化距离 1.0)
    # Dist = Area under V graph = (V * T_acc / K) + (V * T_flat) + (V * T_dec / K) = 1.0
    denom = T_flat + (T_acc + T_dec) / K
    if denom == 0: return 0.0
    V_flat = 1.0 / denom

    # 1. 加速段 (t in [0, T_acc])
    if t < T_acc:
        if T_acc == 0: return 0.0 
        # 映射 t [0, T_acc] 到 tau [0, 0.5]
        tau = 0.5 * (t / T_acc)
        val = calc_sept_poly_val(tau)
        
        # 缩放系数
        # s(T_acc) = V_flat * T_acc / K
        # P(0.5) = 0.5
        # scale * 0.5 = V_flat * T_acc / K  => scale = 2 * V_flat * T_acc / K
        scale = 2.0 * V_flat * T_acc / K
        return scale * val

    # 2. 匀速段 (t in [T_acc, T_total - T_dec])
    elif t < (T_total - T_dec):
        s_acc_end = V_flat * T_acc / K
        dt = t - T_acc
        return s_acc_end + V_flat * dt

    # 3. 减速段 (t in [T_total - T_dec, T_total])
    else:
        if T_dec == 0: return 1.0
        # 对称处理: 计算从终点倒推的距离
        t_rem = T_total - t
        # 映射 t_rem [0, T_dec] 到 tau [0, 0.5]
        tau = 0.5 * (t_rem / T_dec)
        val = calc_sept_poly_val(tau)
        
        scale_dec = 2.0 * V_flat * T_dec / K
        s_rem = scale_dec * val
        return 1.0 - s_rem

def interpolate_pose(start_pose: list, end_pose: list, s_t: float) -> list:
    """
    根据标量 s_t (0到1) 线性插值两个 6-DOF 位姿
    """
    new_pose = [0.0] * 6
    for i in range(6):
        delta_q = end_pose[i] - start_pose[i]
        new_pose[i] = start_pose[i] + delta_q * s_t
    return new_pose

def generate_trajectory(start_pose, end_pose, duration, cycle_time, acc_time, dec_time) -> list:
    """
    在两个位姿之间生成一个完整的三阶段七阶插值轨迹
    """
    print(f"--- 正在生成轨迹 ---")
    print(f"  起点: {start_pose}")
    print(f"  终点: {end_pose}")
    
    # 强制时间对齐到 4ms 周期整数倍 (向上取整 logic: math.ceil)
    # 只要 duration 不是 4ms 的整数倍，就拉长到下一个倍数
    num_steps = int(math.ceil(duration / cycle_time))
    corrected_duration = num_steps * cycle_time
    
    print(f"  时间: {duration}s -> 对齐后(拉长): {corrected_duration:.4f}s (@ {cycle_time*1000}ms, {num_steps}步)")
    print(f"  阶段: 加速={acc_time}s, 匀速={corrected_duration-acc_time-dec_time:.4f}s, 减速={dec_time}s")
    
    trajectory = []
    
    # 1. 拆分位置和姿态
    start_pos = start_pose[0:3]
    end_pos = end_pose[0:3]
    start_orient_abc = start_pose[3:6]
    end_orient_abc = end_pose[3:6]

    # 2. 设置姿态插值 (SLERP)
    # KUKA 机械臂的 A, B, C 对应欧拉角 'zyx'
    try:
        key_rots = R.from_euler('zyx', [start_orient_abc, end_orient_abc], degrees=True)
        key_times = [0.0, 1.0] # 标量时间 (0% -> 100%)
        
        # 修正最短路径逻辑 (需要 Numpy)
        # 1. 将 Rotation 对象提取为 Numpy 数组
        quats = key_rots.as_quat()
        
        # 2. 在 Numpy 数组上执行点积
        if np.dot(quats[0], quats[1]) < 0:
            # 3. 如果需要，反转第二个四元数
            quats[1] = -quats[1]
            # 4. 从修改后的 Numpy 数组重新创建 Rotation 对象
            key_rots = R.from_quat(quats)
            print("  SLERP 侦测到 >180 度旋转，已自动修正为最短路径。")

        # 5. 现在使用修正后的 key_rots 对象创建 SLERP
        slerp_obj = Slerp(key_times, key_rots)
        print("  SLERP 姿态插值已初始化 (Scipy)。")
        
    except Exception as e:
        print(f" 无法初始化 Scipy SLERP: {e}")
        print("  (KUKA 姿态是否接近万向节死锁?)")
        sys.exit(1)


    for i in range(num_steps + 1):
        t = i * cycle_time
        # 1. 计算S型曲线标量 (0.0 -> 1.0)
        s_t = three_stage_sept_poly(t, corrected_duration, acc_time, dec_time)
        
        # 2.  根据标量计算6轴位姿
        
        # 2a. (X, Y, Z) 位置: 使用线性插值 (LERP)
       
        pose_xyz = [0.0] * 3
        for j in range(3):
            delta_q = end_pos[j] - start_pos[j]
            pose_xyz[j] = start_pos[j] + delta_q * s_t

        # 2b. (A, B, C) 姿态: 使用球面线性插值 (SLERP)
        interp_rot_quat = slerp_obj([s_t]) # 传入标量时间 s_t
        
        # 转换回 KUKA 'zyx' 欧拉角 (A, B, C)
        pose_abc = interp_rot_quat.as_euler('zyx', degrees=True)[0]
        
        # 2c. 合并
        pose = pose_xyz + list(pose_abc) # list() 确保合并
        
        trajectory.append(pose)
        
    print(f" 轨迹生成完毕 (共 {len(trajectory)} 个插值点)")
    return trajectory


# 6. CSV 文件操作


def save_to_csv(trajectory: list, filename: str):
    """
    将生成的轨迹保存到 CSV 文件
    确保所有值都格式化为 6 位小数 (高精度)
    """
    print(f"--- 正在保存轨迹到 {filename} ---")
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # 循环遍历轨迹
            for row in trajectory:
                # 将此行中的每个浮点数格式化为指定精度的字符串
                formatted_row = [f"{value:.{DECIMAL_PRECISION}f}" for value in row]
                
                # 写入格式化后的行
                writer.writerow(formatted_row)
                
        print(f" 轨迹已保存 ({DECIMAL_PRECISION}位小数精度)。")
    except Exception as e:
        print(f" 致命错误: 无法写入 CSV 文件: {e}")
        sys.exit(1)


# 7. 工具函数 (精度可配)


def format_float(value: float) -> str:
    """格式化浮点数为字符串，固定点号小数 (使用 DECIMAL_PRECISION)"""
    formatted_str = f"{value:.{DECIMAL_PRECISION}f}"
    return formatted_str if DECIMAL_SEPARATOR == "." else formatted_str.replace(".", ",")

def extract_ipoc(data_str: str) -> str:
    """从KUKA数据包中提取 IPOC (优化版：使用字符串查找切片)"""
    try:
        # 假设 XML 结构相对固定，直接查找标签位置
        start_tag = "<IPOC>"
        end_tag = "</IPOC>"
        
        # find 比正则快得多
        start_idx = data_str.find(start_tag)
        if start_idx == -1:
            return "0"
            
        # 跳过 <IPOC> 长度 (6)
        start_content = start_idx + 6 
        
        end_idx = data_str.find(end_tag, start_content)
        if end_idx == -1:
            return "0"
            
        return data_str[start_content:end_idx]
    except Exception:
        # print(f"[警告] 无法解析 IPOC。")
        return "0"


# 8. 生成 KUKA 回复 (笛卡尔位姿)

def generate_reply(ipoc: str, pose_data: list, stop_signal_bool=False) -> str:
    """
    生成发送给KUKA的XML回复 (笛卡尔绝对位置)
    
    重要: KUKA RSI 必须运行在 #ABSOLUTE 模式!
    """
    stop_value = "1" if stop_signal_bool else "0"

    if len(pose_data) < 6:
        print(f"[警告] CSV 行数据不足6个! (需要6, 得到 {len(pose_data)}). 用 0.0 填充.")
        pose_data.extend([0.0] * (6 - len(pose_data)))

    # 格式化为指定小数位数
    p = {
        "X": format_float(pose_data[0]),
        "Y": format_float(pose_data[1]),
        "Z": format_float(pose_data[2]),
        "A": format_float(pose_data[3]),
        "B": format_float(pose_data[4]),
        "C": format_float(pose_data[5])
    }

    
    reply = (
    '<Sen Type="PythonDemo">\r\n'
    f'<{XML_POSE_TAG} X="{p["X"]}" '
    f'Y="{p["Y"]}" '
    f'Z="{p["Z"]}" '
    f'A="{p["A"]}" '
    f'B="{p["B"]}" '
    f'C="{p["C"]}"/>\r\n'
    f'<{XML_STOP_TAG}>{stop_value}</{XML_STOP_TAG}>\r\n'
    f'<IPOC>{ipoc}</IPOC>\r\n'
    '</Sen>'
    )
    return reply


# 9. 启动 RSI 服务器

def start_rsi_server():
    """启动UDP服务器，监听KUKA RSI通信，并回放轨迹CSV"""
    global start_time, packet_count, csv_data, current_csv_row

    # 1. 打开已生成的 CSV 文件并预读取到内存
    try:
        print(f"--- 启动 RSI 服务器 ---")
        print(f"正在预读取轨迹 CSV 到内存: {TRAJECTORY_CSV_FILE}...")
        
        with open(TRAJECTORY_CSV_FILE, mode='r', encoding='utf-8') as f:
            reader = csv.reader(f)
            # [优化] 预先将字符串数据转换为 float 列表，避免实时循环中的类型转换开销
            raw_rows = list(reader)
            csv_data = []
            for r in raw_rows:
                if not r: continue
                try:
                    # 仅提取前6列并转为 float
                    float_row = [float(x) for x in r[:6]]
                    csv_data.append(float_row)
                except ValueError:
                    continue # 跳过标题行或无效行
            
        if not csv_data:
            print(f" 致命错误: CSV 文件为空或解析失败: {TRAJECTORY_CSV_FILE}")
            sys.exit(1)
            
        print(f"CSV 轨迹文件已预处理加载到内存。共 {len(csv_data)} 点。")
        print(f"第一点: {csv_data[0]}")

    except FileNotFoundError:
        print(f" 致命错误: 找不到 CSV 文件: {TRAJECTORY_CSV_FILE}")
        print("(脚本是否已成功生成该文件?)")
        sys.exit(1)
    except Exception as e:
        print(f" 读取 CSV 文件时出错: {e}")
        sys.exit(1)

    # 2. 绑定 Socket 
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # [优化] 显式增加 UDP 缓冲区 (例如 1MB) 防止丢包
    try:
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
    except Exception as e:
        print(f"[警告] 无法设置 UDP 缓冲区: {e}")

    try:
        udp_socket.bind(KUKA_ADDRESS)
    except socket.error as e:
        print(f" 无法绑定到 {LOCAL_IP}:{LOCAL_PORT} - {e}")
        sys.exit(1)

    print(f"\n RSI 轨迹回放服务器启动: {LOCAL_IP}:{LOCAL_PORT}")
    print(f"   等待 KUKA 控制器连接...")
    print(f"   将发送 '{TRAJECTORY_CSV_FILE}' 中的绝对位姿 ({DECIMAL_PRECISION}位小数)")
    print("按 Ctrl+C 安全停止服务器。")

    remote_address = None
    
    # 标记是否已经提示过轨迹结束，避免重复打印日志
    trajectory_finished_logged = False

    try:
        # [优化] 进入实时循环前禁用 GC，防止不可预测的停顿
        gc.disable()
        # print("GC 已禁用。")

        while True:
            # 3. 阻塞等待 KUKA 的数据包
            data, received_from_address = udp_socket.recvfrom(BUFFER_SIZE)

            if not remote_address:
                remote_address = received_from_address
                start_time = time.time()
                # print(f"\n首次连接来自: {remote_address} (开始 CSV 回放)")
                packet_count = 0

            recv_str = data.decode("utf-8", errors="ignore")
            ipoc = extract_ipoc(recv_str)

            #  5. 核心修改：从内存读取下一行 
            if packet_count < len(csv_data):
                # [优化] 直接获取预处理好的 float 列表，无需转换，速度极快 (引用复制)
                current_csv_row = csv_data[packet_count]
            else:
                # 轨迹结束
                if not trajectory_finished_logged:
                    # print("\n[信息] 轨迹已到达终点 (内存数据末尾)。")
                    # print("   将保持发送最后一个位姿点。")
                    trajectory_finished_logged = True
                # 保持在 current_csv_row 中的最后一个值
                pass 

            # 6. 生成回复 (使用从内存读取的行)
            reply_str = generate_reply(ipoc, current_csv_row, stop_signal_bool=False)
            
            # 7. 发送回复
            udp_socket.sendto(reply_str.encode("ascii"), remote_address)

            # 8. 打印日志 
            # if packet_count % 100 == 0: 
            #     print(f"\n--- (包 #{packet_count}) ---")
            #     print(f"收到 KUKA (IPOC {ipoc}): {recv_str}")
            #     print(f"发送位姿 (来自内存): X-C = {['{:.6f}'.format(f) for f in current_csv_row]}")

            packet_count += 1
            # 显示进度时，如果轨迹结束，显示总行数
            display_row = packet_count if packet_count <= len(csv_data) else len(csv_data)
            # print(f"\r[RSI 已连接] IPOC: {ipoc} 周期#{packet_count} | CSV行: {display_row}/{len(csv_data)}  ", end="")

    except KeyboardInterrupt:
        print("\n 检测到 Ctrl+C, 正在发送停止信号...")
        if remote_address:
            try:
                udp_socket.settimeout(0.5)
                try:
                    # 尝试接收最后一个 IPOC
                    data, _ = udp_socket.recvfrom(BUFFER_SIZE)
                    recv_str = data.decode("utf-8", errors="ignore")
                    ipoc = extract_ipoc(recv_str)
                except socket.timeout:
                    ipoc = "0" # 如果超时, 发送 0
                
                # 发送停止信号 (Stop=True)
                final_reply = generate_reply(ipoc, current_csv_row, stop_signal_bool=True)
                for _ in range(3): # 发送3次以确保 KUKA 收到
                    udp_socket.sendto(final_reply.encode("ascii"), remote_address)
                    time.sleep(0.01)
                print(f" 已发送停止信号 (Stop=1) 至 KUKA {remote_address}")
            except Exception as e:
                print(f"[错误] 发送停止信号时出错: {e}")
    finally:
        # [优化] 恢复 GC
        gc.enable()
        # print("GC 已恢复。")
        
        udp_socket.close()
        print("服务器已关闭。")


# 10. 主入口

if __name__ == "__main__":
    
    
    
    # KUKA 坐标格式 [X, Y, Z, A, B, C] X-C Y-B Z-A
    # (单位: mm 和 度)
    
    START_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    END_POSE   = [300.0,  -300.0, 0.0, 2.0, 2.0, 2.0]
    
    # ---------------------------------------------------------
    # 参数配置
    # ---------------------------------------------------------

    # 目标匀速段速度 (mm/s)
    TARGET_VELOCITY = 20
    
    # 加速和减速时间配置 (秒)
    ACC_DURATION = 3
    DEC_DURATION = 3

    # 发送/接收精度配置 (小数位数)
    DECIMAL_PRECISION = 4

    # ---------------------------------------------------------
    # 自动计算总时间
    # ---------------------------------------------------------
    
    # 1. 计算直线距离 (Euclidean Distance of XYZ)
    dist_vector = np.array(END_POSE[0:3]) - np.array(START_POSE[0:3])
    distance = np.linalg.norm(dist_vector)
    print(f"--- 轨迹参数计算 ---")
    print(f"  距离: {distance:.4f} mm")
    print(f"  目标速度: {TARGET_VELOCITY:.4f} mm/s")
    
    if distance == 0:
        print("  [警告] 距离为 0，无法计算速度，默认设定时长 5s")
        MOTION_DURATION_SEC = 5.0
    else:
        # 2. 根据七阶多项式分段特性反推时间
        # 公式: Distance = V_flat * (T_acc/K + T_flat + T_dec/K)
        # 归一化后: 1.0 = s_dot_flat * (T_acc/K + T_flat + T_dec/K)
        # 其中 s_dot_flat = V_flat / Distance
        # 
        # 展开: Distance = V_target * (T_acc/K + T_flat + T_dec/K)
        # => T_flat = (Distance / V_target) - (T_acc + T_dec)/K
        
        K = 2.1875
        effective_acc_dec_time = (ACC_DURATION + DEC_DURATION) / K
        
        needed_total_time_for_dist = distance / TARGET_VELOCITY
        
        calculated_t_flat = needed_total_time_for_dist - effective_acc_dec_time
        
        if calculated_t_flat < 0:
            print(f"  [警告] 目标速度过高，无法在给定的加减速时间内达到平顶。")
            print(f"  (需要时间 {needed_total_time_for_dist:.4f}s < 有效加减速时间 {effective_acc_dec_time:.4f}s)")
            print(f"  将自动调整为无匀速段模式 (T_flat=0)，实际速度将低于目标值。")
            calculated_t_flat = 0.0
            
        MOTION_DURATION_SEC = ACC_DURATION + calculated_t_flat + DEC_DURATION
        print(f"  计算得出的总时间: {MOTION_DURATION_SEC:.4f} s (包含 {calculated_t_flat:.4f}s 匀速段)")

    # 1. 生成轨迹 (generate_trajectory 会自动对齐到 4ms)
    trajectory = generate_trajectory(
        START_POSE, 
        END_POSE, 
        MOTION_DURATION_SEC, 
        RSI_CYCLE_TIME_SEC,
        ACC_DURATION,
        DEC_DURATION
    )
    
    # 2. 保存到 CSV
    save_to_csv(trajectory, TRAJECTORY_CSV_FILE)
    
    # 3. 启动服务器 
    start_rsi_server()
