from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Center node params
    npz_path = LaunchConfiguration("npz_path")
    npz_preload_chunks = LaunchConfiguration("npz_preload_chunks")
    queue_low = LaunchConfiguration("queue_low")
    queue_high = LaunchConfiguration("queue_high")
    plan_qos_depth = LaunchConfiguration("plan_qos_depth")
    traj_prefill = LaunchConfiguration("traj_prefill")
    traj_low = LaunchConfiguration("traj_low")
    traj_high = LaunchConfiguration("traj_high")
    xyzabc_decimals = LaunchConfiguration("xyzabc_decimals")
    e_decimals = LaunchConfiguration("e_decimals")
    kuka_status_raw = LaunchConfiguration("kuka_status_raw")
    summary_period_ms = LaunchConfiguration("summary_period_ms")

    # System manager params (UI status publisher)
    ui_publish_period_ms = LaunchConfiguration("ui_publish_period_ms")
    heartbeat_timeout_s = LaunchConfiguration("heartbeat_timeout_s")
    traj_queue_limit = LaunchConfiguration("traj_queue_limit")
    event_queue_limit = LaunchConfiguration("event_queue_limit")
    latency_publish_period_ms = LaunchConfiguration("latency_publish_period_ms")
    latency_history_limit = LaunchConfiguration("latency_history_limit")
    latency_stats_window_limit = LaunchConfiguration("latency_stats_window_limit")
    rsi_period_ms = LaunchConfiguration("rsi_period_ms")
    robot_match_cache_back = LaunchConfiguration("robot_match_cache_back")
    robot_match_cache_forward = LaunchConfiguration("robot_match_cache_forward")
    robot_match_search_back = LaunchConfiguration("robot_match_search_back")
    robot_match_search_forward = LaunchConfiguration("robot_match_search_forward")
    robot_match_max_error_mm = LaunchConfiguration("robot_match_max_error_mm")
    robot_match_uncertainty_min_band_mm = LaunchConfiguration("robot_match_uncertainty_min_band_mm")
    robot_match_uncertainty_spacing_multiplier = LaunchConfiguration("robot_match_uncertainty_spacing_multiplier")
    robot_match_nozzle_lever_mm = LaunchConfiguration("robot_match_nozzle_lever_mm")

    # RSI node params
    sen_type = LaunchConfiguration("sen_type")
    decimal_precision = LaunchConfiguration("decimal_precision")
    local_ip = LaunchConfiguration("local_ip")
    local_port = LaunchConfiguration("local_port")
    abort_lift_mm = LaunchConfiguration("abort_lift_mm")
    abort_lift_speed_mm_s = LaunchConfiguration("abort_lift_speed_mm_s")
    diag_sample_period = LaunchConfiguration("diag_sample_period")

    # UART node params
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    extrude_scale = LaunchConfiguration("extrude_scale")

    rsi_node = Node(
        package="rsi_server",
        executable="rsi_node",
        name="rsi_node",
        output="screen",
        parameters=[{
            "sen_type": sen_type,
            "decimal_precision": decimal_precision,
            "local_ip": local_ip,
            "local_port": local_port,
            "abort_lift_mm": abort_lift_mm,
            "abort_lift_speed_mm_s": abort_lift_speed_mm_s,
            "diag_sample_period": diag_sample_period,
        }],
    )

    uart_node = Node(
        package="uart_bridge",
        executable="uart_node",
        name="uart_node",
        output="screen",
        parameters=[{
            "port": port,
            "baudrate": baudrate,
            "extrude_scale": extrude_scale,
        }],
    )

    center_node = Node(
        package="control_center",
        executable="center_node",
        name="center_node",
        output="screen",
        parameters=[{
            "npz_path": npz_path,
            "npz_preload_chunks": npz_preload_chunks,
            "queue_low": queue_low,
            "queue_high": queue_high,
            "plan_qos_depth": plan_qos_depth,
            "traj_prefill": traj_prefill,
            "traj_low": traj_low,
            "traj_high": traj_high,
            "xyzabc_decimals": xyzabc_decimals,
            "e_decimals": e_decimals,
            "kuka_status_raw": kuka_status_raw,
            "summary_period_ms": summary_period_ms,
        }],
    )

    system_manager_node = Node(
        package="control_center",
        executable="system_manager_node",
        name="system_manager_node",
        output="screen",
        parameters=[{
            "ui_publish_period_ms": ui_publish_period_ms,
            "heartbeat_timeout_s": heartbeat_timeout_s,
            "traj_queue_limit": traj_queue_limit,
            "event_queue_limit": event_queue_limit,
        }],
    )

    extruder_latency_monitor_node = Node(
        package="control_center",
        executable="extruder_latency_monitor_node",
        name="extruder_latency_monitor_node",
        output="screen",
        parameters=[{
            "latency_publish_period_ms": latency_publish_period_ms,
            "latency_history_limit": latency_history_limit,
            "latency_stats_window_limit": latency_stats_window_limit,
            "rsi_period_ms": rsi_period_ms,
            "robot_match_cache_back": robot_match_cache_back,
            "robot_match_cache_forward": robot_match_cache_forward,
            "robot_match_search_back": robot_match_search_back,
            "robot_match_search_forward": robot_match_search_forward,
            "robot_match_max_error_mm": robot_match_max_error_mm,
            "robot_match_uncertainty_min_band_mm": robot_match_uncertainty_min_band_mm,
            "robot_match_uncertainty_spacing_multiplier": robot_match_uncertainty_spacing_multiplier,
            "robot_match_nozzle_lever_mm": robot_match_nozzle_lever_mm,
        }],
    )

    # center_node延迟启动
    center_delayed = TimerAction(
        period=LaunchConfiguration("center_start_delay_s"),
        actions=[center_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "center_start_delay_s",
            default_value="1.0",
            description="延迟启动 center_node，确保订阅已建立。",
        ),
        DeclareLaunchArgument(
            "npz_path",
            default_value="/home/jayson/kuka_ram_ws/data/output_npz/test.npz",
            description="轨迹/事件 NPZ 文件路径。",
        ),
        DeclareLaunchArgument(
            "npz_preload_chunks",
            default_value="2",
            description="NPZ 预加载块数。",
        ),
        DeclareLaunchArgument(
            "queue_low",
            default_value="1000",
            description="轨迹队列低水位阈值。",
        ),
        DeclareLaunchArgument(
            "queue_high",
            default_value="2000",
            description="轨迹队列高水位阈值。",
        ),
        DeclareLaunchArgument(
            "plan_qos_depth",
            default_value="2000",
            description="规划话题 QoS 深度。",
        ),
        DeclareLaunchArgument(
            "traj_prefill",
            default_value="1000",
            description="启动预填充轨迹点数量。",
        ),
        DeclareLaunchArgument(
            "traj_low",
            default_value="500",
            description="轨迹 backlog 低阈值告警线。",
        ),
        DeclareLaunchArgument(
            "traj_high",
            default_value="1500",
            description="轨迹 backlog 高阈值告警线。",
        ),
        DeclareLaunchArgument(
            "xyzabc_decimals",
            default_value="6",
            description="轨迹位姿小数保留位数。",
        ),
        DeclareLaunchArgument(
            "e_decimals",
            default_value="6",
            description="挤出量小数保留位数。",
        ),
        DeclareLaunchArgument(
            "kuka_status_raw",
            default_value="false",
            description="是否打印 KUKA 原始 XML 长度。",
        ),
        DeclareLaunchArgument(
            "summary_period_ms",
            default_value="200",
            description="控制中心汇总发布周期（毫秒）。",
        ),
        DeclareLaunchArgument(
            "ui_publish_period_ms",
            default_value="200",
            description="UI 状态发布周期（毫秒）。",
        ),
        DeclareLaunchArgument(
            "heartbeat_timeout_s",
            default_value="1.0",
            description="心跳超时时间（秒）。",
        ),
        DeclareLaunchArgument(
            "traj_queue_limit",
            default_value="5000",
            description="UI 侧轨迹队列上限。",
        ),
        DeclareLaunchArgument(
            "event_queue_limit",
            default_value="2000",
            description="UI 侧事件队列上限。",
        ),
        DeclareLaunchArgument(
            "latency_publish_period_ms",
            default_value="200",
            description="挤出延迟监控状态发布周期（毫秒）。",
        ),
        DeclareLaunchArgument(
            "latency_history_limit",
            default_value="5000",
            description="挤出延迟监控保存的 RSI 心跳序号数量。",
        ),
        DeclareLaunchArgument(
            "latency_stats_window_limit",
            default_value="5000",
            description="延迟 P95/P99 统计窗口样本数。",
        ),
        DeclareLaunchArgument(
            "rsi_period_ms",
            default_value="4.0",
            description="RSI 控制周期（毫秒），用于 seq_lag 换算。",
        ),
        DeclareLaunchArgument(
            "robot_match_cache_back",
            default_value="8000",
            description="机械臂位置匹配保留当前序号之前的轨迹点数量。",
        ),
        DeclareLaunchArgument(
            "robot_match_cache_forward",
            default_value="1000",
            description="机械臂位置匹配保留当前序号之后的轨迹点数量。",
        ),
        DeclareLaunchArgument(
            "robot_match_search_back",
            default_value="5000",
            description="机械臂实际位置匹配向后搜索的序号范围。",
        ),
        DeclareLaunchArgument(
            "robot_match_search_forward",
            default_value="300",
            description="机械臂实际位置匹配向前搜索的序号范围。",
        ),
        DeclareLaunchArgument(
            "robot_match_max_error_mm",
            default_value="1.0",
            description="机械臂位置匹配允许的最大空间误差（毫米）。",
        ),
        DeclareLaunchArgument(
            "robot_match_uncertainty_min_band_mm",
            default_value="0.10",
            description="计算匹配不确定度时的最小误差带（毫米）。",
        ),
        DeclareLaunchArgument(
            "robot_match_uncertainty_spacing_multiplier",
            default_value="3.0",
            description="计算匹配不确定度时的局部轨迹间距倍率。",
        ),
        DeclareLaunchArgument(
            "robot_match_nozzle_lever_mm",
            default_value="401.68",
            description="TCP 姿态误差等效臂长（毫米），默认按 KUKA 工具 XYZ=(-2.99,-1.27,401.67) 计算。",
        ),
        DeclareLaunchArgument(
            "sen_type",
            default_value="PythonDemo",
            description="RSI XML 发送类型（需与 KUKA 端配置一致）。",
        ),
        DeclareLaunchArgument(
            "decimal_precision",
            default_value="4",
            description="RSI 收发数据小数位精度。",
        ),
        DeclareLaunchArgument(
            "local_ip",
            default_value="192.168.1.1",
            description="RSI 本地监听 IP。",
        ),
        DeclareLaunchArgument(
            "local_port",
            default_value="49152",
            description="RSI 本地监听端口。",
        ),
        DeclareLaunchArgument(
            "diag_sample_period",
            default_value="50",
            description="RSI 原始/回包 XML 诊断发布采样周期，1 表示每包发布。",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="UART 串口设备路径。",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="UART 波特率。",
        ),
        DeclareLaunchArgument(
            "extrude_scale",
            default_value="1.0",
            description="UART 挤出倍率（仅影响串口发送的挤出量）。",
        ),
        DeclareLaunchArgument(
            "abort_lift_mm",
            default_value="100.0",
            description="ABORT 时 Z 轴抬升距离（mm）。",
        ),
        DeclareLaunchArgument(
            "abort_lift_speed_mm_s",
            default_value="10.0",
            description="ABORT 时 Z 轴抬升速度（mm/s）。",
        ),
        rsi_node,
        uart_node,
        system_manager_node,
        extruder_latency_monitor_node,
        center_delayed,
    ])
