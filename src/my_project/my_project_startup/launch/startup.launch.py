from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
    print_time_update_period_ms = LaunchConfiguration("print_time_update_period_ms")
    tool_change_fixed_time_s = LaunchConfiguration("tool_change_fixed_time_s")
    latency_publish_period_ms = LaunchConfiguration("latency_publish_period_ms")
    latency_history_limit = LaunchConfiguration("latency_history_limit")
    latency_stats_window_limit = LaunchConfiguration("latency_stats_window_limit")
    rsi_period_ms = LaunchConfiguration("rsi_period_ms")
    robot_match_cache_back = LaunchConfiguration("robot_match_cache_back")
    robot_match_cache_forward = LaunchConfiguration("robot_match_cache_forward")
    robot_match_search_back = LaunchConfiguration("robot_match_search_back")
    robot_match_search_forward = LaunchConfiguration("robot_match_search_forward")
    robot_match_max_error_mm = LaunchConfiguration("robot_match_max_error_mm")
    robot_match_uncertainty_min_band_mm = LaunchConfiguration(
        "robot_match_uncertainty_min_band_mm")
    robot_match_uncertainty_spacing_multiplier = LaunchConfiguration(
        "robot_match_uncertainty_spacing_multiplier")
    robot_match_nozzle_lever_mm = LaunchConfiguration("robot_match_nozzle_lever_mm")

    # RSI node params
    sen_type = LaunchConfiguration("sen_type")
    decimal_precision = LaunchConfiguration("decimal_precision")
    local_ip = LaunchConfiguration("local_ip")
    local_port = LaunchConfiguration("local_port")
    abort_lift_mm = LaunchConfiguration("abort_lift_mm")
    abort_lift_speed_mm_s = LaunchConfiguration("abort_lift_speed_mm_s")
    pause_safe_lift_mm = LaunchConfiguration("pause_safe_lift_mm")
    pause_lift_speed_mm_s = LaunchConfiguration("pause_lift_speed_mm_s")
    pause_retract_mm = LaunchConfiguration("pause_retract_mm")
    pause_retract_speed_mm_s = LaunchConfiguration("pause_retract_speed_mm_s")
    fast_first_reply = LaunchConfiguration("fast_first_reply")

    # UART node params
    port = LaunchConfiguration("port")
    run_id = LaunchConfiguration("run_id")
    runtime_log_enabled = LaunchConfiguration("runtime_log_enabled")
    runtime_log_root = LaunchConfiguration("runtime_log_root")
    runtime_log_socket = LaunchConfiguration("runtime_log_socket")
    runtime_log_buffer_bytes = LaunchConfiguration("runtime_log_buffer_bytes")
    runtime_log_batch_bytes = LaunchConfiguration("runtime_log_batch_bytes")
    runtime_log_flush_period_ms = LaunchConfiguration("runtime_log_flush_period_ms")
    runtime_log_sync_period_ms = LaunchConfiguration("runtime_log_sync_period_ms")
    runtime_log_rotate_bytes = LaunchConfiguration("runtime_log_rotate_bytes")
    runtime_log_shutdown_timeout_ms = LaunchConfiguration("runtime_log_shutdown_timeout_ms")
    runtime_ui_period_ms = LaunchConfiguration("runtime_ui_period_ms")
    # RSI experiment metadata written into the session manifest.
    rsi_xml_sha256 = LaunchConfiguration("rsi_xml_sha256")
    rsi_context = LaunchConfiguration("rsi_context")
    rsi_pt1_t1 = LaunchConfiguration("rsi_pt1_t1")
    rsi_precision = LaunchConfiguration("rsi_precision")
    rsi_tool_base = LaunchConfiguration("rsi_tool_base")
    rsi_spline_config = LaunchConfiguration("rsi_spline_config")
    rsi_extra_json = LaunchConfiguration("rsi_extra_json")
    baudrate = LaunchConfiguration("baudrate")
    extrude_scale = LaunchConfiguration("extrude_scale")
    extrusion_wire_mode = LaunchConfiguration("extrusion_wire_mode")
    dispatch_policy = LaunchConfiguration("dispatch_policy")
    uniform_rate_mm_per_s = LaunchConfiguration("uniform_rate_mm_per_s")

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
            "pause_safe_lift_mm": pause_safe_lift_mm,
            "pause_lift_speed_mm_s": pause_lift_speed_mm_s,
            "pause_retract_mm": pause_retract_mm,
            "pause_retract_speed_mm_s": pause_retract_speed_mm_s,
            "fast_first_reply": fast_first_reply,
            "run_id": run_id,
            "runtime_log_enabled": runtime_log_enabled,
            "runtime_log_socket": runtime_log_socket,
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
            "extrusion_wire_mode": extrusion_wire_mode,
            "dispatch_policy": dispatch_policy,
            "uniform_rate_mm_per_s": uniform_rate_mm_per_s,
            "run_id": run_id,
            "runtime_log_enabled": runtime_log_enabled,
            "runtime_log_socket": runtime_log_socket,
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
            "print_time_update_period_ms": print_time_update_period_ms,
            "tool_change_fixed_time_s": tool_change_fixed_time_s,
        }],
    )

    extruder_latency_monitor_node = Node(
        package="control_center",
        executable="extruder_latency_monitor_node",
        name="extruder_latency_monitor_node",
        output="screen",
        parameters=[{
            "latency_publish_period_ms": latency_publish_period_ms,
            "run_id": run_id,
            "runtime_log_enabled": runtime_log_enabled,
            "runtime_log_socket": runtime_log_socket,
            "latency_history_limit": latency_history_limit,
            "latency_stats_window_limit": latency_stats_window_limit,
            "rsi_period_ms": rsi_period_ms,
            "robot_match_cache_back": robot_match_cache_back,
            "robot_match_cache_forward": robot_match_cache_forward,
            "robot_match_search_back": robot_match_search_back,
            "robot_match_search_forward": robot_match_search_forward,
            "robot_match_max_error_mm": robot_match_max_error_mm,
            "robot_match_uncertainty_min_band_mm": robot_match_uncertainty_min_band_mm,
            "robot_match_uncertainty_spacing_multiplier":
                robot_match_uncertainty_spacing_multiplier,
            "robot_match_nozzle_lever_mm": robot_match_nozzle_lever_mm,
        }],
    )

    # Runtime logger: whitelisted JSONL recorder, owns the IPC socket.
    runtime_logger_node = Node(
        package="runtime_logger",
        executable="runtime_logger_node",
        name="runtime_logger_node",
        output="screen",
        parameters=[{
            "run_id": run_id,
            "runtime_log_enabled": runtime_log_enabled,
            "runtime_log_root": runtime_log_root,
            "runtime_log_socket": runtime_log_socket,
            "runtime_log_buffer_bytes": runtime_log_buffer_bytes,
            "runtime_log_batch_bytes": runtime_log_batch_bytes,
            "runtime_log_flush_period_ms": runtime_log_flush_period_ms,
            "runtime_log_sync_period_ms": runtime_log_sync_period_ms,
            "runtime_log_rotate_bytes": runtime_log_rotate_bytes,
            "runtime_log_shutdown_timeout_ms": runtime_log_shutdown_timeout_ms,
            "rsi_xml_sha256": rsi_xml_sha256,
            "rsi_context": rsi_context,
            "rsi_pt1_t1": rsi_pt1_t1,
            "rsi_precision": rsi_precision,
            "rsi_tool_base": rsi_tool_base,
            "rsi_spline_config": rsi_spline_config,
            "rsi_extra_json": rsi_extra_json,
        }],
    )

    # 1 Hz UI telemetry aggregator.
    runtime_telemetry_ui_node = Node(
        package="runtime_logger",
        executable="runtime_telemetry_ui_node",
        name="runtime_telemetry_ui_node",
        output="screen",
        parameters=[{
            "runtime_ui_period_ms": runtime_ui_period_ms,
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
            default_value="100",
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
            "print_time_update_period_ms",
            default_value="500",
            description="预计打印时间 UI 更新周期（毫秒）。",
        ),
        DeclareLaunchArgument(
            "tool_change_fixed_time_s",
            default_value="15.0",
            description="实测单次阻塞换头固定耗时（秒）。",
        ),
        DeclareLaunchArgument(
            "latency_publish_period_ms",
            default_value="300",
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
            "fast_first_reply",
            default_value="true",
            description="是否仅对首个 KUKA UDP 包快速回包，不推进挤出/轨迹链路。",
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
            "extrusion_wire_mode",
            default_value="canonical_v1",
            description="UART 挤出协议模式（canonical_v1/legacy_v1）。",
        ),
        DeclareLaunchArgument(
            "dispatch_policy",
            default_value="seq_bound",
            description="挤出派发策略: seq_bound/uniform_feedforward，默认 seq_bound。",
        ),
        DeclareLaunchArgument(
            "uniform_rate_mm_per_s",
            default_value="10.0",
            description="uniform_feedforward 基线的名义挤出速率 mm/s。",
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
        DeclareLaunchArgument(
            "pause_safe_lift_mm",
            default_value="20.0",
            description="暂停收尾后安全抬升距离（mm）。",
        ),
        DeclareLaunchArgument(
            "pause_lift_speed_mm_s",
            default_value="10.0",
            description="暂停安全抬升和返回速度（mm/s）。",
        ),
        DeclareLaunchArgument(
            "pause_retract_mm",
            default_value="2.0",
            description="暂停树脂回抽距离（mm）。",
        ),
        DeclareLaunchArgument(
            "pause_retract_speed_mm_s",
            default_value="2.0",
            description="暂停树脂回抽速度（mm/s）。",
        ),
        DeclareLaunchArgument(
            "run_id",
            default_value="",
            description="运行日志会话 ID；空值由 runtime_logger 自动生成。",
        ),
        DeclareLaunchArgument(
            "runtime_log_enabled",
            default_value="true",
            description="启用白名单运行日志（仅记录六类业务字段）。",
        ),
        DeclareLaunchArgument(
            "runtime_log_root",
            default_value=[
                EnvironmentVariable("HOME", default_value="/tmp"),
                "/kuka_ram_ws/data/runtime_logs",
            ],
            description="运行日志根目录。",
        ),
        DeclareLaunchArgument(
            "runtime_log_socket",
            default_value="",
            description="运行日志 AF_UNIX socket 路径；空值按 run_id 生成。",
        ),
        DeclareLaunchArgument(
            "runtime_log_buffer_bytes",
            default_value="8388608",
            description="运行日志总缓冲预算（字节）。",
        ),
        DeclareLaunchArgument(
            "runtime_log_batch_bytes",
            default_value="65536",
            description="写盘批量阈值（字节）。",
        ),
        DeclareLaunchArgument(
            "runtime_log_flush_period_ms",
            default_value="100",
            description="低流量下最长写盘等待（毫秒）。",
        ),
        DeclareLaunchArgument(
            "runtime_log_sync_period_ms",
            default_value="1000",
            description="fdatasync 持久化检查点周期（毫秒）。",
        ),
        DeclareLaunchArgument(
            "runtime_log_rotate_bytes",
            default_value="67108864",
            description="单个 JSONL 分卷字节上限。",
        ),
        DeclareLaunchArgument(
            "runtime_log_shutdown_timeout_ms",
            default_value="5000",
            description="关闭时排空日志的最长等待（毫秒）。",
        ),
        DeclareLaunchArgument(
            "runtime_ui_period_ms",
            default_value="1000",
            description="UI 遥测聚合发布周期（毫秒）。",
        ),
        # RSI experiment metadata written into the run manifest so a
        # logged session can be traced back to the robot-side config.
        DeclareLaunchArgument(
            "rsi_xml_sha256",
            default_value=(
                "3dae7e83a2a96529856bebf772306ee5af598d2a4bab2f70ab7377b9b25fa695"),
            description=(
                "SHA256 of RSI_PythonConfig.xml inside the deployed WVS. "
                "Update if the XML changes; the logged hash must match the "
                "file the controller actually loads."),
        ),
        DeclareLaunchArgument(
            "rsi_context",
            default_value="PythonHomeDemo",
            description="RSI context name from RSI_CREATE in the .src.",
        ),
        DeclareLaunchArgument(
            "rsi_pt1_t1",
            default_value="0.012",
            description=(
                "PT1.T1 set via RSI_SETPUBLICPAR (Filter_T1 in .src). "
                "WorkVisual canvas shows the static default 1; the runtime "
                "value is what the filter actually uses."),
        ),
        DeclareLaunchArgument(
            "rsi_precision",
            default_value="4",
            description="Ethernet_1 Precision parameter.",
        ),
        DeclareLaunchArgument(
            "rsi_tool_base",
            default_value="",
            description=(
                "Active $TOOL/$BASE/$LOAD identification for this run "
                "(e.g. tool_data index + base frame). Fill before logging."),
        ),
        DeclareLaunchArgument(
            "rsi_spline_config",
            default_value="",
            description=(
                "Identifier for the offline spline/path config used in this "
                "run (e.g. npz filename or spline smoothing profile)."),
        ),
        DeclareLaunchArgument(
            "rsi_extra_json",
            default_value="",
            description=(
                "Free-form JSON object merged into manifest config.extra; "
                "use for one-off fields not covered by named parameters."),
        ),
        rsi_node,
        uart_node,
        system_manager_node,
        extruder_latency_monitor_node,
        runtime_logger_node,
        runtime_telemetry_ui_node,
        center_delayed,
    ])
