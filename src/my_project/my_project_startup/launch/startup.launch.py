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

    # RSI node params
    sen_type = LaunchConfiguration("sen_type")
    decimal_precision = LaunchConfiguration("decimal_precision")
    local_ip = LaunchConfiguration("local_ip")
    local_port = LaunchConfiguration("local_port")
    abort_lift_mm = LaunchConfiguration("abort_lift_mm")
    abort_lift_speed_mm_s = LaunchConfiguration("abort_lift_speed_mm_s")

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
            default_value="2",
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
        center_delayed,
    ])
