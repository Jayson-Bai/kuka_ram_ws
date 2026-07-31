# KUKA RAM 机器人增材制造系统

本仓库是一个面向 KUKA 工业机械臂增材制造任务的 ROS 2 工作区。系统将离线 GCode 轨迹规划、KUKA RSI 实时通信、挤出机 UART 控制、状态聚合、延迟监控和 RQT 操作界面拆成多个 ROS 2 包，通过自定义消息和固定话题连接。

核心目标是让机械臂位姿和打印头挤出量以 RSI 心跳为主时钟同步执行：离线阶段把 GCode 转成 4 ms 采样的 `npz`/manifest 轨迹；在线阶段由 `center_node` 送轨迹，`rsi_node` 按 KUKA UDP 心跳回包，`uart_node` 按同一个心跳下发绝对挤出量。

## 快速开始

构建工作区：

```bash
cd /home/jayson/kuka_ram_ws
colcon build
source install/setup.bash
```

从 GCode 离线生成轨迹：

```bash
ros2 run gcode_planner gcode_planner_npz \
  --data-root /home/jayson/kuka_ram_ws/data
```

或显式指定输入输出：

```bash
ros2 run gcode_planner gcode_planner_npz \
  --gcode /path/to/input.gcode \
  --out /home/jayson/kuka_ram_ws/data/output_npz/output.npz
```

启动真实在线链路：

```bash
ros2 launch my_project_startup startup.launch.py \
  npz_path:=/home/jayson/kuka_ram_ws/data/output_npz/test.npz
```

启动 RQT 操作界面：

```bash
rqt --standalone my_project_ui/UiPanel
```

启动仿真链路（虚拟串口、Mock KUKA、Mock MCU、RQT 与主系统一起启动）：

```bash
ros2 launch my_project_simulation simulation.launch.py \
  npz_path:=/home/jayson/kuka_ram_ws/data/output_npz/test.npz
```

仿真启动依赖 `socat` 创建 `/tmp/ttyV0` 与 `/tmp/ttyV1` 虚拟串口。

## 项目架构

```text
GCode
  |
  |  gcode_planner
  v
NPZ / manifest trajectory
  |
  |  control_center:center_node
  v
/planned_trajectory + /planned_events
  |
  |  rsi_server:rsi_node <---- KUKA RSI UDP heartbeat
  v
/rsi/heartbeat + /rsi/triggered_event + /kuka/status
  |
  |  uart_bridge:uart_node
  v
UART extruder / MCU

system_manager_node + extruder_latency_monitor_node
  |
  v
/ui/status + /extruder/latency_status
  |
  v
my_project_ui RQT panel
```

### 离线规划层

`gcode_planner` 读取切片软件生成的 `.gcode`，解析运动、换刀、加热、风扇、挤出复位等指令，并将连续同类型路径交给共享 exporter 拟合或采样。普通长路径会拟合成更平滑的 B 样条轨迹；连续过短线段簇会保持原始折线点序作为 `POLYLINE` 连续采样，避免细碎段反复触发独立加减速。采样周期默认是 `0.004 s`，与 KUKA RSI 4 ms 控制周期对齐。

主要模块：

- `gcode_parser.py`：解析 `G0/G1`、`G90/G91`、`M82/M83`、`G92 E...`、`Tn` 和常见 `M` 指令。
- `bspline_approximation.py`：执行角点回退、点密度加密和全局 B 样条最小二乘拟合。
- `polynomial_interpolator.py`：按时间步采样轨迹，并按路径比例分配挤出量。
- `npz_exporter.py`：导出 `seq/x/y/z/a/b/c/e/tool_id` 以及事件字段；支持单文件 `npz` 或按层/类型拆分的 manifest。默认同时写入 `planned_time_s` RSI 时间轴和 `<base>.timing.json`（拆分输出为 `<base>_timing.json`），时间轴包含每段的加速、匀速、减速参数化元数据。
- `cli.py`：提供 `ros2 run gcode_planner gcode_planner_npz ...` 命令入口。

正式模式预计打印时间直接以最终注入后的 RSI 点为准：七阶多项式运动采样点、空走点、固定驻留点，以及按“长度÷速度”离线展开的预挤出/回抽点统一按固定 4 ms 周期生成累计时间表。剪切抬升、等待和安全回抽也已经体现在这些点中，不重复相加；非阻塞 `cut` 事件自身不额外计时。只有不在点表中且会阻塞 RSI 的换头事件按实测 `15 s × 换头次数` 叠加。加热、风扇确认、挤出复位、通信/调度抖动、人工暂停和故障恢复按现场状态决定，按约定忽略。静态分项通过一次性的 `/print/timing_plan` 发布，250 Hz `TrajectoryPoint` 只携带当前点的离线累计时间，运行时按 `seq_used` O(1) 查表。

`e` 字段在系统中始终表示绝对挤出量。`G92 E...` 会导出 `extrude_reset` 事件，供 UART 侧和固件状态同步。

### 在线控制层

`control_center` 提供三个核心节点：

- `center_node`：加载 `npz` 或 manifest，使用 `NpzLoader` 和 `QueueManager` 维护轨迹/事件队列，发布 `/planned_trajectory` 和 `/planned_events`。它订阅 `/rsi/heartbeat`，根据实际使用的 `seq_used` 继续补发轨迹，避免下游队列过低或过高。
- `system_manager_node`：聚合 KUKA 状态、RSI 心跳、打印头状态、计划轨迹和事件，周期发布 `/ui/status`，供 UI 展示。预计时间在该节点的非 RSI 线程中按 `seq_used` 做 O(1) 低频更新，默认周期为 `print_time_update_period_ms=500` ms。
- `extruder_latency_monitor_node`：订阅 `/rsi/heartbeat`、`/uart/raw`、`/printhead/status`、`/planned_trajectory`、`/kuka/status`，输出 `/extruder/latency_status` 和 `/extruder/latency_text`，用于估计 Linux-MCU、Linux-Robot、MCU-Robot 等延迟。

`rsi_server:rsi_node` 是 KUKA RSI UDP 服务端。它订阅中心节点的轨迹和事件、打印头 ready 状态与系统命令，在每个 KUKA IPOC 心跳内选择轨迹点并构造 RSI XML 回包，同时发布：

- `/kuka/raw_xml`：KUKA 原始 XML。
- `/kuka/status`：解析出的实际位姿 `x/y/z/a/b/c`。
- `/rsi/heartbeat`：本周期实际使用的 `seq_used/tool_id/extrude_abs`。
- `/rsi/triggered_event`：到达触发序号的事件，转发给 UART。
- `/rsi/current_correction`：当前 RSI 修正量，供测试模式使用。

`uart_bridge:uart_node` 负责串口协议。它订阅 `/rsi/heartbeat` 后发送：

```text
E <seq_used> <tool_id> <extrude_abs>
```

事件由 `/rsi/triggered_event` 触发，发送：

```text
EV <trigger_seq> <event_type> <payload>
```

MCU 返回的 `STAT`、`EVACK`、`EVDONE` 等行会被解析为 `/printhead/status`，同时原始串口日志发布到 `/uart/raw`。`cut` 事件发送后在 UART 节点侧按非阻塞事件处理，避免 RSI 链路等待剪切 ACK/DONE；对应的抬升、等待补足和安全回抽已经在离线系统 NPZ 中展开。`extrude_scale` 是 `uart_node` 的运行时参数，只影响最终串口发送的挤出量，不改变轨迹、UI 或 RSI 心跳中的原始绝对 E。

### 操作界面与仿真层

`my_project_ui` 是 RQT 插件，入口为 `my_project_ui/UiPanel`。它订阅 `/ui/status`、`/extruder/latency_status`、`/uart/raw`、`/rsi/sent_xml` 和 `/rsi/current_correction`，并发布系统命令、UART 手动命令和打印测试命令。UI 详细结构见 `src/my_project/my_project_ui/UI_ARCHITECTURE.md`。

`my_project_simulation` 提供：

- `mock_kuka`：模拟 KUKA RSI UDP 心跳。
- `mock_mcu`：模拟打印头 MCU 串口状态。
- `simulation.launch.py`：创建虚拟串口对，启动 mock 节点、主系统和 RQT 面板。

## ROS 2 包说明

| 包 | 类型 | 作用 |
| --- | --- | --- |
| `gcode_planner` | Python / `ament_python` | 离线 GCode 解析、B 样条拟合、4 ms 采样、`npz`/manifest 导出 |
| `control_center` | C++ / `ament_cmake` | 轨迹加载与队列、中心发布、UI 状态聚合、挤出延迟监控 |
| `rsi_server` | C++ / `ament_cmake` | KUKA RSI UDP 通信、状态机、轨迹回包、心跳与事件触发 |
| `uart_bridge` | C++ / `ament_cmake` | Boost.Asio 串口通信、挤出命令、事件命令、打印头状态解析 |
| `my_project_interfaces` | ROSIDL | 自定义消息定义 |
| `my_project_startup` | Python launch | 集中启动真实在线链路并透传参数 |
| `my_project_ui` | Python RQT 插件 | 操作员监控面板和参数/命令入口 |
| `my_project_simulation` | Python launch + mocks | 本机仿真 KUKA 与 MCU |
| `third_party` | 外部依赖 | KUKA drivers、External Control SDK、robot descriptions 等 |

## 关键话题和消息

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
| --- | --- | --- | --- | --- |
| `/planned_trajectory` | `TrajectoryPoint` | `center_node` | `rsi_node`, `system_manager_node`, `extruder_latency_monitor_node` | 规划轨迹点，包含 `XYZABC`、绝对 `E`、`tool_id`、`seq` 和可选 RSI 计划时间 |
| `/print/timing_plan` | `PrintTimingPlan` | `center_node` | `system_manager_node` | 单次发布的离线时间总量、分项和固定事件计数 |
| `/planned_events` | `PlannedEvent` | `center_node` | `rsi_node`, `system_manager_node` | 计划事件，如换刀、加热、风扇、挤出复位 |
| `/rsi/heartbeat` | `RsiHeartBeat` | `rsi_node` | `center_node`, `uart_node`, `system_manager_node`, `extruder_latency_monitor_node` | RSI 主时钟，携带当前执行序号与绝对挤出量 |
| `/rsi/triggered_event` | `PlannedEvent` | `rsi_node` | `uart_node`, `system_manager_node` | 到达触发序号的事件 |
| `/kuka/status` | `KukaStatus` | `rsi_node` | `system_manager_node`, `extruder_latency_monitor_node` | KUKA 实际位姿 |
| `/printhead/status` | `PrintHeadStatus` | `uart_node` | `center_node`, `rsi_node`, `system_manager_node`, `extruder_latency_monitor_node` | 打印头 ready、温度、风扇、工具号和错误码 |
| `/uart/raw` | `std_msgs/String` | `uart_node` | UI, `extruder_latency_monitor_node` | 串口收发日志 |
| `/ui/status` | `UiStatus` | `system_manager_node` | UI | 面向 UI 的状态快照 |
| `/extruder/latency_status` | `ExtruderLatencyStatus` | `extruder_latency_monitor_node` | UI | 延迟统计和匹配结果 |
| `/system/command` | `std_msgs/String` | UI/手动工具 | `center_node`, `rsi_node`, `uart_node`, `system_manager_node` | `PAUSE`、`RESUME`、`ABORT` 等系统命令 |

主要消息定义位于 `src/my_project/my_project_interfaces/msg/`：

- `TrajectoryPoint.msg`：轨迹点，包含 `stamp, x, y, z, a, b, c, e, tool_id, seq`。
- `PlannedEvent.msg`：事件，包含 `event_type, payload, event_src_line, trigger_seq`。
- `RsiHeartBeat.msg`：RSI 心跳，包含 `ipoc, seq_used, tool_id, extrude_abs`。
- `KukaStatus.msg`：KUKA 实际位姿。
- `PrintHeadStatus.msg`：打印头 ready、温度、风扇、当前工具和错误码。
- `UiStatus.msg`：UI 聚合快照。
- `ExtruderLatencyStatus.msg`：挤出延迟和机器人位置匹配统计。

## 启动流程

`my_project_startup/launch/startup.launch.py` 默认启动：

1. `rsi_node`：监听 KUKA RSI UDP。
2. `uart_node`：打开串口并发布初始 ready 状态。
3. `system_manager_node`：发布 `/ui/status`。
4. `extruder_latency_monitor_node`：发布延迟监控状态。
5. 延迟启动 `center_node`：默认延迟 `1.0 s`，避免轨迹/事件在订阅关系建立前丢失。

查看完整 launch 参数：

```bash
ros2 launch my_project_startup startup.launch.py --show-args
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `npz_path` | `/home/jayson/kuka_ram_ws/data/output_npz/test.npz` | 轨迹 `npz` 或 manifest 路径 |
| `center_start_delay_s` | `1.0` | `center_node` 延迟启动秒数 |
| `plan_qos_depth` | `2000` | 轨迹和事件话题 QoS 深度 |
| `traj_prefill` | `1000` | 启动时预填充轨迹点数量 |
| `local_ip` | `192.168.1.1` | RSI 本地监听 IP |
| `local_port` | `49152` | RSI 本地监听端口 |
| `sen_type` | `PythonDemo` | RSI XML 中的 `Sen Type`，需与 KUKA 端配置一致 |
| `decimal_precision` | `4` | RSI 收发小数位精度 |
| `fast_first_reply` | `true` | 首个 UDP 包快速回包，不推进轨迹/挤出链路 |
| `port` | `/dev/ttyUSB0` | UART 串口路径 |
| `baudrate` | `115200` | UART 波特率 |
| `extrude_scale` | `1.0` | UART 挤出倍率 |
| `abort_lift_mm` | `100.0` | `ABORT` 时 Z 轴抬升距离 |
| `abort_lift_speed_mm_s` | `10.0` | `ABORT` 时 Z 轴抬升速度 |
| `latency_publish_period_ms` | `200` | 延迟状态发布周期 |
| `print_time_update_period_ms` | `500` | UI 预计打印时间更新周期；不运行在 RSI 实时线程 |
| `latency_stats_window_limit` | `5000` | P95/P99 延迟统计窗口 |
| `robot_match_max_error_mm` | `1.0` | 机器人实际位置匹配允许的最大空间误差 |

按层/类型导出的 manifest 可以直接作为 `npz_path`：

```bash
ros2 launch my_project_startup startup.launch.py \
  npz_path:=/home/jayson/kuka_ram_ws/data/output_npz/<base>/<base>_manifest.json
```

## 数据目录

约定的数据目录位于仓库根目录的 `data/`：

```text
data/
  input_gcode/    # GCode 输入
  output_npz/     # gcode_planner 输出的 npz 或 manifest
```

通过 `ros2 run` 执行已安装包时，默认路径可能随安装目录变化。建议在开发和调试时显式传入 `--data-root`、`--gcode`、`--out` 或 launch 的 `npz_path`。

## 开发和测试入口

常用构建：

```bash
colcon build --packages-select gcode_planner control_center rsi_server uart_bridge my_project_interfaces my_project_startup my_project_ui my_project_simulation
source install/setup.bash
```

运行 Python 包测试示例：

```bash
colcon test --packages-select gcode_planner my_project_ui my_project_startup my_project_simulation
colcon test-result --verbose
```

运行 C++ 包构建检查示例：

```bash
colcon build --packages-select control_center rsi_server uart_bridge my_project_interfaces
```

可用辅助材料：

- `src/my_project/gcode_planner/README.md`：GCode 解析、B 样条拟合、NPZ 输出字段和 E 链路细节。
- `src/my_project/my_project_startup/README.md`：启动顺序、参数和 manifest 使用说明。
- `src/my_project/my_project_ui/UI_ARCHITECTURE.md`：RQT UI 插件架构、订阅关系和界面职责。
- `graphify-out/GRAPH_REPORT.md`、`graphify-out/graph.json`：当前代码库的 graphify 架构图谱输出。

## 运行时注意事项

- RSI 是在线链路的主时钟。`uart_node` 不直接消费规划轨迹，而是消费 `/rsi/heartbeat` 中的 `seq_used/tool_id/extrude_abs`，保证挤出命令跟随机械臂实际回包节奏。
- `center_node` 依赖 `/printhead/status.ready_for_motion` 决定是否继续发布新轨迹。事件触发后，UART 会清 ready，直到 MCU 通过事件完成状态恢复。
- `PAUSE` 和 `ABORT` 会同时影响 `center_node`、`rsi_node` 和 `uart_node`。`ABORT` 时 RSI 节点按配置执行 Z 轴抬升保护。
- KUKA 真实控制柜参数必须与 launch 中的 `local_ip`、`local_port`、`sen_type` 和小数位精度匹配。
- 若使用仿真，`simulation.launch.py` 会将 RSI IP 改为 `127.0.0.1`，并将 UART 端口改为 `/tmp/ttyV0`。
