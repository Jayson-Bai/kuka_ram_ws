# my_project_startup

该包用于集中管理启动流程，包含核心节点的启动顺序与参数透传。

## 默认启动顺序

- 先启动 `rsi_node` 和 `uart_node`
- 启动 `system_manager_node`（发布 `/ui/status`，给 UI 使用）
- 延迟启动 `center_node`（避免预填充轨迹/事件在订阅未建立时丢失）

## /kuka/status 的来源

`/kuka/status` 由 `rsi_node` 在接收 RSI XML 后解析并发布，无单独的解析节点。

## 启动命令

```bash
ros2 launch my_project_startup startup.launch.py
```

## 启动参数

全部可配置参数如下（含默认值），按节点分组：

### center_node （中心控制节点）
- `center_start_delay_s`：延迟启动 center_node（秒），默认 `1.0`
- `npz_path`：轨迹/事件 NPZ 文件路径，默认 `/home/jayson/kuka_ram_ws/data/output_npz/test.npz`
- `npz_preload_chunks`：NPZ 预加载块数，默认 `2`
- `queue_low`：轨迹队列低水位阈值，默认 `1000`
- `queue_high`：轨迹队列高水位阈值，默认 `2000`
- `plan_qos_depth`：规划话题 QoS 深度，默认 `2000`
- `traj_prefill`：启动预填充轨迹点数量，默认 `1000`
- `traj_low`：轨迹 backlog 低阈值告警线，默认 `500`
- `traj_high`：轨迹 backlog 高阈值告警线，默认 `1500`
- `xyzabc_decimals`：规划出的位姿小数保留位数，默认 `6`
- `e_decimals`：挤出量小数保留位数，默认 `2`
- `kuka_status_raw`：是否打印 KUKA 原始 XML 长度，默认 `false`
- `summary_period_ms`：控制中心汇总发布周期（毫秒），默认 `200`

### rsi_node （机械臂侧）
- `sen_type`：RSI XML 发送类型（需与 KUKA 端配置一致），默认 `PosCorr`
- `decimal_precision`：RSI 收发数据小数位精度，默认 `6`
- `local_ip`：RSI 本地监听 IP，默认 `192.168.1.1`
- `local_port`：RSI 本地监听端口，默认 `49152`

### uart_node （打印头侧）
- `port`：UART 串口设备路径，默认 `/dev/ttyUSB0`
- `baudrate`：UART 波特率，默认 `115200`

### system_manager_node
- `ui_publish_period_ms`：UI 状态发布周期（毫秒），默认 `200`
- `heartbeat_timeout_s`：心跳超时时间（秒），默认 `1.0`
- `traj_queue_limit`：UI 侧轨迹队列上限，默认 `5000`
- `event_queue_limit`：UI 侧事件队列上限，默认 `2000`

## 常用启动参数

使用 `--show-args` 查看完整参数与中文说明：

```bash
ros2 launch my_project_startup startup.launch.py --show-args
```

示例（修改启动使用的npz文件路径、收发小数位数4、KUKA xml中sen_type字段）：
ros2 launch my_project_startup startup.launch.py \
  npz_path:=/home/jayson/kuka_ram_ws/data/output_npz/test2.npz \ 
  sen_type:=PythonDemo \
  decimal_precision:=4

