# my_project_startup

该包用于集中管理启动流程，当前只包含核心三节点的启动顺序与参数透传。

## 默认启动顺序

- 先启动 `rsi_node` 和 `uart_node`
- 延迟启动 `center_node`（避免预填充轨迹/事件在订阅未建立时丢失）

## 启动命令

```bash
ros2 launch my_project_startup startup.launch.py
```

## 启动参数

全部可配置参数如下（含默认值），按节点分组：

### center_node
- `center_start_delay_s`：延迟启动 center_node（秒），默认 `1.0`
- `npz_path`：轨迹/事件 NPZ 文件路径，默认 `/home/jayson/kuka_ram_ws/data/output_npz/test.npz`
- `npz_preload_chunks`：NPZ 预加载块数，默认 `2`
- `queue_low`：轨迹队列低水位阈值，默认 `1000`
- `queue_high`：轨迹队列高水位阈值，默认 `2000`
- `plan_qos_depth`：规划话题 QoS 深度，默认 `2000`
- `traj_prefill`：启动预填充轨迹点数量，默认 `1000`
- `traj_low`：轨迹 backlog 低阈值告警线，默认 `500`
- `traj_high`：轨迹 backlog 高阈值告警线，默认 `1500`
- `xyzabc_decimals`：轨迹位姿小数保留位数，默认 `6`
- `e_decimals`：挤出量小数保留位数，默认 `2`
- `kuka_status_raw`：是否打印 KUKA 原始 XML 长度，默认 `false`
- `summary_period_ms`：控制中心汇总发布周期（毫秒），默认 `200`

### rsi_node
- `sen_type`：RSI XML 发送类型（需与 KUKA 端配置一致），默认 `PosCorr`
- `decimal_precision`：RSI 收发数据小数位精度，默认 `6`
- `local_ip`：RSI 本地监听 IP，默认 `192.168.1.1`
- `local_port`：RSI 本地监听端口，默认 `49152`

### uart_node
- `port`：UART 串口设备路径，默认 `/dev/ttyUSB0`
- `baudrate`：UART 波特率，默认 `115200`

## 常用参数

使用 `--show-args` 查看完整参数与中文说明：

```bash
ros2 launch my_project_startup startup.launch.py --show-args
```

示例（修改中心节点延迟与串口）：

```bash
ros2 launch my_project_startup startup.launch.py center_start_delay_s:=0.5 port:=/dev/ttyUSB1
```

示例（修改 NPZ 路径与 RSI 监听地址）：

```bash
ros2 launch my_project_startup startup.launch.py npz_path:=/home/jayson/kuka_ram_ws/data/output_npz/demo.npz local_ip:=192.168.1.2 local_port:=49152
```

示例（修改轨迹预填充与 QoS 深度）：

```bash
ros2 launch my_project_startup startup.launch.py traj_prefill:=800 plan_qos_depth:=1500
```
