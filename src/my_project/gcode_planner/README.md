# gcode_planner 使用说明

本包提供一个离线流水线：读取 GCode -> 解析 -> B 样条拟合/采样 -> 导出 npz 分片，供后续 RSI/uart 侧消费。

## 快速开始

在工作空间中构建并 source：

```bash
cd /home/jayson/kuka_ram_ws
colcon build --packages-select gcode_planner
source install/setup.bash
```

离线导出 npz：

```bash
ros2 run gcode_planner gcode_planner_npz \
  --data-root /path/to/kuka_ram_ws/data(注意修改)
```


## CLI 参数（gcode_planner_npz）

参数来自 `gcode_planner/cli.py`：

- --gcode: GCode 文件路径；缺省时从 input_gcode_dir 选择首个 .gcode
- --data-root: 数据根目录；默认是 `kuka_ram_ws/data`
- --input-gcode-dir: GCode 输入目录（未指定 gcode 时使用）；默认 `data_root/input_gcode`
- --output-dir: npz 输出目录；默认 `data_root/output_npz`
- --out: 输出 npz 文件路径（优先级最高）
- --dt: 采样周期秒，默认 0.004（4ms）
- --corner-angle-deg: 角点判定夹角阈值（度），默认 10
- --corner-retreat-ratio: 角点回退比例（0-0.49），默认 0.2
- --density: 数据点加密密度，默认 0
- --degree: B 样条阶次，默认 3
- --max-fit-points-per-segment: 单段拟合点数上限，默认 20000，用于限制 density 过高时的点数膨胀
- --export-sleep-ms: 导出节流休眠毫秒数，默认 0（不节流）
- --export-yield-every: 导出节流触发步长，默认 0（不节流）
- --split-by-layer-type: 按层+打印子类型分别导出 npz（会生成 manifest）
- --plot-layer-xy: 导出后按层生成 XY 路径图（仅 split-by-layer-type 生效）
- --plot-stride: 绘图抽样步长，默认 5

注意：通过 `ros2 run` 运行安装后的包时，默认 `data_root` 会随安装路径变化，可能落到 `install/.../data`。建议显式传 `--data-root` 或 `--output-dir`。

优先级与路径规则：
- --out > --output-dir
- --gcode > --input-gcode-dir > data_root/input_gcode
- 

## GCode 输入路径解析规则

优先级（从高到低）：
1. 显式指定路径（CLI 的 `--gcode` / 节点参数 `gcode_file_path`）
2. 从输入目录选择首个 `.gcode` 文件（按文件名排序，取第一个）

默认路径规则：
- data_root 默认值通过代码推算为 `kuka_ram_ws/data`
- input_gcode_dir 默认值为 `data_root/input_gcode`


## 解析规则（GCode -> ParsedCommand）

基础解析：
- 行内注释以 `;` 截断
- 指令以空格分词
- 参数格式：单字母键 + 数值（例如 `X1.0 Y-2.5 E0.1 F600`）

支持的命令：
- 运动：`G0`/`G1`
- 坐标模式：`G90`（绝对）、`G91`（相对）
- 挤出模式：`M82`（绝对）、`M83`（相对）
- 重置挤出：`G92 E...`
- 换刀：`Tn`
- 其他：`M` 指令作为事件解析入口（见“事件映射”）

运动类型判定：
- `G0` 直接标记为 `TRAVEL`
- `G1` 当 `delta_e != 0` 标记为 `PRINT`，否则 `TRAVEL`
- 纯挤出、无位移的指令会被识别为 `EXTRUDE_WAIT`

默认状态：
- 坐标为绝对（`G90`）
- 挤出为绝对（`M82`）
- 默认进给速度 `F = 100 mm/min`
- 解析器内部初始工具号 `current_tool = 0 (T0)`

## 工具头映射与默认值

内部工具号映射（仅在导出阶段使用）：
- `T0 -> 1`（纤维）
- `T1 -> 2`（树脂）

默认工具头：
- 导出阶段默认工具号为 `2`（树脂 / T1）

## 事件映射（M 指令与换刀）

换刀事件：
- `T0/T1` 触发事件 `tool_change_cf` / `tool_change_resin`

加热事件：
- `M104`/`M109` 且包含 `S` 参数
- 按工具号映射为 `heat_cf` 或 `heat_resin`

风扇事件：
- `M106`：开风扇（payload=1）
- `M107`：关风扇（payload=0）
- 按工具号映射为 `fan_cf` 或 `fan_resin`

重置挤出事件：
- `G92 E...` -> `extrude_reset`（payload 为当前 `tool_id`）

## B 样条拟合逻辑（详细）

位置与实现：`gcode_planner/bspline_approximation.py` 的 `GlobalSplinePlanner.fit_global_curve()`。

整体流程：
1. 分段：在 `npz_exporter.py` 中，连续同类型/同层/同子类型的 `MoveCommand` 会被收集成一个段；段结束时进入拟合。
2. 生成拟合点（角点回退）：  
   - 原始点序列 = 段内所有 Move 的起点/终点。  
   - 去除相邻重复点（距离 < 1e-9）。  
   - 对每个夹角 `>= corner_angle_deg` 的顶点，沿前后两段各插入一个回退点：  
     - 回退距离 = 线段长度 * `corner_retreat_ratio`（限制在 0~0.49）。  
   - 目的：避免角点尖锐拐折带来的拟合震荡。
3. 点密度加密（`density`）：  
   - 每次加密在相邻点之间插入一个中点；点数约变为 `2N-1`。  
   - `density=3` 时，点数约膨胀到原来的 ~8 倍。
4. 控制点数估计：  
   - `n_ctrl = ceil(n_points / 5)`，并保证 `n_ctrl >= degree+1` 且 `n_ctrl < n_points`。  
   - 控制点过多会退化为插值，当前逻辑保证“逼近”模式。
5. 参数化与节点向量：  
   - 使用向心参数化（Centripetal）计算参数 `param`。  
   - 采用均匀内部节点向量：`[0...0, t1, t2, ..., 1...1]`，长度为 `n_ctrl + degree + 1`。
6. 最小二乘逼近：  
   - 使用 `bspline_curve.curve_approximation()` 进行全局最小二乘拟合。  
   - 输入维度为 6（XYZABC），输出控制点序列。
7. 生成 `GlobalCurveCommand`：  
   - `type` 标记为 `TRAVEL_FIT` / `PRINT_FIT`。  
   - `start_pos` 为第一个控制点，`control_points` 为后续控制点。  
   - `delta_e` 为原段内 `MoveCommand.delta_e` 的总和（用于后续采样时的挤出分配）。

关键参数的影响：
- `corner_angle_deg` 越小，角点回退点越多，拟合点数上升。
- `corner_retreat_ratio` 越大，回退点偏离越远，拟合平滑性增强但点数不变。
- `density` 每 +1 会显著放大点数，拟合耗时近似指数增加。
- `max-fit-points-per-segment` 会在单段内限制加密后的拟合点数上限，避免高 density 把容器拖死。
- `degree` 越高，拟合自由度更高但计算更慢。


## 输出数据格式（npz）

`gcode_planner/npz_exporter.py` 输出字段：
- seq, x, y, z, a, b, c, e
- tool_id, move_type, src_line
- event_flag, event_type, payload, trigger_seq
- move_type_vocab_keys/move_type_vocab_vals
- event_type_vocab_keys/event_type_vocab_vals

默认输出结构为：

```
output_npz/<base_name>/
  <base_name>.npz
  <base_name>.offset.json
  layer_previews/
    layer_0000.png
    ...
```

当总行数超过 5000000 时，NPZ 文件名为 `<base>_part0000.npz` 形式；否则为单文件 `<base>.npz`。

### 按层+子类型导出

启用 `--split-by-layer-type` 后，输出结构如下：

```
output_npz/<base_name>/
  <base_name>_manifest.json
  <base_name>.offset.json
  layer_0001/
    <base_name>_layer_0001_type_<TYPE>_occ_0001.npz
    ...
  layer_0002/
    ...
```

说明：
- `TYPE` 来自 `;TYPE:` 注释；`UNKNOWN` 会归并为 `TRAVEL`。
- `occ` 为同层同类型的出现序号，保证顺序可复现。
- manifest 用于按原 GCode 顺序播放（基于解析顺序）。
- 启用 `--plot-layer-xy` 后，会在 `layer_previews/` 下生成总览预览图，或在分层分类型导出时生成同级总览图（只绘制真实沉积路径）。

## 挤出量 E 的生成与发送路径（npz -> UART）

下面以当前 `data/output_npz/test3.npz` 为例（包含 `e` 字段，长度 17518，`event_flag` 全 0），说明 E 的来源与发送链路。  
E 在系统内始终以“绝对挤出量（absolute E）”流转。

### 1) gcode_planner 如何生成 E

- GCode 解析阶段（`gcode_planner/gcode_parser.py`）  
  - 支持绝对/相对挤出：`M82` 绝对，`M83` 相对。  
  - `G0/G1` 解析时：  
    - 绝对模式：`target_e = params["E"] 或 state.e`，`delta_e = target_e - state.e`，并更新 `state.e = target_e`。  
    - 相对模式：`delta_e = params.get("E", 0)`，`state.e += delta_e`。  
  - 每条运动生成 `MoveCommand`，记录 `e_val`（当前绝对 E）与 `delta_e`（本段增量）。  
  - `G92 E...` 生成 `ResetECommand` 事件（后续导出为 `extrude_reset`，payload 为当前 `tool_id`）。

- 时间参数化/采样阶段（`gcode_planner/polynomial_interpolator.py`）  
  - 对拟合后的曲线进行 4ms 采样。  
  - 挤出按弧长比例分配：  
    - 起始 E：`start_e = curve.e_val - curve.delta_e`  
    - 每个采样点：`delta_e = curve.delta_e * (delta_s / total_length)`，`current_e += delta_e`  
  - 输出 `InterpolatedPoint.e`（绝对 E），确保整段的 E 总量与原始 GCode 一致。

- npz 导出阶段（`gcode_planner/npz_exporter.py`）  
  - 采样点写入 `CsvRow.e`，最终保存为 npz 字段 `e`（float32）。  
  - `G92 E...` 会导出 `event_type=extrude_reset` 事件行（`event_flag=1`，payload 为当前 `tool_id`），用于串口侧状态同步。

### 2) E 从 npz 到 UART 的发送链路

1. `control_center/NpzLoader` 读取 `e` 数组（`control_center/src/npz_loader.cpp`）。  
2. `QueueManager` 将 `row.e` 填入 `TrajectoryPoint.e`（`control_center/src/queue_manager.cpp`）。  
3. `center_node` 对 `TrajectoryPoint.e` 按 `e_decimals` 四舍五入（默认 6 位），发布到 `/planned_trajectory`（`control_center/src/center_node.cpp`）。  
4. `rsi_node` 在每个 KUKA 心跳周期选择 `TrajectoryPoint`，并发布 `/rsi/heartbeat`：  
   - `hb.seq_used = to_send.seq`  
   - `hb.tool_id = to_send.tool_id`  
   - `hb.extrude_abs = to_send.e`  
   （`rsi_server/src/rsi_node.cpp`）  
5. `uart_node` 订阅 `/rsi/heartbeat`，将 E 以 UART 文本协议发送：  
   - 格式：`E <seq_used> <tool_id> <extrude_abs>\\n`  
   - 发送函数：`send_extrude_command()`（`uart_bridge/src/uart_node.cpp`）

因此，“通过 UART 发送出去的挤出量 E”就是从 npz 中 `e` 列读取、在中心节点做精度裁剪后，经 RSI 心跳携带的 **绝对挤出量**，最终由 UART 节点按 `E seq tool E` 文本格式写到串口。

## 模块/文件说明

### gcode_planner/
- `gcode_planner/__init__.py`: 包初始化文件
- `gcode_planner/cli.py`: 离线流水线 CLI，组织解析、拟合与导出
- `gcode_planner/gcode_parser.py`: GCode 解析逻辑 + 可选 ROS2 节点封装
- `gcode_planner/npz_exporter.py`: 调用采样与 npz 分片导出；事件映射
- `gcode_planner/bspline_approximation.py`: 全局 B 样条拟合器（角点回退/加密）
- `gcode_planner/polynomial_interpolator.py`: 七阶 S 曲线时间参数化与采样逻辑
- `gcode_planner/types.py`: 数据结构定义（Position/MoveCommand/GlobalCurveCommand 等）

### gcode_planner/bspline/（开源B样条拟合代码）
- `gcode_planner/bspline/__init__.py`: 子模块初始化
- `gcode_planner/bspline/parameter_selection.py`: B 样条参数化与节点向量生成
- `gcode_planner/bspline/BaseFunction.py`: Cox-de Boor 基函数
- `gcode_planner/bspline/bspline_curve.py`: B 样条曲线插值/逼近
- `gcode_planner/bspline/bspline_surface.py`: B 样条曲面插值/逼近/采样
