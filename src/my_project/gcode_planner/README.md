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
- --chunk-size: npz 分片行数，默认 100000


## 输出数据格式（npz）

`gcode_planner/npz_exporter.py` 输出字段：
- seq, x, y, z, a, b, c, e
- tool_id, move_type, src_line
- event_flag, event_type, payload, trigger_seq
- move_type_vocab_keys/move_type_vocab_vals
- event_type_vocab_keys/event_type_vocab_vals

当总行数超过 chunk_size 时，文件名为 `<base>_part0000.npz` 形式；否则为单文件 `<base>.npz`。

## 模块/文件说明

### gcode_planner/
- `gcode_planner/__init__.py`: 包初始化文件
- `gcode_planner/cli.py`: 离线流水线 CLI，组织解析、拟合与导出
- `gcode_planner/gcode_parser.py`: GCode 解析逻辑 + 可选 ROS2 节点封装
- `gcode_planner/npz_exporter.py`: 调用采样与 npz 分片导出；事件映射
- `gcode_planner/bspline_approximation.py`: 全局 B 样条拟合器（角点回退/加密）
- `gcode_planner/polynomial_interpolator.py`: 七阶 S 曲线时间参数化与采样逻辑
- `gcode_planner/types.py`: 数据结构定义（Position/MoveCommand/GlobalCurveCommand 等）

### gcode_planner/bspline/（开源代码）
- `gcode_planner/bspline/__init__.py`: 子模块初始化
- `gcode_planner/bspline/parameter_selection.py`: B 样条参数化与节点向量生成
- `gcode_planner/bspline/BaseFunction.py`: Cox-de Boor 基函数
- `gcode_planner/bspline/bspline_curve.py`: B 样条曲线插值/逼近
- `gcode_planner/bspline/bspline_surface.py`: B 样条曲面插值/逼近/采样


