# External NPZ Preprocessor 处理流程

## 1. 总体链路

外部源 NPZ 不直接写成本系统 NPZ。完整链路固定为：

```text
外部源 NPZ
-> load_source_npz()
-> SourceJob / LayerPaths / MaterialPath
-> source_job_to_parsed_commands()
-> path_processing_core.types 命令序列
-> path_processing_core.npz_exporter.export_npz()
-> 系统可用 NPZ
```

核心原则：最终系统 NPZ 必须通过 `npz_exporter.export_npz()` 写出，preprocessor 只负责把外部路径转换成 exporter 已支持的命令序列。

## 2. 源 NPZ 读取

源文件是标准 `.npz`，按层和材料分 key：

```text
meta
layer_0000_R
layer_0000_F
layer_0001_R
layer_0001_F
```

`R` 是树脂，`F` 是纤维。每个 `layer_xxxx_R/F` 是数值型 `float32` 三维数组：

```text
[path_count, max_points_per_path, columns]
```

短路径用整行 `NaN` padding。有效点只支持：

```text
Nx3: [x, y, z]
Nx6: [x, y, z, a, b, c]
```

`Nx3` 会补默认 `ABC` 成 `Nx6`。源 NPZ 必须包含 Z；UI 中的树脂/纤维层高不会生成或覆盖路径 Z。这样曲面切片软件产生的曲面高度会被保留。

## 3. 路径顺序

转换时按层顺序处理。每一层内目前先处理该层所有树脂路径，再处理该层所有纤维路径：

```text
layer.resin_paths
layer.fiber_paths
```

每条 R/F 路径都会独立形成一段打印流程。路径之间如果当前位置和下一条路径起点不同，会插入空走 `MoveCommand(type="TRAVEL")`。

## 4. 工具切换事件

材料和 GCode 工具号映射为：

```text
树脂 R -> gcode tool 1 -> exporter 内映射为系统树脂工具
纤维 F -> gcode tool 0 -> exporter 内映射为系统纤维工具
```

当下一条路径的材料工具和当前工具不同时，preprocessor 先插入：

```text
ToolChangeCommand(type="TOOL_CHANGE", tool=...)
```

`npz_exporter` 接收这个命令后，会在系统 NPZ 中生成工具切换事件：

```text
tool_change_cf
tool_change_resin
```

实际工具偏置补偿不在 preprocessor 中重复实现，而是通过 `export_npz()` 的 `tool_offset` 参数交给 exporter 处理。

## 5. 加热和风扇事件

每次工具切换后，preprocessor 会插入当前材料的工艺事件：

```text
M104 T<tool> S<temperature>
M106 T<tool> 或 M107 T<tool>
```

具体规则：

- 温度 `temperature_c > 0` 时插入 `M104`。
- 风扇开启时插入 `M106`，关闭时插入 `M107`。
- 树脂默认温度 `250 C`，纤维默认温度 `250 C`。
- 树脂/纤维风扇默认开启。

`npz_exporter` 会把这些 `MCommand` 转成系统 NPZ 事件：

```text
heat_resin
heat_cf
fan_resin
fan_cf
```

事件 payload 中保存温度或风扇开关状态。

## 6. 挤出量重置

每条独立路径打印前，preprocessor 都会插入：

```text
ResetECommand(type="RESET_E", val=0.0, raw="G92 E0")
```

这会让每条路径从局部 `E=0` 开始累计。`npz_exporter` 会把它写成系统 NPZ 事件：

```text
extrude_reset
```

## 7. 预挤出和回抽

预挤出/回抽不伪装成移动路径，而是插入 `ExtrudeWait`，表示原地挤出或原地回抽。导出时必须启用：

```text
enable_extrude_wait=True
```

### 树脂路径

树脂路径打印前插入预挤出：

```text
ExtrudeWait(delta_e=+resin.prime_length_mm, feedrate=resin.prime_speed_mm_s * 60)
```

树脂路径打印后插入回抽：

```text
ExtrudeWait(delta_e=-resin.retract_length_mm, feedrate=resin.retract_speed_mm_s * 60)
```

当前默认值：

```text
树脂预挤出: 18 mm @ 15 mm/s
树脂回抽:   15 mm @ 30 mm/s
```

### 纤维路径

纤维路径打印前先插入预回抽，再插入预挤出：

```text
ExtrudeWait(delta_e=-fiber.retract_length_mm, feedrate=fiber.retract_speed_mm_s * 60)
ExtrudeWait(delta_e=+fiber.prime_length_mm,   feedrate=fiber.prime_speed_mm_s * 60)
```

当前默认值：

```text
纤维预回抽: 10 mm @ 5 mm/s
纤维预挤出: 12 mm @ 5 mm/s
```

纤维路径打印后目前不额外插入后置回抽。

## 8. 打印路径和挤出量计算

每条路径从第一个点开始，逐段生成 `MoveCommand(type="PRINT")`：

```text
start_pos = 上一点
pos       = 下一点
feedrate  = 材料打印速度 * 60
delta_e   = 三维路径长度 * e_per_mm
e_val     = 当前路径内累计 E
```

路径长度使用三维距离：

```text
sqrt(dx^2 + dy^2 + dz^2)
```

所以曲面路径中的 Z 起伏会参与路径长度和挤出量计算。

树脂 `E/mm`：

```text
resin_e_per_mm = 2.0 * resin_layer_height_mm * resin_extrusion_scale
```

这里 `2.0 mm` 是固定树脂线宽，不允许用户输入。树脂层高只作为挤出计算参考，不改变源 Z。

纤维 `E/mm`：

```text
fiber_e_per_mm = fiber_extrusion_scale
```

默认 `fiber_extrusion_scale = 1.0`，表示纤维挤出速度和 TCP 移动速度一致。纤维层高目前是工艺参考参数，不改变源 Z，也不改变默认纤维 E。

## 9. 空走路径

如果上一条路径终点和下一条路径起点不同，preprocessor 插入：

```text
MoveCommand(type="TRAVEL", cmd="G0")
```

空走速度来自：

```text
travel_feed_mm_s
```

默认是 `10 mm/s`。空走不产生挤出：

```text
delta_e = 0
e_val = 0
```

## 10. 偏置和最终导出

`convert_external_npz()` 在导出前读取共享偏置数据：

```text
data/head_calibration_offsets/head_offsets.json
```

读取后传给 exporter：

```text
tool_offset = (fiber_x_print_compensation_mm,
               fiber_y_print_compensation_mm,
               fiber_z_print_compensation_mm)

resin_z_print_compensation_mm = resin_z_print_compensation_mm
```

最终调用：

```text
export_npz(
    commands,
    output_path,
    dt=params.dt,
    default_feed_mm_s=params.travel_feed_mm_s,
    corner_angle_deg=params.corner_angle_deg,
    corner_retreat_ratio=params.corner_retreat_ratio,
    density=params.density,
    degree=params.degree,
    max_fit_points_per_segment=params.max_fit_points_per_segment,
    enable_extrude_wait=True,
    tool_offset=tool_offset,
    resin_z_print_compensation_mm=resin_z_print_compensation_mm,
)
```

因此，工具切换补偿、树脂 Z 补偿、事件编码、采样和系统 NPZ 字段写入，都由现有 `npz_exporter` 统一完成。

## 11. 默认路径

源模板目录：

```text
data/external_npz_preprocessor/source_npz_templates
```

输出路径为空时默认导出到：

```text
data/output_npz/<source_stem>/<source_stem>.npz
```

打印参数 JSON 保存到：

```text
data/external_npz_preprocessor/print_params.json
```

这些路径都基于项目共享的 `DEFAULT_DATA_ROOT` 推导，不在代码中写死工作区绝对路径。
