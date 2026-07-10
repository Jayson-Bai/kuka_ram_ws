# external_npz_preprocessor 总说明

## 包定位

`external_npz_preprocessor` 用于把约定格式的外部源 NPZ 转换成系统可加载的 NPZ。它不直接写系统 NPZ，而是把源 NPZ 中的分层树脂/纤维路径转换成 `path_processing_core.types` 命令序列，再调用 `path_processing_core.npz_exporter.export_npz()` 导出。

这个包是系统未来主要的非 GCode 输入入口。正式打印 UI 已支持兼容输入：选择 `.npz` 时走本包，选择 `.gcode/.gc/.g` 时走 `gcode_planner` 的 GCode 适配路径；两条路径最终都汇入 `path_processing_core`。

## 源 NPZ 格式

源文件是标准 `.npz`，按层和材料分 key：

```text
meta
layer_0000_R
layer_0000_F
layer_0001_R
layer_0001_F
```

每个 `layer_xxxx_R/F` 是 `float32` 三维数组：

```text
[path_count, max_points_per_path, columns]
```

短路径用整行 `NaN` padding。有效点只支持显式带 Z 的格式：

```text
Nx3: [x, y, z]
Nx6: [x, y, z, a, b, c]
```

源 Z 是轨迹几何真值，preprocessor 不生成也不覆盖 Z。UI 中的树脂/纤维层高只作为挤出和工艺参数参考。

## 参数与持久化

UI 中的打印参数可通过 `保存打印参数json文件` 保存到：

```text
data/external_npz_preprocessor/print_params.json
```

启动 UI 时会自动读取该文件。所有路径都基于项目共享 data 根目录推导，不在代码中写死工作区绝对路径。

主要默认值：

- 树脂固定线宽：`2.0 mm`，不允许用户输入
- 树脂耗材直径：固定 `1.75 mm`，用于把沉积体积换算成 E 轴料长
- 树脂层高：`0.5 mm`，用于树脂挤出计算
- 纤维层高：`0.1 mm`，作为工艺参考
- 树脂/纤维打印速度：`10 mm/s`
- 空走速度：`10 mm/s`
- 树脂/纤维温度：`250 C`
- 树脂/纤维风扇：默认常开
- 纤维默认挤出倍率：`1.0`，表示纤维进给速度与 TCP 移动速度一致
- 纤维起步加速时间：`2.0 s`，只覆盖外部 NPZ 纤维打印路径的七阶起始加速段

共享喷头偏置读取自：

```text
data/head_calibration_offsets/head_offsets.json
```

## 导出流程

```text
source NPZ
-> load_source_npz()
-> SourceJob / LayerPaths / MaterialPath
-> source_job_to_parsed_commands()
-> ToolChangeCommand / MCommand / ResetECommand / ExtrudeWait / MoveCommand
-> path_processing_core.npz_exporter.export_npz()
-> system NPZ
```

preprocessor 会在转换开始处插入双喷头风扇/加热事件，在工具切换后插入挤出量重置。整件第一条打印路径前先插入一次初始回抽，再执行预挤出；后续树脂/纤维路径打印前只插入预挤出等待。树脂路径打印完成后只插入回抽等待。纤维路径末端只插入语义级 `CUT` 命令，`CUT` 展开后先做 Z 向安全抬升并同步增加同距离 E，再执行等量剪切安全回抽，然后 travel 到下一条路径起点，最后执行下一条路径自己的预挤出。travel 段本身不插入回抽或预挤出。剪切事件、等待补足、安全回抽、短线段折线连续采样、偏置补偿和 NPZ 字段写入都由 `path_processing_core.npz_exporter` 统一完成。

## 模板与测试

两层 R/F 源 NPZ 模板位于：

```text
data/external_npz_preprocessor/source_npz_templates/two_layer_rf_template.npz
```

该模板使用数值数组和 `NaN` padding，可用 `np.load(..., allow_pickle=False)` 读取，不包含序列化 Python object。
