# 上位机 UART 挤出 canonical_v1 固件确认说明

## 目的

本文供固件侧确认当前上位机已经实现的 UART 挤出行为。第一阶段旧 ASCII `E` 命令语法保持不变；本次只规范数值文本、去重键和串口写成功后的状态提交边界，不新增命令、ACK 或重放协议。

## 线上协议

线上命令保持为：

```text
E <seq_used> <tool_id> <absolute_E_fixed6>\n
```

`absolute_E_fixed6` 是应用 `extrude_scale` 后的绝对 E，单位为 mm。字段顺序、字段间单个空格和行尾 LF 均保持不变；上式和下列示例中的 `\n` 表示一个 LF 字节，不发送反斜杠字符。

```text
E 1842 0 1.200000\n
E 1843 1 0.000001\n
E 1844 1 -0.250000\n
```

## 4 ms 实时流程

KUKA RSI 心跳约每 4 ms 触发一次挤出检查。处理顺序如下：

1. 保留 PAUSE、ABORT、pending event 三项既有门控；门控不允许发送时直接结束本次检查。
2. 允许发送时计算 `scaled_E = heartbeat.extrude_abs * extrude_scale`。
3. 按当前 wire mode 做量化、格式化和去重判定，生成候选命令；此时不更新已提交去重状态。
4. 对候选命令执行同步 UART 写入。
5. 仅当写入无 error 且完整写入全部字节时，才 commit 该候选项。

相同去重键对应的重复值不会发送，因此 UART 是稀疏绝对 E 流，不保证每个 4 ms 心跳都有一条 `E` 命令。

## 默认模式 canonical_v1

`canonical_v1` 是默认启动模式，规则如下：

- 使用 `e_nm = round(scaled_E * 1,000,000)` 将 E 量化为 `10^-6 mm` 整数单位，并以 `int64_t` 保存；线上值固定保留 6 位小数，不使用科学计数法。
- 已提交去重键为 `(tool_id, e_nm)`。同一工具、同一量化值只发送一次。
- 尚无已提交 E 时，量化后的初始 `0` 继续省略，且不建立去重状态。
- 工具变化时，即使 E 相同也会发送；已发送非零 E 后回到 `0` 会发送 `0.000000`。
- 非有限值以及量化后超出 `int64_t` 范围的值会被拒绝，不发送，也不污染已提交去重状态。
- UART 节点参数 `extrusion_wire_mode` 是只读启动参数；未知模式会在节点启动时被拒绝。

## 写失败语义

同步写返回无 error 且 `written == line.size()` 才视为成功并 commit；错误或短写都不 commit。

写失败后，上位机不重放失败时的旧 heartbeat。下一次通过门控的有效心跳会按当时最新的绝对 E、`tool_id` 和 `seq_used` 重新量化、去重并决定是否发送。现有协议没有 E 命令 ACK 闭环，上位机不能据此证明 MCU 已接收或执行。

## legacy_v1 回退

`legacy_v1` 恢复旧行为：缩放结果转为 `float` 后使用 default-stream 文本格式；去重仍使用缩放后的 `double`，并以 `epsilon = 1e-9` 比较，同一工具且差值不超过 epsilon 时视为重复。它与 `canonical_v1` 一样，仅在同步串口完整写成功后 commit。

使用仓库实际启动包和 launch 文件的完整回退命令为：

```bash
ros2 launch my_project_startup startup.launch.py extrusion_wire_mode:=legacy_v1
```

模式只能在启动时选择。`extrusion_wire_mode` 运行时只读，因此切换到 `legacy_v1` 或切回 `canonical_v1` 都必须重启 UART 节点；不能运行时动态修改。

## 未修改链路

本次实现未修改 NPZ、控制中心、RSI、4 ms 主循环、PAUSE/ABORT、事件、`extrude_reset` reset 事件语义、UI 或消息接口。`extrude_scale` 的含义和动态调整能力保持不变；UART 节点仍允许在运行时设置有限且大于 0 的 `extrude_scale`。

## 已接受的系统边界

1. 若最后一条 E 永久丢失且之后没有任何新 E，固件无法从现有单向稀疏协议推断该丢失。
2. 在 UART 文本边界之前已经丢失的精度无法恢复，例如 NPZ `float32` E 的量化误差。
3. 没有编码器反馈时，上位机和 MCU 都不能确认物理失步或打滑。
4. 长时间保持 EN 的温升和电流需要固件/硬件侧通过实机确认。

## 固件联调建议

以下项目是建议的固件/硬件联调覆盖项，不代表本次已经完成 MCU 实机验证：

1. canonical 正常发送：确认 fixed6 文本、绝对 mm 语义、字段顺序和 LF。
2. 重复抑制：同一工具、同一量化 E 不产生重复行，固件状态保持正确。
3. 工具切换：E 相同时切换 `tool_id` 仍接收新行。
4. 回零：非零 E 后回到零时接收 `0.000000`。
5. 非法值：非有限值和超范围值不进入 UART，也不影响下一条合法 E。
6. 断线后恢复：制造写失败后恢复串口，确认下一有效心跳携带当时最新绝对 E 和 `seq_used`，且没有旧 heartbeat 重放假设。
7. legacy 回退：重启到 `legacy_v1`，确认旧 float/default-stream 文本和 epsilon 去重兼容性，再重启切回默认模式。

## 本次上位机验证

验证在隔离工作树 `/home/jayson/kuka_ram_ws/.worktrees/uart-canonical-v1`、ROS Humble 环境中执行。成功执行的命令和真实结果如下：

```bash
/bin/bash -lc 'source /opt/ros/humble/setup.bash && colcon build --packages-select uart_bridge my_project_startup --symlink-install'
```

结果：退出码 0，`uart_bridge`、`my_project_startup` 共 2 个包完成构建；未使用 `--packages-up-to`。

```bash
/bin/bash -lc 'source /opt/ros/humble/setup.bash && colcon test --packages-select uart_bridge my_project_startup --event-handlers console_direct+'
```

结果：退出码 0，2 个包完成测试；`uart_bridge` 的 CTest 为 10/10 通过，其中 extrusion wire gtest 为 19/19 通过。`my_project_startup` 为 2 passed、1 skipped，并出现 2 条 `SelectableGroups` 弃用 warning；没有测试失败。

```bash
/bin/bash -lc 'source /opt/ros/humble/setup.bash && colcon test-result --verbose'
```

结果：退出码 0，`Summary: 59 tests, 0 errors, 0 failures, 5 skipped`。

```bash
python3 -m pytest -q src/my_project/uart_bridge/test src/my_project/control_center/test/test_staged_pause_contract.py
```

结果：退出码 0，`22 passed in 0.02s`。读取 `control_center` 契约测试不代表修改该包。

```bash
python3 -m py_compile src/my_project/my_project_startup/launch/startup.launch.py
```

结果：退出码 0，无输出。

```bash
git diff --check
git diff --name-status e9d2b28..HEAD
git diff --exit-code e9d2b28..HEAD -- \
  src/my_project/rsi_server \
  src/my_project/control_center \
  src/my_project/path_processing_core \
  src/my_project/my_project_interfaces \
  src/my_project/my_project_ui
```

结果：空白检查退出码 0；实施范围只包含 `src/my_project/uart_bridge/` 和 `src/my_project/my_project_startup/launch/startup.launch.py`，禁止路径 diff 退出码 0 且无输出。本交接文档是验证后新增的唯一文档文件。

以上均为上位机侧构建、单元测试、契约测试、语法检查和 Git 范围审计；没有执行 MCU、驱动器或整机串口实机验证。

## 固件确认清单

- [ ] 固件继续按旧语法解析 `E <seq_used> <tool_id> <absolute_E>\n`，字段顺序和 LF 不变。
- [ ] 固件接受固定 6 位小数文本，包括 `0.000000`。
- [ ] 固件将 E 解释为绝对位置，单位 mm，而不是每周期增量。
- [ ] 固件允许 E 行因上位机去重而省略，不以缺行表示零或停止。
- [ ] 固件不依赖每 4 ms 必有一条 E。
- [ ] 固件不假设上位机会等待 ACK 或重放失败时的旧 heartbeat。
