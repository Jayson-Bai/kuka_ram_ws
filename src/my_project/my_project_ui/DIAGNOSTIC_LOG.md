# 手动导出诊断日志

沿用 rqt 的 **导出诊断日志** 按钮及 `_DEFAULT_DATA_ROOT/diagnostic_logs` 目录。
只在点击按钮时生成 `diagnostic_log_<时间含纳秒>.jsonl.gz`。打印结束不自动导出，
不需要额外启动采集进程。导出不会清空历史，再次导出包含此前记录和新增记录。

## 精简与保留

- 诊断记录在内存中以约64 KiB未压缩数据为块，用gzip level 1压缩保存。
- 删除可由time_epoch还原的显示时间字符串；UART保留完整line，省略重复的
  direction/payload。JSON使用紧凑分隔符。其他字段保留，不对时间序列降采样。
- 增加host_monotonic_ns用于同一主机的时间间隔分析，时间点仍是UI回调观测。
- 不设128 MiB停采线、不设20万条裁剪、不覆盖前段记录。打印测试准备只加
  session_boundary标记，不再清空诊断历史。
- UART“放大查看”的显示历史仅保留最近2000行；这不影响完整诊断导出。
- 原有UI消息接收频率、ROS订阅、UART协议和固件均保持原样。无法补回到达UI前
  已丢失的消息，也不把UI观测数据宣称为4 ms完整硬实时测量。

用 `gzip -dc diagnostic_log_*.jsonl.gz` 或 Python `gzip.open(path, 'rt')` 读取；
文件是标准gzip多成员流，解压后为JSONL。旧分析工具需改用gzip读取，并从line解析
UART方向与payload。export_metadata记录版本和字段口径。

## 边界

压缩能显著减小重复日志，不能让无限时长数据占用固定内存。缓存大小随运行时间增长，
本方案优先保留完整诊断过程。只在按钮导出时落盘，因此rqt进程被杀死、崩溃或断电
会失去尚未导出的内存记录；运行中的故障若UI仍可操作，可立即点击导出。
不把缓存停止/自动导出隐藏在打印结束流程里。
