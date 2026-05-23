# KUKA 机器人增材制造 (KUKA RAM) 系统

本项目是一个基于 ROS 2 的软硬件协同控制系统，旨在利用 KUKA 工业机械臂完成 3D 打印（增材制造）任务。

## 整体流程与系统架构思路

整个系统分为 **离线预处理阶段** 和 **在线实时控制阶段**，通过 ROS 2 节点解耦了规划与执行。

### 1. 离线预处理阶段 (Offline Planning)
**核心模块：** `gcode_planner`
- **输入**：标准的 3D 打印切片软件（如 Cura、PrusaSlicer）生成的 `.gcode` 文件。
- **解析与重构**：将 GCode 指令解析为内部表示，识别打印、空走、换刀、风扇等事件。
- **B 样条曲线拟合**：为了防止机械臂在走直角或尖锐拐角时产生剧烈震荡，系统对 GCode 的直线段路径进行全局 B 样条拟合，并在角点处实施“回退倒角”策略，生成平滑的连续曲线。
- **恒定周期采样**：按照 KUKA 机器人的控制心跳周期（默认 4ms）对平滑曲线进行等时间步采样，将总挤出量按路径比例分配到各个离散点上。
- **输出**：生成高密度的轨迹数据点集合，并保存为 `.npz`（NumPy 压缩）文件或按层/类型的 Manifest 序列。这些数据包含了时间步序列 (seq)、位姿 (X/Y/Z/A/B/C)、绝对挤出量 (E) 以及事件标志。

### 2. 在线实时控制阶段 (Online Real-time Control)
**核心模块：** `control_center`, `rsi_server`, `uart_bridge`, `system_manager_node`

一旦进入打印流程，数据流向如下：

1. **轨迹加载与分发 (`control_center` -> `center_node`)**：
   - 节点读取预生成的 `.npz` 轨迹文件，将其放入内存队列中。
   - 作为一个“数据泵”，它负责将轨迹点以固定的频率和 QoS 策略发布到 ROS 2 的内部话题（如 `/planned_trajectory`），供下游节点消费。

2. **机械臂实时控制 (`rsi_server` -> `rsi_node`)**：
   - 采用 KUKA RSI (Robot Sensor Interface) 协议与机械臂控制柜建立硬实时以太网通信。
   - 在每一个 KUKA 传来的心跳周期（如 4ms）内，`rsi_node` 会从内部话题中取出最新的轨迹点（位姿 X/Y/Z/A/B/C）。
   - 同时将当前发出的轨迹序号（seq）和挤出量（E）封装进 `/rsi/heartbeat` 话题发布给系统，以实现全系统的时钟对齐。

3. **挤出机同步控制 (`uart_bridge` -> `uart_node`)**：
   - 订阅 `/rsi/heartbeat` 话题，获取当前机械臂真正执行到的序号和对应的挤出量。
   - 将挤出量 (E) 通过 UART 串口协议（如 `E <seq> <tool_id> <extrude_abs>`）发送给底层挤出机主板（如 Marlin/Klipper 改装板），确保机械臂运动与耗材挤出在微秒级别保持严格同步。

4. **状态监控与 UI (`system_manager_node` & UI)**：
   - 汇总 RSI 的实际执行状态、队列堆积情况等，发布给前端 UI，以便操作员实时监控打印进度。

## 各个子包的作用总结

- `gcode_planner`：离线路径平滑与生成（Python）。
- `control_center`：在线数据中心与轨迹队列管理器（C++）。
- `rsi_server`：KUKA 机械臂 RSI 接口通信服务端（C++）。
- `uart_bridge`：挤出机串口通信与挤出指令下发（C++）。
- `my_project_startup`：ROS 2 启动文件集合，管理节点拉起顺序与参数配置。
- `my_project_interfaces`：自定义 ROS 2 消息和服务定义。
- `third_party/`：KUKA 相关的依赖包与 URDF 描述。
- `scripts/`：数据绘图、离线调试与模拟测试脚本。

## 数据流向总结 (Data Flow)
```text
[离线 GCode] 
    | (gcode_planner)
    v
[平滑的 .npz 轨迹阵列: XYZABC, E, Seq]
    | (ROS 2 Launch 加载)
    v
[control_center] --(发布轨迹话题)--> [rsi_server] --(RSI UDP)--> [KUKA 机械臂]
                                       |
                               (发布心跳同步话题包含 E)
                                       |
                                       v
                                 [uart_bridge] --(串口 UART)--> [挤出机控制板]
```
