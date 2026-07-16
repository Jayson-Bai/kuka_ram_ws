# UART canonical_v1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改 RSI、控制中心、NPZ、消息接口和 UI 的前提下，为旧 ASCII UART E 命令增加默认 `canonical_v1` 固定六位边界、成功写入后提交去重状态，以及 `legacy_v1` 启动回退。

**Architecture:** 在 `uart_bridge` 内新增不依赖 ROS/Boost 的 `ExtrusionForwarder` 纯 C++ 组件，以 `prepare -> write -> commit` 分离确定性文本、去重状态和串口 I/O。`uart_node` 保留全部既有门控与事件逻辑，只在心跳出口接入组件并让同步写函数返回成功状态；`startup.launch.py` 只声明和透传 UART 模式参数。

**Tech Stack:** C++17、ROS 2 `rclcpp`、Boost.Asio、CMake/ament、GoogleTest、pytest、colcon。

---

## 文件结构

- Create: `src/my_project/uart_bridge/include/uart_bridge/extrusion_wire.hpp`
  - 声明模式、准备结果、候选命令和纯状态机接口。
- Create: `src/my_project/uart_bridge/src/extrusion_wire.cpp`
  - 实现 fixed6 整数量化/格式化、legacy 文本、去重和提交。
- Create: `src/my_project/uart_bridge/test/test_extrusion_wire.cpp`
  - 对纯组件做真实 C++ 单元测试，不模拟 ROS 或串口。
- Create: `src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py`
  - 验证节点接线、发送事务顺序、日志兼容和启动参数。
- Modify: `src/my_project/uart_bridge/src/uart_node.cpp`
  - 接入边界组件；保持心跳门控；让 `write_line()` 返回成功状态。
- Modify: `src/my_project/uart_bridge/test/test_extrude_reset_handshake.py`
  - 把旧内部成员断言更新为边界组件和 reset 接口断言。
- Modify: `src/my_project/uart_bridge/CMakeLists.txt`
  - 构建静态纯组件，链接节点，注册 gtest/pytest。
- Modify: `src/my_project/uart_bridge/package.xml`
  - 增加 `ament_cmake_gtest` 与 `ament_cmake_pytest` 测试依赖。
- Modify: `src/my_project/my_project_startup/launch/startup.launch.py`
  - 声明并向 UART 节点透传 `extrusion_wire_mode`，不碰 RSI 参数块。
- Create: `docs/firmware_uart_extrusion_canonical_v1_handoff.md`
  - 面向固件侧总结最终线上行为、回退方式和已接受边界。

明确不修改：

- `src/my_project/rsi_server/**`
- `src/my_project/control_center/**`
- `src/my_project/path_processing_core/**`
- `src/my_project/my_project_interfaces/**`
- `src/my_project/my_project_ui/**`

### Task 0: 建立隔离工作区和基线

**Files:** None

- [ ] **Step 1: 从设计提交创建隔离分支/worktree**

在用户同意后，按 `using-git-worktrees` 选择仓库已有目录；若没有则使用已确认被
Git 忽略的 `.worktrees/uart-canonical-v1`，分支名使用
`feature/uart-canonical-v1`。

- [ ] **Step 2: 确认基线 Git 状态**

Run:

```bash
git status --short --branch
git log -1 --oneline
```

Expected: 工作树无修改，HEAD 包含设计提交
`90b868c docs: design canonical UART extrusion boundary`。

- [ ] **Step 3: 运行现有 UART 源码契约测试**

Run:

```bash
python3 -m pytest -q \
  src/my_project/uart_bridge/test/test_extrude_reset_handshake.py \
  src/my_project/uart_bridge/test/test_temperature_ready_threshold.py \
  src/my_project/control_center/test/test_staged_pause_contract.py
```

Expected: PASS。若失败，停止实现并报告基线失败，不把它误判为本次回归。

- [ ] **Step 4: 构建 UART 基线**

Run:

```bash
colcon build --packages-select uart_bridge --symlink-install
```

Expected: `uart_bridge` 构建成功。

### Task 1: 用测试定义 canonical fixed6 API

**Files:**
- Create: `src/my_project/uart_bridge/test/test_extrusion_wire.cpp`
- Modify: `src/my_project/uart_bridge/CMakeLists.txt`
- Modify: `src/my_project/uart_bridge/package.xml`
- Create: `src/my_project/uart_bridge/include/uart_bridge/extrusion_wire.hpp`
- Create: `src/my_project/uart_bridge/src/extrusion_wire.cpp`

- [ ] **Step 1: 添加首组失败的 C++ 测试**

测试先引用尚不存在的接口，并定义最小期望：

```cpp
#include <gtest/gtest.h>

#include <locale>
#include <stdexcept>
#include <string>

#include "uart_bridge/extrusion_wire.hpp"

using uart_bridge::ExtrusionDecision;
using uart_bridge::ExtrusionForwarder;
using uart_bridge::ExtrusionWireMode;

TEST(ExtrusionWireModeTest, ParsesOnlyNamedModes)
{
  EXPECT_EQ(
    uart_bridge::parse_extrusion_wire_mode("canonical_v1"),
    ExtrusionWireMode::CanonicalV1);
  EXPECT_EQ(
    uart_bridge::parse_extrusion_wire_mode("legacy_v1"),
    ExtrusionWireMode::LegacyV1);
  EXPECT_THROW(
    uart_bridge::parse_extrusion_wire_mode("unknown"),
    std::invalid_argument);
}

TEST(CanonicalExtrusionWireTest, FormatsOldSyntaxWithFixedSixDecimals)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto prepared = forwarder.prepare(42U, 3, 1.2);

  ASSERT_EQ(prepared.decision, ExtrusionDecision::Send);
  ASSERT_TRUE(prepared.candidate.has_value());
  EXPECT_EQ(prepared.candidate->line, "E 42 3 1.200000\n");
}

TEST(CanonicalExtrusionWireTest, NormalizesNegativeZeroAndAvoidsScientificNotation)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto zero = forwarder.prepare(1U, 2, -0.0000004);
  const auto large = forwarder.prepare(2U, 2, 1234567.25);

  EXPECT_EQ(zero.decision, ExtrusionDecision::SuppressInitialZero);
  ASSERT_EQ(large.decision, ExtrusionDecision::Send);
  EXPECT_EQ(large.candidate->line, "E 2 2 1234567.250000\n");
}

class CommaDecimalPoint : public std::numpunct<char>
{
protected:
  char do_decimal_point() const override {return ',';}
  char do_thousands_sep() const override {return '_';}
  std::string do_grouping() const override {return "\3";}
};

TEST(CanonicalExtrusionWireTest, IsIndependentOfGlobalLocale)
{
  const std::locale previous = std::locale();
  std::locale::global(std::locale(previous, new CommaDecimalPoint));
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto prepared = forwarder.prepare(1234U, 2, 12.5);
  std::locale::global(previous);

  ASSERT_EQ(prepared.decision, ExtrusionDecision::Send);
  EXPECT_EQ(prepared.candidate->line, "E 1234 2 12.500000\n");
}

TEST(CanonicalExtrusionWireTest, RoundsAtSixDecimalBoundary)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  EXPECT_EQ(
    forwarder.prepare(1U, 1, 0.00000049).decision,
    ExtrusionDecision::SuppressInitialZero);
  const auto rounded = forwarder.prepare(2U, 1, 0.00000051);
  ASSERT_EQ(rounded.decision, ExtrusionDecision::Send);
  EXPECT_EQ(rounded.candidate->line, "E 2 1 0.000001\n");
}
```

在 `BUILD_TESTING` 中先注册 `ament_cmake_gtest` 测试，并在 `package.xml` 增加：

```xml
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>ament_cmake_pytest</test_depend>
```

- [ ] **Step 2: 运行测试确认 RED**

Run:

```bash
colcon build --packages-select uart_bridge --symlink-install
```

Expected: FAIL，明确报告 `uart_bridge/extrusion_wire.hpp` 不存在，而不是测试拼写错误。

- [ ] **Step 3: 添加最小纯组件接口与 canonical 格式化实现**

头文件定义以下稳定接口：

```cpp
namespace uart_bridge
{
enum class ExtrusionWireMode {CanonicalV1, LegacyV1};
enum class ExtrusionDecision {
  Send,
  SuppressInitialZero,
  SuppressDuplicate,
  RejectNonFinite,
  RejectOutOfRange
};

struct ExtrusionCandidate
{
  std::string line;
  int32_t tool_id{0};
  int64_t canonical_e_nm{0};
  double scaled_e_abs{0.0};
};

struct ExtrusionPreparation
{
  ExtrusionDecision decision{ExtrusionDecision::SuppressDuplicate};
  std::optional<ExtrusionCandidate> candidate;
};

ExtrusionWireMode parse_extrusion_wire_mode(const std::string & value);
const char * extrusion_wire_mode_name(ExtrusionWireMode mode) noexcept;

class ExtrusionForwarder
{
public:
  explicit ExtrusionForwarder(ExtrusionWireMode mode) noexcept;
  ExtrusionPreparation prepare(
    uint32_t seq_used, int32_t tool_id, double scaled_e_abs) const;
  void commit(const ExtrusionCandidate & candidate) noexcept;
  void reset() noexcept;
  ExtrusionWireMode mode() const noexcept;

private:
  ExtrusionWireMode mode_;
  bool last_sent_valid_{false};
  int32_t last_sent_tool_id_{0};
  int64_t last_sent_canonical_e_nm_{0};
  double last_sent_legacy_e_abs_{0.0};
};
}  // namespace uart_bridge
```

实现使用 `long double` 乘 `1'000'000.0L` 后 `std::round`，再安全转换为
`int64_t`；文本从整数的整数部和六位余数生成，并对流显式使用
`std::locale::classic()`。此步骤只实现首组测试需要的 canonical 路径；
`commit/reset` 保持最小定义，后续任务以新失败测试驱动状态逻辑。

在 CMake 中创建并链接纯静态库：

```cmake
add_library(uart_extrusion_wire STATIC src/extrusion_wire.cpp)
target_include_directories(uart_extrusion_wire PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(uart_extrusion_wire PUBLIC cxx_std_17)

target_link_libraries(uart_node uart_extrusion_wire Boost::system)
```

测试目标和既有 Python 测试按以下方式注册：

```cmake
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
ament_add_gtest(test_extrusion_wire test/test_extrusion_wire.cpp)
target_link_libraries(test_extrusion_wire uart_extrusion_wire)
ament_add_pytest_test(
  test_extrude_reset_handshake test/test_extrude_reset_handshake.py)
ament_add_pytest_test(
  test_temperature_ready_threshold test/test_temperature_ready_threshold.py)
```

- [ ] **Step 4: 验证 GREEN**

Run:

```bash
colcon build --packages-select uart_bridge --symlink-install
colcon test --packages-select uart_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 新增 gtest 和既有 Python 测试全部通过。

- [ ] **Step 5: 提交 canonical 编码基础**

```bash
git add \
  src/my_project/uart_bridge/include/uart_bridge/extrusion_wire.hpp \
  src/my_project/uart_bridge/src/extrusion_wire.cpp \
  src/my_project/uart_bridge/test/test_extrusion_wire.cpp \
  src/my_project/uart_bridge/CMakeLists.txt \
  src/my_project/uart_bridge/package.xml
git commit -m "feat(uart): add canonical extrusion wire encoder"
```

### Task 2: 用提交状态测试驱动 canonical 去重

**Files:**
- Modify: `src/my_project/uart_bridge/test/test_extrusion_wire.cpp`
- Modify: `src/my_project/uart_bridge/src/extrusion_wire.cpp`

- [ ] **Step 1: 添加失败的状态与去重测试**

逐项覆盖以下真实 API 行为：

```cpp
TEST(CanonicalExtrusionWireTest, DoesNotSuppressUntilCandidateIsCommitted)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto first = forwarder.prepare(10U, 1, 2.0000001);
  const auto retry = forwarder.prepare(11U, 1, 2.0000004);
  ASSERT_EQ(first.decision, ExtrusionDecision::Send);
  ASSERT_EQ(retry.decision, ExtrusionDecision::Send);

  forwarder.commit(*first.candidate);
  const auto duplicate = forwarder.prepare(12U, 1, 2.0000004);
  EXPECT_EQ(duplicate.decision, ExtrusionDecision::SuppressDuplicate);
}

TEST(CanonicalExtrusionWireTest, ToolChangeAndReturnToZeroAreForwarded)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto nonzero = forwarder.prepare(1U, 1, 3.0);
  forwarder.commit(*nonzero.candidate);
  EXPECT_EQ(
    forwarder.prepare(2U, 2, 3.0).decision,
    ExtrusionDecision::Send);
  const auto zero = forwarder.prepare(3U, 1, 0.0);
  ASSERT_EQ(zero.decision, ExtrusionDecision::Send);
  EXPECT_EQ(zero.candidate->line, "E 3 1 0.000000\n");
}

TEST(CanonicalExtrusionWireTest, ResetRestoresInitialZeroSuppression)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  const auto first = forwarder.prepare(1U, 1, 1.0);
  forwarder.commit(*first.candidate);
  forwarder.reset();
  EXPECT_EQ(
    forwarder.prepare(2U, 1, 0.0).decision,
    ExtrusionDecision::SuppressInitialZero);
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run:

```bash
colcon test --packages-select uart_bridge --event-handlers console_direct+
```

Expected: FAIL，失败原因是未提交候选项仍被当成已发送，或 `commit/reset` 尚未维护状态。

- [ ] **Step 3: 实现 canonical 已提交键**

`prepare()` 只读取 `(last_sent_tool_id_, last_sent_canonical_e_nm_)`；初始
`e_nm == 0` 返回 `SuppressInitialZero`，同工具同 `e_nm` 返回
`SuppressDuplicate`。`commit()` 才把候选项复制到已提交状态；`reset()` 清空
valid、工具和两种模式的数值成员。

- [ ] **Step 4: 运行测试确认 GREEN**

Run:

```bash
colcon test --packages-select uart_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 全部通过。

- [ ] **Step 5: 提交事务去重状态**

```bash
git add \
  src/my_project/uart_bridge/src/extrusion_wire.cpp \
  src/my_project/uart_bridge/test/test_extrusion_wire.cpp
git commit -m "feat(uart): commit extrusion dedup after delivery"
```

### Task 3: 驱动非法值拒绝和 legacy 回退

**Files:**
- Modify: `src/my_project/uart_bridge/test/test_extrusion_wire.cpp`
- Modify: `src/my_project/uart_bridge/src/extrusion_wire.cpp`

- [ ] **Step 1: 添加失败的校验和 legacy 测试**

```cpp
TEST(CanonicalExtrusionWireTest, RejectsNonFiniteAndOutOfRangeWithoutStateChange)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::CanonicalV1);
  EXPECT_EQ(
    forwarder.prepare(1U, 1, std::numeric_limits<double>::quiet_NaN()).decision,
    ExtrusionDecision::RejectNonFinite);
  EXPECT_EQ(
    forwarder.prepare(2U, 1, std::numeric_limits<double>::infinity()).decision,
    ExtrusionDecision::RejectNonFinite);
  EXPECT_EQ(
    forwarder.prepare(3U, 1, 1.0e20).decision,
    ExtrusionDecision::RejectOutOfRange);
  EXPECT_EQ(
    forwarder.prepare(4U, 1, 0.0).decision,
    ExtrusionDecision::SuppressInitialZero);
}

TEST(LegacyExtrusionWireTest, KeepsOldFloatTextAndDoubleEpsilonDedup)
{
  ExtrusionForwarder forwarder(ExtrusionWireMode::LegacyV1);
  const auto first = forwarder.prepare(7U, 2, 1.25);
  ASSERT_EQ(first.decision, ExtrusionDecision::Send);
  EXPECT_EQ(first.candidate->line, "E 7 2 1.25\n");
  forwarder.commit(*first.candidate);
  EXPECT_EQ(
    forwarder.prepare(8U, 2, 1.25 + 0.5e-9).decision,
    ExtrusionDecision::SuppressDuplicate);
  EXPECT_EQ(
    forwarder.prepare(9U, 2, 1.25 + 2.0e-9).decision,
    ExtrusionDecision::Send);
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run:

```bash
colcon test --packages-select uart_bridge --event-handlers console_direct+
```

Expected: FAIL，canonical 尚未返回拒绝状态，legacy 尚未输出旧格式或旧 epsilon 去重。

- [ ] **Step 3: 实现严格 canonical 校验和 legacy 分支**

canonical 在任何整数转换前依次检查：

```cpp
if (!std::isfinite(scaled_e_abs)) {
  return {ExtrusionDecision::RejectNonFinite, std::nullopt};
}
const long double units = static_cast<long double>(scaled_e_abs) * 1'000'000.0L;
const long double rounded = std::round(units);
constexpr long double kInt64Min = -9223372036854775808.0L;
constexpr long double kInt64MaxExclusive = 9223372036854775808.0L;
if (!std::isfinite(rounded) || rounded < kInt64Min ||
  rounded >= kInt64MaxExclusive)
{
  return {ExtrusionDecision::RejectOutOfRange, std::nullopt};
}
```

legacy 分支保持当前顺序：以缩放后的 `double` 和 `1e-9` 做初始零/重复判断，
构造行时才 `static_cast<float>(scaled_e_abs)` 并使用默认 `ostringstream`。
两个模式都只由 `commit()` 更新状态。

- [ ] **Step 4: 运行测试确认 GREEN**

Run:

```bash
colcon test --packages-select uart_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 全部通过。

- [ ] **Step 5: 提交模式回退和输入校验**

```bash
git add \
  src/my_project/uart_bridge/src/extrusion_wire.cpp \
  src/my_project/uart_bridge/test/test_extrusion_wire.cpp
git commit -m "feat(uart): add legacy extrusion wire rollback"
```

### Task 4: 在 uart_node 接入成功写入事务

**Files:**
- Create: `src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py`
- Modify: `src/my_project/uart_bridge/test/test_extrude_reset_handshake.py`
- Modify: `src/my_project/uart_bridge/src/uart_node.cpp`
- Modify: `src/my_project/uart_bridge/CMakeLists.txt`

- [ ] **Step 1: 先写失败的节点契约测试**

测试读取源码并明确保护既有门控及新事务顺序：

```python
def test_heartbeat_keeps_existing_guards_and_commits_only_after_write_success():
    src = UART_NODE.read_text(encoding="utf-8")
    heartbeat = src.split("void on_heartbeat(const RsiHeartBeat & hb)", 1)[1].split(
        "void read_loop", 1
    )[0]
    assert "paused_.load() || aborted_.load() || has_pending_event()" in heartbeat
    assert "extrude_abs * scale" in heartbeat
    assert "extrusion_forwarder_->prepare" in heartbeat
    assert "write_line(candidate.line)" in heartbeat
    assert "extrusion_forwarder_->commit(candidate)" in heartbeat
    assert heartbeat.index("write_line(candidate.line)") < heartbeat.index(
        "extrusion_forwarder_->commit(candidate)"
    )


def test_write_line_preserves_tx_log_and_reports_delivery_status():
    src = UART_NODE.read_text(encoding="utf-8")
    writer = src.split("bool write_line(const std::string & line)", 1)[1].split(
        "void on_print_test_command", 1
    )[0]
    assert 'publish_uart_log("TX", line)' in writer
    assert "boost::asio::write" in writer
    assert writer.index('publish_uart_log("TX", line)') < writer.index(
        "boost::asio::write"
    )
    assert 'publish_uart_log("TX_FAIL"' not in writer
    assert "return false" in writer
    assert "return true" in writer
```

把旧 reset 测试中的 `last_sent_e_*` 断言改为
`reset_extrude_forward_state()` 调用纯组件 `reset()`；把脆弱的函数切分终点更新为
`bool write_line`。在 CMake 中注册新 pytest 文件。

- [ ] **Step 2: 运行测试确认 RED**

Run:

```bash
python3 -m pytest -q \
  src/my_project/uart_bridge/test/test_extrude_reset_handshake.py \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py
```

Expected: FAIL，源码尚无 `extrusion_forwarder_->prepare`，且 `write_line` 仍返回 void。

- [ ] **Step 3: 最小化修改 uart_node 接线**

实施以下局部变化：

```cpp
#include "uart_bridge/extrusion_wire.hpp"
#include <memory>
```

成员替换为：

```cpp
std::mutex extrude_forward_mutex_;
std::unique_ptr<uart_bridge::ExtrusionForwarder> extrusion_forwarder_;
```

构造期声明并严格解析参数：

```cpp
const auto wire_mode_text =
  declare_parameter<std::string>("extrusion_wire_mode", "canonical_v1");
const auto wire_mode = uart_bridge::parse_extrusion_wire_mode(wire_mode_text);
extrusion_forwarder_ =
  std::make_unique<uart_bridge::ExtrusionForwarder>(wire_mode);
RCLCPP_INFO(
  get_logger(), "UART挤出边界模式: %s",
  uart_bridge::extrusion_wire_mode_name(wire_mode));
```

心跳门控前三项保持原样，门控通过后只替换 UART 出口：

```cpp
const double scale = extrude_scale_.load();
const double scaled = hb.extrude_abs * scale;
std::lock_guard<std::mutex> lk(extrude_forward_mutex_);
const auto prepared =
  extrusion_forwarder_->prepare(hb.seq_used, hb.tool_id, scaled);
if (prepared.decision == uart_bridge::ExtrusionDecision::RejectNonFinite ||
  prepared.decision == uart_bridge::ExtrusionDecision::RejectOutOfRange)
{
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "拒绝非法UART挤出量: seq=%u tool=%d value=%.17g",
    hb.seq_used, hb.tool_id, scaled);
  return;
}
if (prepared.decision != uart_bridge::ExtrusionDecision::Send) {
  return;
}
const auto & candidate = prepared.candidate.value();
if (write_line(candidate.line)) {
  extrusion_forwarder_->commit(candidate);
}
```

`reset_extrude_forward_state()` 保留现有调用点，只改内部为加锁后
`extrusion_forwarder_->reset()`。删除旧 `last_sent_e_*` 状态、
`should_forward_extrude()` 和 `send_extrude_command()`。

`write_line()` 改为：

```cpp
bool write_line(const std::string & line)
{
  publish_uart_log("TX", line);
  std::lock_guard<std::mutex> lk(serial_write_mutex_);
  boost::system::error_code ec;
  const auto written = boost::asio::write(serial_, boost::asio::buffer(line), ec);
  if (ec || written != line.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "写入失败：%s (%zu/%zu bytes)",
      ec ? ec.message().c_str() : "partial write", written, line.size());
    return false;
  }
  return true;
}
```

其他事件、手动命令和 ABORT 调用点继续忽略 bool 返回值，控制流程不变。

- [ ] **Step 4: 验证节点契约和构建 GREEN**

Run:

```bash
python3 -m pytest -q \
  src/my_project/uart_bridge/test/test_extrude_reset_handshake.py \
  src/my_project/uart_bridge/test/test_temperature_ready_threshold.py \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py \
  src/my_project/control_center/test/test_staged_pause_contract.py
colcon build --packages-select uart_bridge --symlink-install
colcon test --packages-select uart_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 全部通过；编译无新增 warning。

- [ ] **Step 5: 检查没有无关链路改动**

Run:

```bash
git diff --name-only
git diff -- src/my_project/rsi_server src/my_project/control_center \
  src/my_project/path_processing_core src/my_project/my_project_interfaces \
  src/my_project/my_project_ui
```

Expected: 第二条命令无输出。

- [ ] **Step 6: 提交节点事务接线**

```bash
git add \
  src/my_project/uart_bridge/src/uart_node.cpp \
  src/my_project/uart_bridge/test/test_extrude_reset_handshake.py \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py \
  src/my_project/uart_bridge/CMakeLists.txt
git commit -m "feat(uart): commit E state after serial write"
```

### Task 5: 启动参数透传与回退契约

**Files:**
- Modify: `src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py`
- Modify: `src/my_project/my_project_startup/launch/startup.launch.py`

- [ ] **Step 1: 添加失败的启动参数测试**

```python
def test_startup_defaults_to_canonical_mode_and_only_passes_it_to_uart():
    launch = STARTUP_LAUNCH.read_text(encoding="utf-8")
    assert 'extrusion_wire_mode = LaunchConfiguration("extrusion_wire_mode")' in launch
    assert '"extrusion_wire_mode": extrusion_wire_mode' in launch
    declaration = 'DeclareLaunchArgument(\n            "extrusion_wire_mode"'
    assert declaration in launch
    assert 'default_value="canonical_v1"' in launch.split(declaration, 1)[1].split(
        "),", 1
    )[0]
    rsi_parameters = launch.split("rsi_node = Node(", 1)[1].split(
        "uart_node = Node(", 1
    )[0]
    assert "extrusion_wire_mode" not in rsi_parameters
```

- [ ] **Step 2: 运行测试确认 RED**

Run:

```bash
python3 -m pytest -q \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py
```

Expected: FAIL，启动文件尚未声明 `extrusion_wire_mode`。

- [ ] **Step 3: 只在 UART 参数区声明和透传**

在现有 UART `LaunchConfiguration` 三项后增加：

```python
extrusion_wire_mode = LaunchConfiguration("extrusion_wire_mode")
```

只在 `uart_node.parameters` 增加：

```python
"extrusion_wire_mode": extrusion_wire_mode,
```

在 `extrude_scale` 声明后增加：

```python
DeclareLaunchArgument(
    "extrusion_wire_mode",
    default_value="canonical_v1",
    description="UART 挤出文本/去重模式：canonical_v1 或 legacy_v1。",
),
```

- [ ] **Step 4: 验证 GREEN 和 Python 语法**

Run:

```bash
python3 -m pytest -q \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py \
  src/my_project/control_center/test/test_staged_pause_contract.py
python3 -m py_compile \
  src/my_project/my_project_startup/launch/startup.launch.py
```

Expected: 全部通过。

- [ ] **Step 5: 提交启动回退参数**

```bash
git add \
  src/my_project/my_project_startup/launch/startup.launch.py \
  src/my_project/uart_bridge/test/test_uart_extrusion_boundary_contract.py
git commit -m "feat(uart): expose extrusion wire rollback mode"
```

### Task 6: 全量相关验证和固件交接文档

**Files:**
- Create: `docs/firmware_uart_extrusion_canonical_v1_handoff.md`

- [ ] **Step 1: 运行相关包构建和测试**

Run:

```bash
colcon build --packages-select uart_bridge my_project_startup --symlink-install
colcon test --packages-select uart_bridge my_project_startup \
  --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest -q \
  src/my_project/uart_bridge/test \
  src/my_project/control_center/test/test_staged_pause_contract.py
```

Expected: 构建成功，所有相关测试通过，无失败。

- [ ] **Step 2: 审计改动范围**

Run:

```bash
git diff 90b868c --name-only
git diff 90b868c -- src/my_project/rsi_server src/my_project/control_center \
  src/my_project/path_processing_core src/my_project/my_project_interfaces \
  src/my_project/my_project_ui
```

Expected: 只出现本计划“文件结构”中的允许文件；第二条命令无输出。

- [ ] **Step 3: 创建固件交接文档**

文档必须明确写出以下最终接口，不把计划行为冒充已验证行为：

```markdown
# 上位机 UART 挤出 canonical_v1 固件确认说明

## 线上协议
`E <seq_used> <tool_id> <absolute_E_fixed6>\n`，E 是 mm 绝对位置；
命令语法、字段顺序和 LF 均未改变。

## 4 ms 实时流程
KUKA RSI 心跳约每 4 ms 触发一次检查；PAUSE、ABORT 或 pending event 时不发；
其余周期先做 `heartbeat.extrude_abs * extrude_scale`，再 canonical 量化和去重。
重复 E 仍省略，因此 UART 不保证每 4 ms 都有 E 行。

## canonical_v1
默认模式。以 `10^-6 mm` 整数值 fixed6 输出；去重键为 `(tool_id, e_nm)`；
初始零省略，工具变化和非零后回零会发送；非有限值/越界值拒绝。

## 写失败
只有同步串口完整写成功才提交去重键。失败后下一次有效心跳发送当时最新绝对 E
和最新 `seq_used`；没有 ACK 闭环，也不重放旧心跳。

## 回退
启动参数 `extrusion_wire_mode:=legacy_v1` 恢复旧 float/default-stream 文本和
`1e-9` 数值去重，但仍采用写成功后提交。

## 未修改链路
NPZ、控制中心、RSI、4 ms 主循环、PAUSE/ABORT、事件、reset、UI 和消息接口
均未修改。

## 已接受系统边界
最后一条永久丢失无法推断；UART 前丢失精度无法恢复；无编码器无法确认物理失步；
长时间 EN 的温升和电流需固件/硬件侧实机确认。
```

补充实际验证命令、测试结果和 `legacy_v1` 回退命令，供固件侧直接确认。

- [ ] **Step 4: 自审文档与 diff**

Run:

```bash
rg -n "T[B]D|T[O]DO|尚未实[现]|待[定]|占[位]" \
  docs/firmware_uart_extrusion_canonical_v1_handoff.md
git diff --check
git status --short
```

Expected: 无占位符、无空白错误，只保留预期改动。

- [ ] **Step 5: 提交固件交接文档**

```bash
git add docs/firmware_uart_extrusion_canonical_v1_handoff.md
git commit -m "docs: hand off canonical UART extrusion behavior"
```

- [ ] **Step 6: 最终验证并记录提交链**

Run:

```bash
git status --short --branch
git log --oneline 90b868c..HEAD
```

Expected: 实现 worktree 干净，提交按编码器、事务去重、节点接线、启动参数、交接
文档分层；然后使用 `verification-before-completion` 和
`finishing-a-development-branch` 完成交付与集成选择。
