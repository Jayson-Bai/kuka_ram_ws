#pragma once

#include <array>
#include <cstdint>

#include "bounded_ring.hpp"

namespace runtime_logging
{

/*
 * Fixed-size capture of one RSI cycle, produced on the UDP thread
 * immediately after sendto() and consumed by a single relay thread.
 * All fields are plain value types so a pre-allocated ring slot never
 * allocates or resizes in the hot path.
 */
struct RsiTelemetryFrame
{
  static constexpr uint32_t kRxCapacity = 4096;
  static constexpr uint32_t kTxCapacity = 512;
  static constexpr uint32_t kMaxIpocLen = 32;

  uint64_t rx_index{0};           // monotonically increasing recv count
  int64_t rx_mono_ns{0};          // CLOCK_MONOTONIC at recvfrom() return
  int64_t tx_mono_ns{0};          // CLOCK_MONOTONIC at sendto() return
  uint32_t rx_len{0};             // actual bytes received
  uint32_t tx_len{0};             // actual bytes sent
  bool rx_truncated{false};       // MSG_TRUNC observed on recv
  bool send_ok{false};            // sendto() returned the full tx_len
  int32_t send_errno{0};          // errno after sendto(), 0 when send_ok
  bool startup_fast_reply{false}; // first-packet fast path
  uint8_t state{0};               // RSINode::State at send time

  // Command actually sent this cycle (snapshot of to_send).
  uint32_t seq_used{0};
  bool seq_valid{false};          // to_send advanced this cycle
  uint32_t path_id{0};
  int32_t tool_id{0};
  double cmd_x{0.0};
  double cmd_y{0.0};
  double cmd_z{0.0};
  double cmd_a{0.0};
  double cmd_b{0.0};
  double cmd_c{0.0};
  double cmd_e{0.0};

  // IPOC text extracted on the UDP thread for the reply; stored verbatim.
  char ipoc_raw[kMaxIpocLen + 1]{};
  bool ipoc_valid{false};

  std::array<char, kRxCapacity> rx{};
  std::array<char, kTxCapacity> tx{};
};

using RsiTelemetryRing = BoundedRing<RsiTelemetryFrame>;

}  // namespace runtime_logging
