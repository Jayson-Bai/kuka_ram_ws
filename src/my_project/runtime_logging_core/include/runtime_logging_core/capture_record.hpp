#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace runtime_logging
{

/* Direction of a captured UART frame. */
enum class Direction : uint8_t
{
  kRx = 0,
  kTx = 1,
};

/* Payload class before whitelist projection. */
enum class CaptureType : uint8_t
{
  kUnknown = 0,
  kStat = 1,
  kEstat = 2,    // full ESTAT frame (hot + cold fields)
  kEstatHot = 3, // ESTAT_H hot-only frame
  kUstat = 4,
  kEack = 5,
  kEwarn = 6,
  kEvent = 7,    // EV / EVACK / EVDONE / EVERR
  kTx = 8,       // host TX line (E / EV / other)
};

/*
 * A captured UART line with its source-side monotonic timestamp.
 * host_mono_ns is CLOCK_MONOTONIC nanoseconds stamped at capture time,
 * never the writer or consumer time.
 */
struct CaptureRecord
{
  uint64_t capture_seq{0};       // global capture order for this producer
  int64_t host_mono_ns{0};       // CLOCK_MONOTONIC at capture
  Direction direction{Direction::kRx};
  CaptureType type{CaptureType::kUnknown};
  bool tx_ok{true};              // TX only: boost::asio write result
  uint32_t bytes_written{0};     // TX only
  std::string line;              // wire text without trailing \r\n
};

int64_t clock_monotonic_ns();

CaptureType classify_line(const std::string & line, Direction dir);

}  // namespace runtime_logging
