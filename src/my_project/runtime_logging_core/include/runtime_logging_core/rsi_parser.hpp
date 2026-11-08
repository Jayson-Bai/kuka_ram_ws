#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace runtime_logging
{

/*
 * Per-tag parsed view of one <Rob .../> RSI feedback frame.
 * Only the whitelisted tags are extracted: Pos, AIPos, ASPos, Delay,
 * IPOC. Each group has its own valid flag; a missing or malformed
 * group never borrows values from the previous frame.
 */
struct RsiFeedbackView
{
  bool pos_valid{false};
  std::array<double, 6> pos{};            // X Y Z A B C
  int64_t pos_s{0};
  int64_t pos_t{0};
  bool pos_st_valid{false};               // S and T both parsed as integers

  bool aipos_valid{false};
  std::array<double, 6> aipos{};          // A1..A6 actual, degrees

  bool aspos_valid{false};
  std::array<double, 6> aspos{};          // A1..A6 setpoint, degrees

  bool delay_valid{false};
  int64_t delay_count{0};
  std::string delay_raw;

  bool ipoc_valid{false};
  std::string ipoc_raw;

  // Bit flags describing parse problems for the log record.
  uint32_t parse_flags{0};
};

enum RsiParseFlag : uint32_t
{
  kFlagTruncatedRx = 1u << 0,
  kFlagMissingPos = 1u << 1,
  kFlagMissingAipos = 1u << 2,
  kFlagMissingAspos = 1u << 3,
  kFlagMissingDelay = 1u << 4,
  kFlagMissingIpoc = 1u << 5,
  kFlagMalformed = 1u << 6,
};

/*
 * Parse a bounded Rob XML frame in place. The input is a byte view;
 * rx_len bounds the read and NUL termination is not required.
 * Locale-independent, attribute-order tolerant, no regex backtracking
 * and no external entity handling.
 */
RsiFeedbackView parse_rsi_feedback(const char * data, size_t rx_len);

}  // namespace runtime_logging
