#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "capture_record.hpp"

namespace runtime_logging
{

/* Whitelisted business record kinds (schema_version 1). */
enum class RecordKind : uint8_t
{
  kEstatHot = 0,
  kEstatCold = 1,
  kUstat = 2,
  kLatency = 3,
  kEvent = 4,
  kETx = 5,
};

const char * record_kind_name(RecordKind kind);

/*
 * A whitelisted key/value projection of one captured line.
 * values are stored verbatim as strings so integer precision and hex
 * formatting are preserved exactly as they appeared on the wire.
 */
struct WhitelistedRecord
{
  RecordKind kind{RecordKind::kEstatHot};
  uint64_t record_seq{0};
  int64_t host_mono_ns{0};
  uint64_t capture_seq{0};
  // MCU-side identity fields; null when the frame did not carry them.
  std::optional<uint64_t> mcu_sample_seq;
  std::optional<int64_t> mcu_sample_us;
  std::optional<uint64_t> mcu_boot_id;
  std::optional<int64_t> mcu_event_us;
  std::optional<int64_t> tool_id;
  std::optional<uint64_t> event_request_id;
  std::optional<uint32_t> reset_epoch;
  bool valid{true};
  std::string invalid_reason;
  bool tx_ok{true};
  uint32_t bytes_written{0};
  // kind-specific whitelisted business fields, wire order preserved.
  std::vector<std::pair<std::string, std::string>> fields;
};

/*
 * Strict key whitelist per kind. A captured field that is not in the
 * whitelist for the record kind never reaches the log.
 */
class Whitelist
{
public:
  // Build the six whitelist entries for one captured line.
  // Returns zero or more records (a full ESTAT produces hot+cold).
  std::vector<WhitelistedRecord> project(
    const CaptureRecord & capture,
    uint64_t first_record_seq) const;

private:
  static const std::map<std::string, bool> & allowed(RecordKind kind);
};

/* Encode one record as a single JSON line (no trailing newline). */
std::string encode_record_json(
  const WhitelistedRecord & rec,
  const std::string & run_id,
  const std::string & producer_id,
  uint64_t producer_epoch);

}  // namespace runtime_logging
