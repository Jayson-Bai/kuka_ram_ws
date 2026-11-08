#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace runtime_logger
{

/* Per-producer accounting for the integrity report. */
struct ProducerAccount
{
  std::string producer_id;
  uint64_t producer_epoch{0};
  bool started{false};
  bool ended{false};
  uint64_t first_record_seq{0};
  uint64_t last_record_seq{0};
  uint64_t expected_records{0};
  uint64_t received_records{0};
  uint64_t written_records{0};
  uint64_t gap_count{0};
  uint64_t parse_errors{0};
  // Comma-separated "start-end" gap ranges, capped.
  std::string gap_ranges;
};

class Integrity
{
public:
  ProducerAccount & producer(
    const std::string & id, uint64_t epoch, bool started);
  void note_record(
    const std::string & producer_id,
    uint64_t record_seq,
    uint64_t capture_seq,
    bool parsed_ok);
  void note_end(const std::string & producer_id);
  void note_overflow(uint64_t n);
  void note_write(uint64_t n);
  void note_parse_error(const std::string & producer_id);
  std::string to_json(bool session_complete) const;
  uint64_t total_gaps() const;
  uint64_t total_received() const;
  uint64_t total_written() const {return written_;}
  uint64_t total_overflow() const {return overflow_;}
  const std::map<std::string, ProducerAccount> & accounts() const
  {
    return accounts_;
  }

private:
  std::map<std::string, ProducerAccount> accounts_;
  uint64_t overflow_{0};
  uint64_t written_{0};
};

}  // namespace runtime_logger
