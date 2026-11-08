#include "runtime_logger/integrity.hpp"

#include <cstdio>
#include <sstream>

namespace runtime_logger
{

ProducerAccount & Integrity::producer(
  const std::string & id, uint64_t epoch, bool started)
{
  auto & acc = accounts_[id];
  acc.producer_id = id;
  acc.producer_epoch = epoch;
  acc.started = started;
  return acc;
}

void Integrity::note_record(
  const std::string & producer_id,
  uint64_t record_seq,
  uint64_t /*capture_seq*/,
  bool parsed_ok)
{
  auto & acc = accounts_[producer_id];
  if (acc.producer_id.empty()) {
    acc.producer_id = producer_id;
  }
  if (!parsed_ok) {
    ++acc.parse_errors;
    return;
  }
  if (acc.received_records == 0) {
    acc.first_record_seq = record_seq;
    acc.last_record_seq = record_seq;
  } else if (record_seq > acc.last_record_seq + 1) {
    const uint64_t missed = record_seq - acc.last_record_seq - 1;
    acc.gap_count += missed;
    if (acc.gap_ranges.size() < 2048) {
      if (!acc.gap_ranges.empty()) {acc.gap_ranges += ",";}
      char buf[64];
      std::snprintf(
        buf, sizeof(buf), "%llu-%llu",
        static_cast<unsigned long long>(acc.last_record_seq + 1),
        static_cast<unsigned long long>(record_seq - 1));
      acc.gap_ranges += buf;
    }
    acc.last_record_seq = record_seq;
  } else if (record_seq > acc.last_record_seq) {
    acc.last_record_seq = record_seq;
  }
  ++acc.received_records;
}

void Integrity::note_end(const std::string & producer_id)
{
  auto it = accounts_.find(producer_id);
  if (it != accounts_.end()) {
    it->second.ended = true;
  }
}

void Integrity::note_overflow(uint64_t n)
{
  overflow_ += n;
}

void Integrity::note_write(uint64_t n)
{
  written_ += n;
}

void Integrity::note_parse_error(const std::string & producer_id)
{
  auto & acc = accounts_[producer_id];
  if (acc.producer_id.empty()) {
    acc.producer_id = producer_id;
  }
  ++acc.parse_errors;
}

uint64_t Integrity::total_gaps() const
{
  uint64_t n = overflow_;
  for (const auto & kv : accounts_) {
    n += kv.second.gap_count;
    if (kv.second.started && !kv.second.ended) {
      // Un-ended producer counts as incomplete rather than a numeric gap.
    }
  }
  return n;
}

uint64_t Integrity::total_received() const
{
  uint64_t n = 0;
  for (const auto & kv : accounts_) {
    n += kv.second.received_records;
  }
  return n;
}

std::string Integrity::to_json(bool session_complete) const
{
  std::ostringstream os;
  os << "{\"session_complete\":" << (session_complete ? "true" : "false")
     << ",\"overflow_records\":" << overflow_
     << ",\"written_records\":" << written_
     << ",\"producers\":{";
  bool first = true;
  for (const auto & kv : accounts_) {
    if (!first) {os << ",";}
    first = false;
    const auto & a = kv.second;
    os << "\"" << a.producer_id << "\":{"
       << "\"epoch\":" << a.producer_epoch
       << ",\"started\":" << (a.started ? "true" : "false")
       << ",\"ended\":" << (a.ended ? "true" : "false")
       << ",\"first_seq\":" << a.first_record_seq
       << ",\"last_seq\":" << a.last_record_seq
       << ",\"received\":" << a.received_records
       << ",\"parse_errors\":" << a.parse_errors
       << ",\"gaps\":" << a.gap_count
       << ",\"gap_ranges\":\"" << a.gap_ranges << "\""
       << "}";
  }
  os << "}}";
  return os.str();
}

}  // namespace runtime_logger
