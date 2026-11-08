#include "runtime_logger/integrity.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace runtime_logger
{

namespace
{

uint64_t producer_loss(const ProducerAccount & acc)
{
  uint64_t loss = acc.ring_dropped + acc.ipc_dropped;
  if (acc.sent_records > acc.received_records) {
    loss += acc.sent_records - acc.received_records;
  }
  return loss;
}

}  // namespace

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
  acc.started = true;
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

void Integrity::note_producer_status(const ProducerStatusUpdate & update)
{
  auto & acc = accounts_[update.producer_id];
  if (acc.producer_id.empty()) {
    acc.producer_id = update.producer_id;
  }
  acc.started = true;
  acc.producer_epoch = update.producer_epoch;
  acc.status_phase = update.phase;
  acc.producer_first_seq = update.first_record_seq;
  acc.producer_last_seq = update.last_record_seq;
  acc.sent_records = update.sent_records;
  acc.ring_dropped = update.ring_dropped;
  acc.ipc_dropped = update.ipc_dropped;
  acc.send_errors = update.send_errors;
  acc.pending_messages = update.pending_messages;
  acc.pending_bytes = update.pending_bytes;
  acc.connected = update.connected;
  if (update.phase == "final") {
    acc.ended = true;
  }
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

uint64_t Integrity::total_producer_drops() const
{
  uint64_t n = 0;
  for (const auto & kv : accounts_) {
    n += producer_loss(kv.second);
  }
  return n;
}

uint64_t Integrity::total_gaps() const
{
  uint64_t n = overflow_ + total_producer_drops();
  for (const auto & kv : accounts_) {
    n += kv.second.gap_count;
  }
  return n;
}

bool Integrity::all_producers_ended() const
{
  for (const auto & kv : accounts_) {
    if (kv.second.started && !kv.second.ended) {
      return false;
    }
  }
  return true;
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
       << ",\"status_phase\":\"" << a.status_phase << "\""
       << ",\"producer_first_seq\":" << a.producer_first_seq
       << ",\"producer_last_seq\":" << a.producer_last_seq
       << ",\"sent_records\":" << a.sent_records
       << ",\"ring_dropped\":" << a.ring_dropped
       << ",\"ipc_dropped\":" << a.ipc_dropped
       << ",\"send_errors\":" << a.send_errors
       << ",\"pending_messages\":" << a.pending_messages
       << ",\"pending_bytes\":" << a.pending_bytes
       << ",\"connected\":" << (a.connected ? "true" : "false")
       << ",\"estimated_loss\":" << producer_loss(a)
       << "}";
  }
  os << "}}";
  return os.str();
}

}  // namespace runtime_logger
