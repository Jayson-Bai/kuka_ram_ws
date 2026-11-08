#include "runtime_logging_core/producer_status.hpp"

#include <sstream>
#include <string>

namespace runtime_logging
{

namespace
{

std::string json_escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

}  // namespace

std::string encode_producer_status_json(const ProducerStatus & s)
{
  std::ostringstream os;
  os << "{";
  os << "\"control\":\"producer_status\"";
  os << ",\"schema_version\":1";
  os << ",\"run_id\":\"" << json_escape(s.run_id) << "\"";
  os << ",\"producer_id\":\"" << json_escape(s.producer_id) << "\"";
  os << ",\"producer_epoch\":" << s.producer_epoch;
  os << ",\"phase\":\"" << json_escape(s.phase) << "\"";
  os << ",\"first_record_seq\":" << s.first_record_seq;
  os << ",\"last_record_seq\":" << s.last_record_seq;
  os << ",\"sent_records\":" << s.sent_records;
  os << ",\"ring_dropped\":" << s.ring_dropped;
  os << ",\"ipc_dropped\":" << s.ipc_dropped;
  os << ",\"send_errors\":" << s.send_errors;
  os << ",\"pending_messages\":" << s.pending_messages;
  os << ",\"pending_bytes\":" << s.pending_bytes;
  os << ",\"connected\":" << (s.connected ? "true" : "false");
  os << "}";
  return os.str();
}

}  // namespace runtime_logging
