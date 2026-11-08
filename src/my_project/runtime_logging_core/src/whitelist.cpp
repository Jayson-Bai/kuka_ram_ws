#include "runtime_logging_core/whitelist.hpp"

#include <cctype>
#include <cstdio>
#include <sstream>

namespace runtime_logging
{

namespace
{

const std::map<std::string, bool> & hot_keys()
{
  static const std::map<std::string, bool> keys = {
    {"state", true},
    {"target_steps", true},
    {"submitted_steps", true},
    {"last_seq", true},
    {"ff_sps", true},
    {"v_sps", true},
    {"fault", true},
  };
  return keys;
}

const std::map<std::string, bool> & cold_keys()
{
  static const std::map<std::string, bool> keys = {
    {"completed_steps", true},
    {"phase", true},
    {"hard_fault", true},
  };
  return keys;
}

const std::map<std::string, bool> & ustat_keys()
{
  static const std::map<std::string, bool> keys = {
    {"fifo_ovf", true},
    {"buffer_full", true},
    {"line_drop", true},
    {"seq_gap", true},
    {"parse_err", true},
  };
  return keys;
}

const std::map<std::string, bool> & latency_keys()
{
  static const std::map<std::string, bool> keys = {
    {"seq_lag", true},
    {"ack_delay_ms", true},
    {"linux_mcu_delay_ms", true},
    {"mcu_robot_delay_ms", true},
    {"robot_match_error_mm", true},
  };
  return keys;
}

const std::map<std::string, bool> & event_keys()
{
  static const std::map<std::string, bool> keys = {
    {"event_type", true},
    {"event_stage", true},
  };
  return keys;
}

const std::map<std::string, bool> & etx_keys()
{
  static const std::map<std::string, bool> keys = {
    {"last_extrude_abs", true},
  };
  return keys;
}

using KvList = std::vector<std::pair<std::string, std::string>>;

/* Parse "key=value" tokens after the leading tag word. */
KvList parse_kv(const std::string & line, size_t tag_len)
{
  KvList out;
  std::istringstream ss(line.substr(tag_len));
  std::string tok;
  while (ss >> tok) {
    const auto eq = tok.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    out.emplace_back(tok.substr(0, eq), tok.substr(eq + 1));
  }
  return out;
}

std::optional<int64_t> find_i64(const KvList & kv, const char * key)
{
  for (const auto & p : kv) {
    if (p.first == key) {
      try {
        return std::stoll(p.second);
      } catch (...) {
        return std::nullopt;
      }
    }
  }
  return std::nullopt;
}

std::optional<uint64_t> find_u64(const KvList & kv, const char * key)
{
  for (const auto & p : kv) {
    if (p.first == key) {
      try {
        return std::stoull(p.second);
      } catch (...) {
        return std::nullopt;
      }
    }
  }
  return std::nullopt;
}

std::optional<uint32_t> find_u32(const KvList & kv, const char * key)
{
  const auto v = find_u64(kv, key);
  if (!v || *v > 0xffffffffULL) {
    return std::nullopt;
  }
  return static_cast<uint32_t>(*v);
}

void pick_fields(
  WhitelistedRecord & rec, const KvList & kv,
  const std::map<std::string, bool> & allowed_keys)
{
  for (const auto & p : kv) {
    if (allowed_keys.count(p.first)) {
      rec.fields.push_back(p);
    }
  }
}

std::string json_escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

/* Emit an integer-valued wire token verbatim; decimal stays decimal and
 * 0x-prefixed hex becomes a fixed hex string (schema-consistent). */
std::string scalar_json(const std::string & v)
{
  if (v.empty()) {
    return "null";
  }
  if (v.rfind("0x", 0) == 0 || v.rfind("0X", 0) == 0) {
    return "\"" + json_escape(v) + "\"";
  }
  bool numeric = true;
  for (char c : v) {
    if (!(std::isdigit(static_cast<unsigned char>(c)) || c == '-' ||
      c == '+' || c == '.' || c == 'e' || c == 'E'))
    {
      numeric = false;
      break;
    }
  }
  if (numeric) {
    return v;
  }
  return "\"" + json_escape(v) + "\"";
}

template <typename T>
void opt_json(std::ostringstream & os, const char * name,
              const std::optional<T> & v, bool & first)
{
  if (!first) {os << ",";}
  first = false;
  os << "\"" << name << "\":";
  if (v) {
    os << *v;
  } else {
    os << "null";
  }
}

}  // namespace

const char * record_kind_name(RecordKind kind)
{
  switch (kind) {
    case RecordKind::kEstatHot: return "estat_hot";
    case RecordKind::kEstatCold: return "estat_cold";
    case RecordKind::kUstat: return "ustat";
    case RecordKind::kLatency: return "latency";
    case RecordKind::kEvent: return "event";
    case RecordKind::kETx: return "e_tx";
  }
  return "unknown";
}

const std::map<std::string, bool> & Whitelist::allowed(RecordKind kind)
{
  switch (kind) {
    case RecordKind::kEstatHot: return hot_keys();
    case RecordKind::kEstatCold: return cold_keys();
    case RecordKind::kUstat: return ustat_keys();
    case RecordKind::kLatency: return latency_keys();
    case RecordKind::kEvent: return event_keys();
    case RecordKind::kETx: return etx_keys();
  }
  return hot_keys();
}

std::vector<WhitelistedRecord> Whitelist::project(
  const CaptureRecord & capture, uint64_t first_record_seq) const
{
  std::vector<WhitelistedRecord> out;
  auto base = [&](RecordKind kind) {
    WhitelistedRecord rec;
    rec.kind = kind;
    rec.capture_seq = capture.capture_seq;
    rec.host_mono_ns = capture.host_mono_ns;
    rec.tx_ok = capture.tx_ok;
    rec.bytes_written = capture.bytes_written;
    return rec;
  };

  switch (capture.type) {
    case CaptureType::kEstatHot:
    case CaptureType::kEstat: {
      // Both frame types share the ESTAT tag length family.
      const size_t tag_len =
        capture.type == CaptureType::kEstatHot ? 8 : 6;
      const KvList kv = parse_kv(capture.line, tag_len);
      WhitelistedRecord hot = base(RecordKind::kEstatHot);
      pick_fields(hot, kv, allowed(RecordKind::kEstatHot));
      hot.tool_id = find_i64(kv, "tool");
      hot.mcu_sample_seq = find_u64(kv, "sample_seq");
      hot.mcu_sample_us = find_i64(kv, "sample_us");
      hot.mcu_boot_id = find_u64(kv, "boot_id");
      hot.reset_epoch = find_u32(kv, "reset_epoch");
      if (hot.fields.size() != allowed(RecordKind::kEstatHot).size()) {
        hot.valid = false;
        hot.invalid_reason = "incomplete hot fields";
      }
      hot.record_seq = first_record_seq;
      out.push_back(std::move(hot));

      if (capture.type == CaptureType::kEstat) {
        WhitelistedRecord cold = base(RecordKind::kEstatCold);
        pick_fields(cold, kv, allowed(RecordKind::kEstatCold));
        cold.tool_id = find_i64(kv, "tool");
        cold.mcu_sample_seq = find_u64(kv, "sample_seq");
        cold.mcu_sample_us = find_i64(kv, "sample_us");
        cold.mcu_boot_id = find_u64(kv, "boot_id");
        cold.reset_epoch = find_u32(kv, "reset_epoch");
        if (cold.fields.size() != allowed(RecordKind::kEstatCold).size()) {
          cold.valid = false;
          cold.invalid_reason = "incomplete cold fields";
        }
        cold.record_seq = first_record_seq + 1;
        out.push_back(std::move(cold));
      }
      break;
    }
    case CaptureType::kUstat: {
      const KvList kv = parse_kv(capture.line, 6);
      WhitelistedRecord rec = base(RecordKind::kUstat);
      pick_fields(rec, kv, allowed(RecordKind::kUstat));
      rec.mcu_sample_seq = find_u64(kv, "sample_seq");
      rec.mcu_sample_us = find_i64(kv, "sample_us");
      rec.mcu_boot_id = find_u64(kv, "boot_id");
      if (rec.fields.size() != allowed(RecordKind::kUstat).size()) {
        rec.valid = false;
        rec.invalid_reason = "incomplete ustat fields";
      }
      rec.record_seq = first_record_seq;
      out.push_back(std::move(rec));
      break;
    }
    case CaptureType::kEvent: {
      // "EVACK <seq> <type> ..." / "EVDONE <seq> <type> ..." / "EVERR ..."
      std::istringstream ss(capture.line);
      std::string tag, type;
      uint64_t seq = 0;
      ss >> tag >> seq >> type;
      WhitelistedRecord rec = base(RecordKind::kEvent);
      if (tag == "EVACK") {
        rec.fields.emplace_back("event_stage", "EVACK");
      } else if (tag == "EVDONE") {
        rec.fields.emplace_back("event_stage", "EVDONE");
      } else {
        rec.fields.emplace_back("event_stage", "EVERR");
      }
      rec.fields.emplace_back("event_type", type);
      rec.event_request_id = seq;
      // Last token is the MCU event timestamp (microseconds).
      std::string tok, last;
      while (ss >> tok) {last = tok;}
      if (!last.empty()) {
        try {
          rec.mcu_event_us = std::stoll(last);
        } catch (...) {
        }
      }
      rec.record_seq = first_record_seq;
      out.push_back(std::move(rec));
      break;
    }
    case CaptureType::kTx: {
      // Host TX: only "E <seq> <tool> <e_abs>" lines carry the whitelist
      // business field. Other TX lines (EV commands etc.) are captured
      // for ordering but produce no business record.
      std::istringstream ss(capture.line);
      std::string tag;
      ss >> tag;
      if (tag != "E") {
        break;
      }
      uint64_t seq = 0;
      int64_t tool = 0;
      std::string e_abs;
      ss >> seq >> tool >> e_abs;
      if (e_abs.empty()) {
        WhitelistedRecord rec = base(RecordKind::kETx);
        rec.valid = false;
        rec.invalid_reason = "unparseable E line";
        rec.record_seq = first_record_seq;
        out.push_back(std::move(rec));
        break;
      }
      WhitelistedRecord rec = base(RecordKind::kETx);
      rec.fields.emplace_back("last_extrude_abs", e_abs);
      rec.tool_id = tool;
      rec.record_seq = first_record_seq;
      out.push_back(std::move(rec));
      break;
    }
    default:
      break;
  }
  return out;
}

std::string encode_record_json(
  const WhitelistedRecord & rec,
  const std::string & run_id,
  const std::string & producer_id,
  uint64_t producer_epoch)
{
  std::ostringstream os;
  os << "{";
  os << "\"schema_version\":1";
  os << ",\"run_id\":\"" << json_escape(run_id) << "\"";
  os << ",\"producer_id\":\"" << json_escape(producer_id) << "\"";
  os << ",\"producer_epoch\":" << producer_epoch;
  os << ",\"record_seq\":" << rec.record_seq;
  os << ",\"capture_seq\":" << rec.capture_seq;
  os << ",\"kind\":\"" << record_kind_name(rec.kind) << "\"";
  os << ",\"host_mono_ns\":" << rec.host_mono_ns;
  bool first = false;
  opt_json(os, "mcu_boot_id", rec.mcu_boot_id, first);
  opt_json(os, "mcu_sample_seq", rec.mcu_sample_seq, first);
  opt_json(os, "mcu_sample_us", rec.mcu_sample_us, first);
  opt_json(os, "mcu_event_us", rec.mcu_event_us, first);
  opt_json(os, "tool_id", rec.tool_id, first);
  opt_json(os, "event_request_id", rec.event_request_id, first);
  opt_json(os, "reset_epoch", rec.reset_epoch, first);
  os << ",\"valid\":" << (rec.valid ? "true" : "false");
  if (!rec.valid) {
    os << ",\"invalid_reason\":\"" << json_escape(rec.invalid_reason) << "\"";
  }
  os << ",\"tx_ok\":" << (rec.tx_ok ? "true" : "false");
  os << ",\"bytes_written\":" << rec.bytes_written;
  os << ",\"data\":{";
  bool first_field = true;
  for (const auto & f : rec.fields) {
    if (!first_field) {os << ",";}
    first_field = false;
    os << "\"" << json_escape(f.first) << "\":" << scalar_json(f.second);
  }
  os << "}}";
  return os.str();
}

}  // namespace runtime_logging
