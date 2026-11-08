#include "runtime_logging_core/whitelist.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "runtime_logging_core/rsi_parser.hpp"

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

const std::map<std::string, bool> & rsi_keys()
{
  static const std::map<std::string, bool> keys = {
    {"rx_index", true},
    {"rx_mono_ns", true},
    {"tx_mono_ns", true},
    {"rx_bytes", true},
    {"tx_bytes", true},
    {"rx_truncated", true},
    {"send_ok", true},
    {"send_errno", true},
    {"startup_fast_reply", true},
    {"state", true},
    {"seq_used", true},
    {"seq_valid", true},
    {"path_id", true},
    {"tool_id", true},
    {"cmd_x", true},
    {"cmd_y", true},
    {"cmd_z", true},
    {"cmd_a", true},
    {"cmd_b", true},
    {"cmd_c", true},
    {"cmd_e", true},
    {"ipoc_raw", true},
    {"ipoc_valid", true},
    {"pos_valid", true},
    {"pos_x", true},
    {"pos_y", true},
    {"pos_z", true},
    {"pos_a", true},
    {"pos_b", true},
    {"pos_c", true},
    {"pos_s", true},
    {"pos_t", true},
    {"aipos_valid", true},
    {"a1_deg", true},
    {"a2_deg", true},
    {"a3_deg", true},
    {"a4_deg", true},
    {"a5_deg", true},
    {"a6_deg", true},
    {"aspos_valid", true},
    {"s1_deg", true},
    {"s2_deg", true},
    {"s3_deg", true},
    {"s4_deg", true},
    {"s5_deg", true},
    {"s6_deg", true},
    {"delay_valid", true},
    {"delay_count", true},
    {"parse_flags", true},
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
    case RecordKind::kRsi: return "rsi";
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
    case RecordKind::kRsi: return rsi_keys();
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

namespace
{

void f_u64(WhitelistedRecord & r, const char * k, uint64_t v)
{
  r.fields.emplace_back(k, std::to_string(v));
}

void f_i64(WhitelistedRecord & r, const char * k, int64_t v)
{
  r.fields.emplace_back(k, std::to_string(v));
}

void f_bool(WhitelistedRecord & r, const char * k, bool v)
{
  r.fields.emplace_back(k, v ? "1" : "0");
}

void f_str(WhitelistedRecord & r, const char * k, const std::string & v)
{
  r.fields.emplace_back(k, v);
}

void f_dbl(WhitelistedRecord & r, const char * k, double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  r.fields.emplace_back(k, buf);
}

void f_dbl_opt(WhitelistedRecord & r, const char * k, bool valid, double v)
{
  if (!valid) {
    r.fields.emplace_back(k, "");
    return;
  }
  f_dbl(r, k, v);
}

}  // namespace

WhitelistedRecord project_rsi_frame(
  const RsiTelemetryFrame & frame,
  uint64_t record_seq)
{
  WhitelistedRecord rec;
  rec.kind = RecordKind::kRsi;
  rec.record_seq = record_seq;
  rec.capture_seq = frame.rx_index;
  rec.host_mono_ns = frame.rx_mono_ns;
  rec.tx_ok = frame.send_ok;
  rec.bytes_written = frame.tx_len;

  const RsiFeedbackView fb =
    parse_rsi_feedback(frame.rx.data(), frame.rx_len);

  f_u64(rec, "rx_index", frame.rx_index);
  f_i64(rec, "rx_mono_ns", frame.rx_mono_ns);
  f_i64(rec, "tx_mono_ns", frame.tx_mono_ns);
  f_u64(rec, "rx_bytes", frame.rx_len);
  f_u64(rec, "tx_bytes", frame.tx_len);
  f_bool(rec, "rx_truncated", frame.rx_truncated);
  f_bool(rec, "send_ok", frame.send_ok);
  f_i64(rec, "send_errno", frame.send_errno);
  f_bool(rec, "startup_fast_reply", frame.startup_fast_reply);
  f_u64(rec, "state", frame.state);

  f_u64(rec, "seq_used", frame.seq_used);
  f_bool(rec, "seq_valid", frame.seq_valid);
  f_u64(rec, "path_id", frame.path_id);
  f_i64(rec, "tool_id", frame.tool_id);
  f_dbl(rec, "cmd_x", frame.cmd_x);
  f_dbl(rec, "cmd_y", frame.cmd_y);
  f_dbl(rec, "cmd_z", frame.cmd_z);
  f_dbl(rec, "cmd_a", frame.cmd_a);
  f_dbl(rec, "cmd_b", frame.cmd_b);
  f_dbl(rec, "cmd_c", frame.cmd_c);
  f_dbl(rec, "cmd_e", frame.cmd_e);

  f_str(rec, "ipoc_raw", fb.ipoc_raw);
  f_bool(rec, "ipoc_valid", fb.ipoc_valid);

  f_bool(rec, "pos_valid", fb.pos_valid);
  f_dbl_opt(rec, "pos_x", fb.pos_valid, fb.pos[0]);
  f_dbl_opt(rec, "pos_y", fb.pos_valid, fb.pos[1]);
  f_dbl_opt(rec, "pos_z", fb.pos_valid, fb.pos[2]);
  f_dbl_opt(rec, "pos_a", fb.pos_valid, fb.pos[3]);
  f_dbl_opt(rec, "pos_b", fb.pos_valid, fb.pos[4]);
  f_dbl_opt(rec, "pos_c", fb.pos_valid, fb.pos[5]);
  f_i64(rec, "pos_s", fb.pos_s);
  f_i64(rec, "pos_t", fb.pos_t);

  f_bool(rec, "aipos_valid", fb.aipos_valid);
  f_dbl_opt(rec, "a1_deg", fb.aipos_valid, fb.aipos[0]);
  f_dbl_opt(rec, "a2_deg", fb.aipos_valid, fb.aipos[1]);
  f_dbl_opt(rec, "a3_deg", fb.aipos_valid, fb.aipos[2]);
  f_dbl_opt(rec, "a4_deg", fb.aipos_valid, fb.aipos[3]);
  f_dbl_opt(rec, "a5_deg", fb.aipos_valid, fb.aipos[4]);
  f_dbl_opt(rec, "a6_deg", fb.aipos_valid, fb.aipos[5]);

  f_bool(rec, "aspos_valid", fb.aspos_valid);
  f_dbl_opt(rec, "s1_deg", fb.aspos_valid, fb.aspos[0]);
  f_dbl_opt(rec, "s2_deg", fb.aspos_valid, fb.aspos[1]);
  f_dbl_opt(rec, "s3_deg", fb.aspos_valid, fb.aspos[2]);
  f_dbl_opt(rec, "s4_deg", fb.aspos_valid, fb.aspos[3]);
  f_dbl_opt(rec, "s5_deg", fb.aspos_valid, fb.aspos[4]);
  f_dbl_opt(rec, "s6_deg", fb.aspos_valid, fb.aspos[5]);

  f_bool(rec, "delay_valid", fb.delay_valid);
  if (fb.delay_valid) {
    f_i64(rec, "delay_count", fb.delay_count);
  } else {
    rec.fields.emplace_back("delay_count", "");
  }

  uint32_t flags = fb.parse_flags;
  if (frame.rx_truncated) {
    flags |= kFlagTruncatedRx;
  }
  f_u64(rec, "parse_flags", flags);

  if (rec.fields.size() != rsi_keys().size()) {
    rec.valid = false;
    rec.invalid_reason = "incomplete rsi fields";
  }

  return rec;
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
