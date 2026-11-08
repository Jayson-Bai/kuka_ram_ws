#include "runtime_logging_core/rsi_parser.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

namespace runtime_logging
{

namespace
{

struct View
{
  const char * p;
  size_t n;
};

bool is_name_char(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') || c == '_' || c == '.';
}

/* Find "<tag" or "<tag " bounded element open; returns offset just
 * past the tag name, or npos. Only matches direct element names
 * inside <Rob ...>, i.e. we scan for '<' + exact tag + delimiter. */
size_t find_tag(const View & v, const char * tag)
{
  const size_t tag_len = std::strlen(tag);
  for (size_t i = 0; i + 1 < v.n; ++i) {
    if (v.p[i] != '<') {
      continue;
    }
    if (i + 1 + tag_len >= v.n) {
      break;
    }
    if (v.p[i + 1] == '/' || v.p[i + 1] == '!' || v.p[i + 1] == '?') {
      continue;
    }
    if (std::strncmp(v.p + i + 1, tag, tag_len) != 0) {
      continue;
    }
    const char next = v.p[i + 1 + tag_len];
    // Tag must be followed by whitespace, '/', or '>'.
    if (next == ' ' || next == '\t' || next == '\r' || next == '\n' ||
      next == '/' || next == '>')
    {
      return i + 1 + tag_len;
    }
  }
  return std::string::npos;
}

/* Find the '>' that closes the open tag starting at pos. */
size_t tag_close(const View & v, size_t pos)
{
  char quote = 0;
  for (size_t i = pos; i < v.n; ++i) {
    const char c = v.p[i];
    if (quote != 0) {
      if (c == quote) {
        quote = 0;
      }
      continue;
    }
    if (c == '"' || c == '\'') {
      quote = c;
      continue;
    }
    if (c == '>') {
      return i;
    }
  }
  return std::string::npos;
}

/* Extract attribute "name" from the open-tag region [pos, close).
 * Accepts single or double quotes, arbitrary whitespace around '='.
 * Returns the value extent [vstart, vend) inside the view. */
bool find_attr(
  const View & v, size_t pos, size_t close, const char * name,
  size_t & vstart, size_t & vend)
{
  const size_t name_len = std::strlen(name);
  for (size_t i = pos; i + name_len < close; ++i) {
    if (std::strncmp(v.p + i, name, name_len) != 0) {
      continue;
    }
    // Name boundary on both sides.
    if (i > pos && is_name_char(v.p[i - 1])) {
      continue;
    }
    size_t j = i + name_len;
    while (j < close && (v.p[j] == ' ' || v.p[j] == '\t')) {
      ++j;
    }
    if (j >= close || v.p[j] != '=') {
      continue;
    }
    ++j;
    while (j < close && (v.p[j] == ' ' || v.p[j] == '\t')) {
      ++j;
    }
    if (j >= close || (v.p[j] != '"' && v.p[j] != '\'')) {
      continue;
    }
    const char q = v.p[j];
    ++j;
    const size_t vs = j;
    while (j < close && v.p[j] != q) {
      ++j;
    }
    if (j >= close) {
      return false;
    }
    vstart = vs;
    vend = j;
    return true;
  }
  return false;
}

/* Strict locale-independent double parse: full-consumption,
 * finite result required (rejects NaN/Inf/trailing garbage). */
bool parse_double(const View & v, size_t s, size_t e, double & out)
{
  if (s >= e) {
    return false;
  }
  std::string tmp(v.p + s, e - s);
  char * end = nullptr;
  errno = 0;
  const double val = std::strtod(tmp.c_str(), &end);
  if (end != tmp.c_str() + tmp.size()) {
    return false;
  }
  if (errno == ERANGE || !std::isfinite(val)) {
    return false;
  }
  out = val;
  return true;
}

bool parse_i64(const View & v, size_t s, size_t e, int64_t & out)
{
  if (s >= e) {
    return false;
  }
  std::string tmp(v.p + s, e - s);
  char * end = nullptr;
  errno = 0;
  const long long val = std::strtoll(tmp.c_str(), &end, 10);
  if (end != tmp.c_str() + tmp.size() || errno == ERANGE) {
    return false;
  }
  out = static_cast<int64_t>(val);
  return true;
}

/* Parse six joint attributes A1..A6 of an element opened at pos. */
bool parse_six_axes(
  const View & v, size_t pos, size_t close,
  std::array<double, 6> & out)
{
  static const char * names[6] = {"A1", "A2", "A3", "A4", "A5", "A6"};
  bool ok = true;
  for (int i = 0; i < 6; ++i) {
    size_t vs, ve;
    if (!find_attr(v, pos, close, names[i], vs, ve) ||
      !parse_double(v, vs, ve, out[i]))
    {
      ok = false;
    }
  }
  return ok;
}

/* Extract text of a simple <TAG>text</TAG> element. */
bool element_text(
  const View & v, const char * tag, size_t & ts, size_t & te)
{
  const size_t open = find_tag(v, tag);
  if (open == std::string::npos) {
    return false;
  }
  const size_t close = tag_close(v, open);
  if (close == std::string::npos) {
    return false;
  }
  // Build closing tag "</TAG>".
  std::string ct = "</";
  ct += tag;
  ct += '>';
  for (size_t i = close + 1; i + ct.size() <= v.n; ++i) {
    if (std::strncmp(v.p + i, ct.c_str(), ct.size()) == 0) {
      ts = close + 1;
      te = i;
      return true;
    }
  }
  return false;
}

}  // namespace

RsiFeedbackView parse_rsi_feedback(const char * data, size_t rx_len)
{
  RsiFeedbackView out;
  const View v{data, rx_len};

  // ---- Pos ----
  {
    static const char * pos_names[6] = {"X", "Y", "Z", "A", "B", "C"};
    const size_t pos = find_tag(v, "Pos");
    if (pos == std::string::npos) {
      out.parse_flags |= kFlagMissingPos;
    } else {
      const size_t close = tag_close(v, pos);
      if (close == std::string::npos) {
        out.parse_flags |= kFlagMalformed;
      } else {
        bool ok = true;
        for (int i = 0; i < 6; ++i) {
          size_t vs, ve;
          if (!find_attr(v, pos, close, pos_names[i], vs, ve) ||
            !parse_double(v, vs, ve, out.pos[i]))
          {
            ok = false;
          }
        }
        out.pos_valid = ok;

        size_t vs, ve;
        const bool s_ok =
          find_attr(v, pos, close, "S", vs, ve) &&
          parse_i64(v, vs, ve, out.pos_s);
        const bool t_ok =
          find_attr(v, pos, close, "T", vs, ve) &&
          parse_i64(v, vs, ve, out.pos_t);
        out.pos_st_valid = s_ok && t_ok;
      }
    }
  }

  // ---- AIPos ----
  {
    const size_t pos = find_tag(v, "AIPos");
    if (pos == std::string::npos) {
      out.parse_flags |= kFlagMissingAipos;
    } else {
      const size_t close = tag_close(v, pos);
      if (close == std::string::npos) {
        out.parse_flags |= kFlagMalformed;
      } else {
        out.aipos_valid = parse_six_axes(v, pos, close, out.aipos);
      }
    }
  }

  // ---- ASPos ----
  {
    const size_t pos = find_tag(v, "ASPos");
    if (pos == std::string::npos) {
      out.parse_flags |= kFlagMissingAspos;
    } else {
      const size_t close = tag_close(v, pos);
      if (close == std::string::npos) {
        out.parse_flags |= kFlagMalformed;
      } else {
        out.aspos_valid = parse_six_axes(v, pos, close, out.aspos);
      }
    }
  }

  // ---- Delay ----
  {
    const size_t pos = find_tag(v, "Delay");
    if (pos == std::string::npos) {
      out.parse_flags |= kFlagMissingDelay;
    } else {
      const size_t close = tag_close(v, pos);
      size_t vs, ve;
      if (close == std::string::npos ||
        !find_attr(v, pos, close, "D", vs, ve))
      {
        out.parse_flags |= kFlagMalformed;
      } else {
        out.delay_raw.assign(v.p + vs, ve - vs);
        if (!out.delay_raw.empty() &&
          parse_i64(v, vs, ve, out.delay_count) &&
          out.delay_count >= 0)
        {
          out.delay_valid = true;
        }
        // Empty or negative Delay is a parse-level invalid signal,
        // not silently zero.
      }
    }
  }

  // ---- IPOC ----
  {
    size_t ts, te;
    if (element_text(v, "IPOC", ts, te)) {
      out.ipoc_raw.assign(v.p + ts, te - ts);
      out.ipoc_valid = !out.ipoc_raw.empty();
    } else {
      out.parse_flags |= kFlagMissingIpoc;
    }
  }

  return out;
}

}  // namespace runtime_logging
