#include "uart_bridge/extrusion_wire.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace uart_bridge
{
namespace
{

constexpr std::uint64_t kCanonicalUnitsPerWhole = 1'000'000U;
constexpr double kLegacyEpsilon = 1e-9;

std::uint64_t unsigned_magnitude(std::int64_t value) noexcept
{
  if (value >= 0) {
    return static_cast<std::uint64_t>(value);
  }
  return static_cast<std::uint64_t>(-(value + 1)) + 1U;
}

std::string make_canonical_line(
  std::uint32_t seq_used, std::int32_t tool_id, std::int64_t canonical_e_nm)
{
  const std::uint64_t magnitude = unsigned_magnitude(canonical_e_nm);
  const std::uint64_t whole = magnitude / kCanonicalUnitsPerWhole;
  const std::uint64_t fraction = magnitude % kCanonicalUnitsPerWhole;

  std::ostringstream command;
  command.imbue(std::locale::classic());
  command << "E " << seq_used << ' ' << tool_id << ' ';
  if (canonical_e_nm < 0) {
    command << '-';
  }
  command << whole << '.' << std::setfill('0') << std::setw(6) << fraction << '\n';
  return command.str();
}

}  // namespace

ExtrusionWireMode parse_extrusion_wire_mode(const std::string & value)
{
  if (value == "canonical_v1") {
    return ExtrusionWireMode::CanonicalV1;
  }
  if (value == "legacy_v1") {
    return ExtrusionWireMode::LegacyV1;
  }
  throw std::invalid_argument("unsupported extrusion wire mode: " + value);
}

const char * extrusion_wire_mode_name(ExtrusionWireMode mode) noexcept
{
  switch (mode) {
    case ExtrusionWireMode::CanonicalV1:
      return "canonical_v1";
    case ExtrusionWireMode::LegacyV1:
      return "legacy_v1";
  }
  return "unknown";
}

ExtrusionForwarder::ExtrusionForwarder(ExtrusionWireMode mode) noexcept
: mode_(mode)
{
}

ExtrusionPreparation ExtrusionForwarder::prepare(
  std::uint32_t seq_used, std::int32_t tool_id, double scaled_e_abs) const
{

  if (mode_ == ExtrusionWireMode::LegacyV1) {
    if (!last_sent_valid_ && std::abs(scaled_e_abs) <= kLegacyEpsilon) {
      return {ExtrusionDecision::SuppressInitialZero, std::nullopt};
    }
    if (
      last_sent_valid_ && last_sent_tool_id_ == tool_id &&
      std::abs(last_sent_legacy_e_abs_ - scaled_e_abs) <= kLegacyEpsilon)
    {
      return {ExtrusionDecision::SuppressDuplicate, std::nullopt};
    }

    std::ostringstream command;
    command << "E " << seq_used << " " << tool_id << " " <<
      static_cast<float>(scaled_e_abs) << "\n";

    ExtrusionCandidate candidate;
    candidate.line = command.str();
    candidate.tool_id = tool_id;
    candidate.scaled_e_abs = scaled_e_abs;
    return {ExtrusionDecision::Send, candidate};
  }

  if (!std::isfinite(scaled_e_abs)) {
    return {ExtrusionDecision::RejectNonFinite, std::nullopt};
  }

  const long double units = static_cast<long double>(scaled_e_abs) * 1'000'000.0L;
  if (!std::isfinite(units)) {
    return {ExtrusionDecision::RejectOutOfRange, std::nullopt};
  }

  const long double rounded_units = std::round(units);
  const long double int64_limit = std::ldexp(1.0L, 63);
  if (rounded_units < -int64_limit || rounded_units >= int64_limit) {
    return {ExtrusionDecision::RejectOutOfRange, std::nullopt};
  }

  const auto canonical_e_nm = static_cast<std::int64_t>(rounded_units);
  if (!last_sent_valid_ && canonical_e_nm == 0) {
    return {ExtrusionDecision::SuppressInitialZero, std::nullopt};
  }
  if (
    last_sent_valid_ && last_sent_tool_id_ == tool_id &&
    last_sent_canonical_e_nm_ == canonical_e_nm)
  {
    return {ExtrusionDecision::SuppressDuplicate, std::nullopt};
  }

  ExtrusionCandidate candidate;
  candidate.line = make_canonical_line(seq_used, tool_id, canonical_e_nm);
  candidate.tool_id = tool_id;
  candidate.canonical_e_nm = canonical_e_nm;
  candidate.scaled_e_abs = scaled_e_abs;
  return {ExtrusionDecision::Send, candidate};
}

void ExtrusionForwarder::commit(const ExtrusionCandidate & candidate) noexcept
{
  last_sent_valid_ = true;
  last_sent_tool_id_ = candidate.tool_id;
  last_sent_canonical_e_nm_ = candidate.canonical_e_nm;
  last_sent_legacy_e_abs_ = candidate.scaled_e_abs;
}

void ExtrusionForwarder::reset() noexcept
{
  last_sent_valid_ = false;
  last_sent_tool_id_ = 0;
  last_sent_canonical_e_nm_ = 0;
  last_sent_legacy_e_abs_ = 0.0;
}

ExtrusionWireMode ExtrusionForwarder::mode() const noexcept
{
  return mode_;
}

}  // namespace uart_bridge
