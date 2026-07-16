#ifndef UART_BRIDGE__EXTRUSION_WIRE_HPP_
#define UART_BRIDGE__EXTRUSION_WIRE_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace uart_bridge
{

enum class ExtrusionWireMode
{
  CanonicalV1,
  LegacyV1
};

enum class ExtrusionDecision
{
  Send,
  SuppressInitialZero,
  SuppressDuplicate,
  RejectNonFinite,
  RejectOutOfRange
};

struct ExtrusionCandidate
{
  std::string line;
  int32_t tool_id{0};
  int64_t canonical_e_nm{0};
  double scaled_e_abs{0.0};
};

struct ExtrusionPreparation
{
  ExtrusionDecision decision{ExtrusionDecision::SuppressDuplicate};
  std::optional<ExtrusionCandidate> candidate;
};

ExtrusionWireMode parse_extrusion_wire_mode(const std::string & value);
const char * extrusion_wire_mode_name(ExtrusionWireMode mode) noexcept;

class ExtrusionForwarder
{
public:
  explicit ExtrusionForwarder(ExtrusionWireMode mode) noexcept;
  ExtrusionPreparation prepare(
    uint32_t seq_used, int32_t tool_id, double scaled_e_abs) const;
  void commit(const ExtrusionCandidate & candidate) noexcept;
  void reset() noexcept;
  ExtrusionWireMode mode() const noexcept;

private:
  ExtrusionWireMode mode_;
  bool last_sent_valid_{false};
  int32_t last_sent_tool_id_{0};
  int64_t last_sent_canonical_e_nm_{0};
  double last_sent_legacy_e_abs_{0.0};
};

}  // namespace uart_bridge

#endif  // UART_BRIDGE__EXTRUSION_WIRE_HPP_
