#include <gtest/gtest.h>

#include <locale>
#include <stdexcept>
#include <limits>
#include <string>

#include "uart_bridge/extrusion_wire.hpp"

namespace
{

class CommaUnderscoreNumpunct : public std::numpunct<char>
{
protected:
  char do_decimal_point() const override
  {
    return ',';
  }

  char do_thousands_sep() const override
  {
    return '_';
  }

  std::string do_grouping() const override
  {
    return "\3";
  }
};

TEST(ExtrusionWireMode, ParsesCanonicalV1)
{
  EXPECT_EQ(
    uart_bridge::parse_extrusion_wire_mode("canonical_v1"),
    uart_bridge::ExtrusionWireMode::CanonicalV1);
}

TEST(ExtrusionWireMode, ParsesLegacyV1)
{
  EXPECT_EQ(
    uart_bridge::parse_extrusion_wire_mode("legacy_v1"),
    uart_bridge::ExtrusionWireMode::LegacyV1);
}

TEST(ExtrusionWireMode, RejectsUnknownMode)
{
  EXPECT_THROW(
    uart_bridge::parse_extrusion_wire_mode("unknown"),
    std::invalid_argument);
}

TEST(CanonicalExtrusionWire, FormatsFixedSixCommand)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(42U, 3, 1.2);

  EXPECT_EQ(preparation.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(preparation.candidate.has_value());
  EXPECT_EQ(preparation.candidate->line, "E 42 3 1.200000\n");
}

TEST(CanonicalExtrusionWire, RejectsNonFiniteAndOutOfRangeWithoutStateChange)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto nan = forwarder.prepare(
    1U, 1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(nan.decision, uart_bridge::ExtrusionDecision::RejectNonFinite);
  EXPECT_FALSE(nan.candidate.has_value());

  const auto positive_infinity = forwarder.prepare(
    2U, 1, std::numeric_limits<double>::infinity());
  EXPECT_EQ(
    positive_infinity.decision,
    uart_bridge::ExtrusionDecision::RejectNonFinite);
  EXPECT_FALSE(positive_infinity.candidate.has_value());

  const auto negative_infinity = forwarder.prepare(
    3U, 1, -std::numeric_limits<double>::infinity());
  EXPECT_EQ(
    negative_infinity.decision,
    uart_bridge::ExtrusionDecision::RejectNonFinite);
  EXPECT_FALSE(negative_infinity.candidate.has_value());

  const auto out_of_range = forwarder.prepare(4U, 1, 1.0e20);
  EXPECT_EQ(
    out_of_range.decision,
    uart_bridge::ExtrusionDecision::RejectOutOfRange);
  EXPECT_FALSE(out_of_range.candidate.has_value());

  const auto zero = forwarder.prepare(5U, 1, 0.0);
  EXPECT_EQ(
    zero.decision,
    uart_bridge::ExtrusionDecision::SuppressInitialZero);
  EXPECT_FALSE(zero.candidate.has_value());
}

TEST(CanonicalExtrusionWire, FormatsNegativeValues)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(5U, 2, -0.00000051);

  EXPECT_EQ(preparation.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(preparation.candidate.has_value());
  EXPECT_EQ(preparation.candidate->line, "E 5 2 -0.000001\n");
}

TEST(CanonicalExtrusionWire, SuppressesInitialNegativeQuantizedZero)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(1U, 1, -0.0000004);

  EXPECT_EQ(
    preparation.decision,
    uart_bridge::ExtrusionDecision::SuppressInitialZero);
  EXPECT_FALSE(preparation.candidate.has_value());
}

TEST(CanonicalExtrusionWire, NeverUsesScientificNotation)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(2U, 2, 1234567.25);

  EXPECT_EQ(preparation.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(preparation.candidate.has_value());
  EXPECT_EQ(preparation.candidate->line, "E 2 2 1234567.250000\n");
}

TEST(CanonicalExtrusionWire, IgnoresGlobalNumericLocale)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);
  const std::locale previous_locale = std::locale();
  std::locale::global(std::locale(previous_locale, new CommaUnderscoreNumpunct));

  uart_bridge::ExtrusionPreparation preparation;
  try {
    preparation = forwarder.prepare(1234U, 2, 12.5);
  } catch (...) {
    std::locale::global(previous_locale);
    throw;
  }
  std::locale::global(previous_locale);

  EXPECT_EQ(preparation.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(preparation.candidate.has_value());
  EXPECT_EQ(preparation.candidate->line, "E 1234 2 12.500000\n");
}

TEST(CanonicalExtrusionWire, SuppressesValuesBelowHalfOfSixthDecimal)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(1U, 1, 0.00000049);

  EXPECT_EQ(
    preparation.decision,
    uart_bridge::ExtrusionDecision::SuppressInitialZero);
  EXPECT_FALSE(preparation.candidate.has_value());
}

TEST(CanonicalExtrusionWire, RoundsValuesAboveHalfOfSixthDecimal)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto preparation = forwarder.prepare(2U, 1, 0.00000051);

  EXPECT_EQ(preparation.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(preparation.candidate.has_value());
  EXPECT_EQ(preparation.candidate->line, "E 2 1 0.000001\n");
}

TEST(CanonicalExtrusionWire, DoesNotSuppressUntilCandidateIsCommitted)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);

  const auto first = forwarder.prepare(10U, 1, 2.0000001);

  EXPECT_EQ(first.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(first.candidate.has_value());

  const auto retry = forwarder.prepare(11U, 1, 2.0000004);

  EXPECT_EQ(retry.decision, uart_bridge::ExtrusionDecision::Send);
  EXPECT_TRUE(retry.candidate.has_value());

  forwarder.commit(*first.candidate);
  const auto duplicate = forwarder.prepare(12U, 1, 2.0000004);

  EXPECT_EQ(
    duplicate.decision,
    uart_bridge::ExtrusionDecision::SuppressDuplicate);
  EXPECT_FALSE(duplicate.candidate.has_value());
}

TEST(CanonicalExtrusionWire, ToolChangeAndReturnToZeroAreForwarded)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);
  const auto first = forwarder.prepare(1U, 1, 3.0);
  ASSERT_EQ(first.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(first.candidate.has_value());
  forwarder.commit(*first.candidate);

  const auto tool_change = forwarder.prepare(2U, 2, 3.0);

  EXPECT_EQ(tool_change.decision, uart_bridge::ExtrusionDecision::Send);
  EXPECT_TRUE(tool_change.candidate.has_value());

  const auto zero = forwarder.prepare(3U, 1, 0.0);

  EXPECT_EQ(zero.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(zero.candidate.has_value());
  EXPECT_EQ(zero.candidate->line, "E 3 1 0.000000\n");
}

TEST(CanonicalExtrusionWire, ResetRestoresInitialZeroSuppression)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::CanonicalV1);
  const auto first = forwarder.prepare(1U, 1, 1.0);
  ASSERT_EQ(first.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(first.candidate.has_value());
  forwarder.commit(*first.candidate);

  forwarder.reset();
  const auto zero = forwarder.prepare(2U, 1, 0.0);

  EXPECT_EQ(
    zero.decision,
    uart_bridge::ExtrusionDecision::SuppressInitialZero);
  EXPECT_FALSE(zero.candidate.has_value());
}

}  // namespace

TEST(LegacyExtrusionWire, KeepsOldFloatTextAndDoubleEpsilonDedup)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::LegacyV1);

  const auto first = forwarder.prepare(7U, 2, 1.25);
  EXPECT_EQ(first.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(first.candidate.has_value());
  EXPECT_EQ(first.candidate->line, "E 7 2 1.25\n");
  forwarder.commit(*first.candidate);

  const auto duplicate = forwarder.prepare(8U, 2, 1.25 + 0.5e-9);
  EXPECT_EQ(
    duplicate.decision,
    uart_bridge::ExtrusionDecision::SuppressDuplicate);
  EXPECT_FALSE(duplicate.candidate.has_value());

  const auto changed = forwarder.prepare(9U, 2, 1.25 + 2.0e-9);
  EXPECT_EQ(changed.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(changed.candidate.has_value());

  const auto tool_change = forwarder.prepare(9U, 3, 1.25 + 2.0e-9);
  EXPECT_EQ(tool_change.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(tool_change.candidate.has_value());
}

TEST(LegacyExtrusionWire, SuppressesInitialNearZero)
{
  const uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::LegacyV1);

  const auto near_zero = forwarder.prepare(1U, 1, 0.5e-9);
  EXPECT_EQ(
    near_zero.decision,
    uart_bridge::ExtrusionDecision::SuppressInitialZero);
  EXPECT_FALSE(near_zero.candidate.has_value());

  const auto send = forwarder.prepare(2U, 1, 2.0e-9);
  EXPECT_EQ(send.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(send.candidate.has_value());
}

TEST(LegacyExtrusionWire, ForwardsReturnToZeroAfterCommit)
{
  uart_bridge::ExtrusionForwarder forwarder(
    uart_bridge::ExtrusionWireMode::LegacyV1);

  const auto first = forwarder.prepare(1U, 2, 1.0);
  ASSERT_EQ(first.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(first.candidate.has_value());
  forwarder.commit(*first.candidate);

  const auto zero = forwarder.prepare(2U, 2, 0.0);

  EXPECT_EQ(zero.decision, uart_bridge::ExtrusionDecision::Send);
  ASSERT_TRUE(zero.candidate.has_value());
  EXPECT_EQ(zero.candidate->line, "E 2 2 0\n");
}
