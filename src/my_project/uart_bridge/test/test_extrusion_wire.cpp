#include <gtest/gtest.h>

#include <locale>
#include <stdexcept>
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

}  // namespace
