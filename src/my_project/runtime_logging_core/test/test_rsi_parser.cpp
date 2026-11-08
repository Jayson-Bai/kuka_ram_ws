#include <gtest/gtest.h>
#include <cstring>
#include "runtime_logging_core/rsi_parser.hpp"

using runtime_logging::RsiFeedbackView;
using runtime_logging::parse_rsi_feedback;
using runtime_logging::kFlagMissingAipos;
using runtime_logging::kFlagMissingAspos;
using runtime_logging::kFlagMissingDelay;
using runtime_logging::kFlagMissingIpoc;
using runtime_logging::kFlagMissingPos;
using runtime_logging::kFlagMalformed;

static RsiFeedbackView parse(const char * s)
{
  return parse_rsi_feedback(s, std::strlen(s));
}

TEST(RsiParser, OldPosOnly)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1.0000\" Y=\"2.0000\" Z=\"3.0000\" A=\"4.0000\" B=\"5.0000\" C=\"6.0000\" S=\"2\" T=\"35\"/>"
    "<IPOC>12345678</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_TRUE(v.pos_valid);
  EXPECT_DOUBLE_EQ(v.pos[0], 1.0);
  EXPECT_DOUBLE_EQ(v.pos[5], 6.0);
  EXPECT_TRUE(v.pos_st_valid);
  EXPECT_EQ(v.pos_s, 2);
  EXPECT_EQ(v.pos_t, 35);
  EXPECT_TRUE(v.ipoc_valid);
  EXPECT_EQ(v.ipoc_raw, "12345678");
  EXPECT_TRUE(v.parse_flags & kFlagMissingAipos);
  EXPECT_TRUE(v.parse_flags & kFlagMissingAspos);
  EXPECT_TRUE(v.parse_flags & kFlagMissingDelay);
  EXPECT_FALSE(v.parse_flags & kFlagMissingPos);
}

TEST(RsiParser, NewFieldsAllValid)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1.0\" Y=\"2.0\" Z=\"3.0\" A=\"4.0\" B=\"5.0\" C=\"6.0\" S=\"2\" T=\"35\"/>"
    "<AIPos A1=\"0.0000\" A2=\"-90.0000\" A3=\"90.0000\" A4=\"0.0000\" A5=\"90.0000\" A6=\"0.0000\"/>"
    "<ASPos A1=\"0.0000\" A2=\"-90.0000\" A3=\"90.0000\" A4=\"0.0000\" A5=\"90.0000\" A6=\"0.0000\"/>"
    "<Delay D=\"0\"/>"
    "<IPOC>87654321</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_TRUE(v.pos_valid);
  EXPECT_TRUE(v.aipos_valid);
  EXPECT_TRUE(v.aspos_valid);
  EXPECT_TRUE(v.delay_valid);
  EXPECT_EQ(v.delay_count, 0);
  EXPECT_TRUE(v.ipoc_valid);
  EXPECT_EQ(v.ipoc_raw, "87654321");
  EXPECT_EQ(v.parse_flags, 0u);
  EXPECT_DOUBLE_EQ(v.aipos[1], -90.0);
  EXPECT_DOUBLE_EQ(v.aspos[2], 90.0);
}

TEST(RsiParser, AttrOrderAndWhitespace)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos  C=\"6\"   B = \"5\" A='4' Z=\"3\" Y=\"2\" X=\"1\" T=\"35\" S=\"2\" />"
    "<AIPos A6=\"6\" A5=\"5\" A4=\"4\" A3=\"3\" A2=\"2\" A1=\"1\"/>"
    "<ASPos A6=\"60\" A5=\"50\" A4=\"40\" A3=\"30\" A2=\"20\" A1=\"10\"/>"
    "<Delay  D = '7' />"
    "<IPOC>99</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_TRUE(v.pos_valid);
  EXPECT_DOUBLE_EQ(v.pos[0], 1.0);
  EXPECT_DOUBLE_EQ(v.pos[3], 4.0);
  EXPECT_TRUE(v.aipos_valid);
  EXPECT_DOUBLE_EQ(v.aipos[0], 1.0);
  EXPECT_DOUBLE_EQ(v.aipos[5], 6.0);
  EXPECT_TRUE(v.aspos_valid);
  EXPECT_DOUBLE_EQ(v.aspos[0], 10.0);
  EXPECT_TRUE(v.delay_valid);
  EXPECT_EQ(v.delay_count, 7);
}

TEST(RsiParser, ScientificNotation)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1e-3\" Y=\"-2.5E+2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"1e-2\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"10\" A2=\"20\" A3=\"30\" A4=\"40\" A5=\"50\" A6=\"60\"/>"
    "<Delay D=\"0\"/>"
    "<IPOC>1</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_TRUE(v.pos_valid);
  EXPECT_DOUBLE_EQ(v.pos[0], 0.001);
  EXPECT_DOUBLE_EQ(v.pos[1], -250.0);
  EXPECT_TRUE(v.aipos_valid);
  EXPECT_DOUBLE_EQ(v.aipos[0], 0.01);
}

TEST(RsiParser, MissingAxisInvalid)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\"/>"
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"0\"/>"
    "<IPOC>1</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_FALSE(v.pos_valid);
  EXPECT_FALSE(v.aipos_valid);  // A6 missing
  EXPECT_TRUE(v.aspos_valid);
  EXPECT_TRUE(v.delay_valid);
}

TEST(RsiParser, RejectNaNAndInf)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"nan\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"inf\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"0\"/>"
    "<IPOC>1</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_FALSE(v.pos_valid);
  EXPECT_FALSE(v.aipos_valid);
}

TEST(RsiParser, EmptyDelayInvalid)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"\"/>"
    "<IPOC>1</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_FALSE(v.delay_valid);
  EXPECT_EQ(v.delay_raw, "");
}

TEST(RsiParser, NegativeDelayInvalid)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"-3\"/>"
    "<IPOC>1</IPOC>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_FALSE(v.delay_valid);
}

TEST(RsiParser, MissingIpoc)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"0\"/>"
    "</Rob>";
  auto v = parse(xml);
  EXPECT_TRUE(v.parse_flags & kFlagMissingIpoc);
  EXPECT_FALSE(v.ipoc_valid);
}

TEST(RsiParser, MalformedUnclosedTag)
{
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\""
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<IPOC>1</IPOC>";
  auto v = parse(xml);
  // Unclosed Pos tag means A1..C attrs bleed into the tag region but
  // the tag close '>' is found later at AIPos close; parser is best-effort.
  // Main assertion: no crash, flags may or may not be set.
  (void)v;
}

TEST(RsiParser, BoundedLengthNotNullTerminated)
{
  // Feed a buffer where the XML is not NUL-terminated: only first N bytes
  // are the frame; a trailing garbage byte must not be read.
  char buf[256];
  const char * xml =
    "<Rob TYPE=\"KUKA\">"
    "<Pos X=\"1\" Y=\"2\" Z=\"3\" A=\"4\" B=\"5\" C=\"6\" S=\"0\" T=\"0\"/>"
    "<AIPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<ASPos A1=\"1\" A2=\"2\" A3=\"3\" A4=\"4\" A5=\"5\" A6=\"6\"/>"
    "<Delay D=\"2\"/>"
    "<IPOC>42</IPOC>"
    "</Rob>";
  const size_t n = std::strlen(xml);
  std::memcpy(buf, xml, n);
  buf[n] = '\xde';  // garbage sentinel, parser must not read it
  auto v = parse_rsi_feedback(buf, n);
  EXPECT_TRUE(v.pos_valid);
  EXPECT_TRUE(v.aipos_valid);
  EXPECT_TRUE(v.ipoc_valid);
  EXPECT_EQ(v.ipoc_raw, "42");
}
