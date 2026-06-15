#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "rsi_server/rsi_xml_parser.hpp"

TEST(RsiXmlParser, ParsesPoseAttributesWithoutRegex)
{
  const std::string xml =
    "<Rob Type=\"KUKA\"><RIst X=\"123.456\" Y=\"-7.5\" Z=\"0\" "
    "A=\"180.0\" B=\"-0.25\" C=\"90.125\"/><IPOC>42</IPOC></Rob>";

  const auto pose = rsi_server::parse_kuka_xyzabc(xml);

  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->x, 123.456);
  EXPECT_DOUBLE_EQ(pose->y, -7.5);
  EXPECT_DOUBLE_EQ(pose->z, 0.0);
  EXPECT_DOUBLE_EQ(pose->a, 180.0);
  EXPECT_DOUBLE_EQ(pose->b, -0.25);
  EXPECT_DOUBLE_EQ(pose->c, 90.125);
}

TEST(RsiXmlParser, LeavesMissingAxesAsNan)
{
  const auto pose = rsi_server::parse_kuka_xyzabc("<Rob><RIst X=\"1\" C=\"2\"/></Rob>");

  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->x, 1.0);
  EXPECT_TRUE(std::isnan(pose->y));
  EXPECT_TRUE(std::isnan(pose->z));
  EXPECT_TRUE(std::isnan(pose->a));
  EXPECT_TRUE(std::isnan(pose->b));
  EXPECT_DOUBLE_EQ(pose->c, 2.0);
}

TEST(RsiXmlParser, ReturnsNulloptWhenNoPoseAttributesExist)
{
  EXPECT_FALSE(rsi_server::parse_kuka_xyzabc("<Rob><IPOC>42</IPOC></Rob>").has_value());
}
