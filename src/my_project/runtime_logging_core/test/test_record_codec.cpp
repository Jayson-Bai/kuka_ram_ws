#include <gtest/gtest.h>
#include "runtime_logging_core/capture_record.hpp"
#include "runtime_logging_core/whitelist.hpp"

using runtime_logging::CaptureType;
using runtime_logging::Direction;
using runtime_logging::classify_line;

TEST(Classify, RxLines)
{
  EXPECT_EQ(classify_line("ESTAT tool=1 x", Direction::kRx), CaptureType::kEstat);
  EXPECT_EQ(classify_line("ESTAT_H tool=1 x", Direction::kRx), CaptureType::kEstatHot);
  EXPECT_EQ(classify_line("USTAT x", Direction::kRx), CaptureType::kUstat);
  EXPECT_EQ(classify_line("STAT x", Direction::kRx), CaptureType::kStat);
  EXPECT_EQ(classify_line("EACK 1 1 0.1", Direction::kRx), CaptureType::kEack);
  EXPECT_EQ(classify_line("EVACK 1 cut", Direction::kRx), CaptureType::kEvent);
  EXPECT_EQ(classify_line("EVDONE 1 cut", Direction::kRx), CaptureType::kEvent);
  EXPECT_EQ(classify_line("junk", Direction::kRx), CaptureType::kUnknown);
}

TEST(Classify, TxLines)
{
  EXPECT_EQ(classify_line("E 1 1 0.1", Direction::kTx), CaptureType::kTx);
  EXPECT_EQ(classify_line("EV 1 cut 0", Direction::kTx), CaptureType::kTx);
}
