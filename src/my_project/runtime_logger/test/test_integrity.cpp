#include <gtest/gtest.h>

#include "runtime_logger/integrity.hpp"

using runtime_logger::Integrity;
using runtime_logger::ProducerStatusUpdate;

namespace
{

ProducerStatusUpdate status(
  const std::string & id, const std::string & phase, uint64_t sent)
{
  ProducerStatusUpdate s;
  s.producer_id = id;
  s.producer_epoch = 1;
  s.phase = phase;
  s.sent_records = sent;
  s.last_record_seq = sent > 0 ? sent - 1 : 0;
  return s;
}

}  // namespace

TEST(Integrity, DetectsGapsAndProducerLoss)
{
  Integrity integ;
  integ.note_record("uart_node", 0, 0, true);
  integ.note_record("uart_node", 1, 1, true);
  integ.note_record("uart_node", 3, 3, true);
  integ.note_producer_status(status("uart_node", "final", 4));

  EXPECT_EQ(integ.accounts().at("uart_node").gap_count, 1u);
  // 4 sent, 3 received -> one more missing record.
  EXPECT_EQ(integ.total_producer_drops(), 1u);
  EXPECT_EQ(integ.total_gaps(), 2u);
  EXPECT_TRUE(integ.all_producers_ended());
}

TEST(Integrity, CountsRingAndIpcDrops)
{
  Integrity integ;
  auto st = status("rsi_node", "final", 0);
  st.ring_dropped = 3;
  st.ipc_dropped = 2;
  integ.note_producer_status(st);

  EXPECT_EQ(integ.total_producer_drops(), 5u);
  EXPECT_EQ(integ.total_gaps(), 5u);
}

TEST(Integrity, UnendedProducerBlocksCompletion)
{
  Integrity integ;
  integ.note_producer_status(status("rsi_node", "hello", 0));
  EXPECT_FALSE(integ.all_producers_ended());
}
