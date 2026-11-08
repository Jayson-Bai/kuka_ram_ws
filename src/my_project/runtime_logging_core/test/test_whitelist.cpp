#include <gtest/gtest.h>
#include "runtime_logging_core/whitelist.hpp"

using runtime_logging::CaptureRecord;
using runtime_logging::CaptureType;
using runtime_logging::Direction;
using runtime_logging::RecordKind;
using runtime_logging::Whitelist;
using runtime_logging::encode_record_json;
using runtime_logging::record_kind_name;

static CaptureRecord make(Direction d, CaptureType t, const std::string & line)
{
  CaptureRecord r;
  r.direction = d;
  r.type = t;
  r.line = line;
  r.host_mono_ns = 123456789;
  r.capture_seq = 7;
  return r;
}

TEST(Whitelist, FullEstatProducesHotAndCold)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kEstat,
    "ESTAT tool=2 state=RUN target_steps=1000 submitted_steps=990 "
    "completed_steps=950 host_total_um=12345 last_cmd_us=999 "
    "ff_until_us=888 ff_sps=800.000 v_sps=850.000 phase=0.123456 "
    "last_seq=200 fault=0x00000000 hard_fault=0x00000000 "
    "rmt_submit_err=0 rmt_done_err=0 qdepth=4 dir=1 enabled=1 "
    "hold_latched=0 host_valid=1 boot_id=11 sample_seq=42 "
    "sample_us=777 reset_epoch=3 active=1");
  auto recs = wl.project(rec, 100);
  ASSERT_EQ(recs.size(), 2u);
  EXPECT_EQ(recs[0].kind, RecordKind::kEstatHot);
  EXPECT_EQ(recs[1].kind, RecordKind::kEstatCold);
  EXPECT_EQ(recs[0].record_seq, 100u);
  EXPECT_EQ(recs[1].record_seq, 101u);
  EXPECT_EQ(recs[0].fields.size(), 7u);
  EXPECT_EQ(recs[1].fields.size(), 3u);
  EXPECT_EQ(recs[0].mcu_sample_seq.value(), 42u);
  EXPECT_EQ(recs[0].mcu_sample_us.value(), 777);
  EXPECT_EQ(recs[0].reset_epoch.value(), 3u);
}

TEST(Whitelist, HotFrameProducesOnlyHot)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kEstatHot,
    "ESTAT_H tool=1 state=DECEL target_steps=10 submitted_steps=9 "
    "last_seq=5 ff_sps=1.0 v_sps=2.0 fault=0x00000001 boot_id=1 "
    "sample_seq=9 sample_us=55 reset_epoch=0 active=1");
  auto recs = wl.project(rec, 0);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].kind, RecordKind::kEstatHot);
}

TEST(Whitelist, NonWhitelistedKeysDropped)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kUstat,
    "USTAT fifo_ovf=1 buffer_full=2 line_drop=3 cmd_drop=4 old_seq=5 "
    "seq_gap=6 parse_err=7 secret_key=999 boot_id=1 sample_seq=2 "
    "sample_us=3");
  auto recs = wl.project(rec, 0);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].fields.size(), 5u);
  for (const auto & f : recs[0].fields) {
    EXPECT_NE(f.first, "cmd_drop");
    EXPECT_NE(f.first, "old_seq");
    EXPECT_NE(f.first, "secret_key");
  }
}

TEST(Whitelist, EventStageAndId)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kEvent,
    "EVDONE 17 cut done 123456");
  auto recs = wl.project(rec, 0);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].kind, RecordKind::kEvent);
  EXPECT_EQ(recs[0].event_request_id.value(), 17u);
  bool saw_stage = false, saw_type = false;
  for (const auto & f : recs[0].fields) {
    if (f.first == "event_stage") {saw_stage = true; EXPECT_EQ(f.second, "EVDONE");}
    if (f.first == "event_type") {saw_type = true; EXPECT_EQ(f.second, "cut");}
  }
  EXPECT_TRUE(saw_stage);
  EXPECT_TRUE(saw_type);
}

TEST(Whitelist, ETxOnlyE)
{
  Whitelist wl;
  auto good = make(Direction::kTx, CaptureType::kTx, "E 42 1 12.345678");
  auto recs = wl.project(good, 5);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].kind, RecordKind::kETx);
  EXPECT_EQ(recs[0].fields.size(), 1u);
  EXPECT_EQ(recs[0].fields[0].second, "12.345678");

  auto ev = make(Direction::kTx, CaptureType::kTx, "EV 3 cut 10");
  auto ev_recs = wl.project(ev, 5);
  EXPECT_TRUE(ev_recs.empty());
}

TEST(Whitelist, JsonEncoding)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kEstatHot,
    "ESTAT_H tool=2 state=RUN target_steps=-5 submitted_steps=0 "
    "last_seq=1 ff_sps=0.5 v_sps=-0.5 fault=0xdeadbeef boot_id=9 "
    "sample_seq=3 sample_us=44 reset_epoch=2 active=1");
  auto recs = wl.project(rec, 0);
  ASSERT_EQ(recs.size(), 1u);
  const std::string j = encode_record_json(recs[0], "run-abc", "uart_node", 1);
  EXPECT_NE(j.find("\"kind\":\"estat_hot\""), std::string::npos);
  EXPECT_NE(j.find("\"run_id\":\"run-abc\""), std::string::npos);
  EXPECT_NE(j.find("\"target_steps\":-5"), std::string::npos);
  EXPECT_NE(j.find("\"fault\":\"0xdeadbeef\""), std::string::npos);
  EXPECT_NE(j.find("\"mcu_sample_seq\":3"), std::string::npos);
}

TEST(Whitelist, MissingFieldMarkedInvalid)
{
  Whitelist wl;
  auto rec = make(
    Direction::kRx, CaptureType::kEstatHot,
    "ESTAT_H tool=1 state=RUN target_steps=1");
  auto recs = wl.project(rec, 0);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_FALSE(recs[0].valid);
}

TEST(Whitelist, KindNames)
{
  EXPECT_STREQ(record_kind_name(RecordKind::kLatency), "latency");
  EXPECT_STREQ(record_kind_name(RecordKind::kETx), "e_tx");
}
