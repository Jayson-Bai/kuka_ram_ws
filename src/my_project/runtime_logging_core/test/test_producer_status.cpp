#include <gtest/gtest.h>

#include "runtime_logging_core/producer_status.hpp"

using runtime_logging::encode_producer_status_json;
using runtime_logging::ProducerStatus;

TEST(ProducerStatus, EncodesControlEnvelope)
{
  ProducerStatus s;
  s.producer_id = "rsi_node";
  s.run_id = "abc\"def";
  s.producer_epoch = 3;
  s.phase = "final";
  s.first_record_seq = 10;
  s.last_record_seq = 19;
  s.sent_records = 10;
  s.ring_dropped = 2;
  s.ipc_dropped = 1;
  s.send_errors = 4;
  s.pending_messages = 5;
  s.pending_bytes = 512;
  s.connected = true;

  const std::string json = encode_producer_status_json(s);
  EXPECT_NE(json.find("\"control\":\"producer_status\""), std::string::npos);
  EXPECT_NE(json.find("\"producer_id\":\"rsi_node\""), std::string::npos);
  EXPECT_NE(json.find("\"run_id\":\"abc\\\"def\""), std::string::npos);
  EXPECT_NE(json.find("\"phase\":\"final\""), std::string::npos);
  EXPECT_NE(json.find("\"sent_records\":10"), std::string::npos);
  EXPECT_NE(json.find("\"ring_dropped\":2"), std::string::npos);
  EXPECT_NE(json.find("\"ipc_dropped\":1"), std::string::npos);
  EXPECT_NE(json.find("\"connected\":true"), std::string::npos);
}
