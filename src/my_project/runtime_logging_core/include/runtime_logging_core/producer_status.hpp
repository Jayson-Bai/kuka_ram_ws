#pragma once

#include <cstdint>
#include <string>

namespace runtime_logging
{

/*
 * Side-channel status frame sent by each producer relay over the same
 * AF_UNIX socket as the data records. It lets the collector account for
 * losses that happen before a record is written (ring drops, IPC queue
 * rejections, send errors, pending bytes) and mark a producer as cleanly
 * ended. Control frames are never written to the JSONL record stream.
 */
struct ProducerStatus
{
  std::string producer_id;
  std::string run_id;
  uint64_t producer_epoch{0};
  // "hello" (first connect), "periodic", or "final" (relay shutdown).
  std::string phase{"periodic"};
  uint64_t first_record_seq{0};
  uint64_t last_record_seq{0};
  // Records successfully queued for IPC (not rejected by the queue).
  uint64_t sent_records{0};
  // Captures dropped before they could become records.
  uint64_t ring_dropped{0};
  // Records rejected locally by the bounded IPC queue.
  uint64_t ipc_dropped{0};
  // Fatal socket send errors observed by the relay.
  uint64_t send_errors{0};
  uint64_t pending_messages{0};
  uint64_t pending_bytes{0};
  bool connected{false};
};

/* Encode a producer status frame as one JSON object (no trailing newline). */
std::string encode_producer_status_json(const ProducerStatus & status);

}  // namespace runtime_logging
