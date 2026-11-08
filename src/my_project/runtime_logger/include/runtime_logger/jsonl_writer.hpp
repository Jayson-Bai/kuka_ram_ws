// =============================================================
// JSONL segment writer: batch append, short-write handling, fsync
// checkpointing, size-based rotation, .partial lifecycle.
// =============================================================

#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <sys/types.h>

namespace runtime_logger
{

struct WriterHealth
{
  uint64_t written_records{0};
  uint64_t durable_records{0};
  uint64_t write_errors{0};
  uint64_t sync_errors{0};
  uint64_t bytes_written{0};
  int64_t last_write_mono_ns{0};
  int64_t last_sync_mono_ns{0};
  std::string last_error;
};

class JsonlWriter
{
public:
  JsonlWriter() = default;
  ~JsonlWriter();

  JsonlWriter(const JsonlWriter &) = delete;
  JsonlWriter & operator=(const JsonlWriter &) = delete;

  bool open_segment(const std::string & dir, uint32_t index);
  // Queue one JSON line. Memory-bounded by caller.
  void queue_line(std::string line);
  // Flush queued lines to the fd (no fsync).
  bool flush();
  // fdatasync and advance durable boundary.
  bool sync();
  // Close the current segment: flush + sync + rename .partial -> final.
  bool close_segment();
  // Abandon without rename (keeps .partial).
  void abort();

  bool is_open() const {return fd_ >= 0;}
  uint64_t segment_bytes() const {return segment_bytes_;}
  size_t queued() const {return queue_.size();}
  const WriterHealth & health() const {return health_;}

private:
  int fd_{-1};
  std::string dir_;
  std::string partial_path_;
  std::string final_path_;
  uint32_t segment_index_{0};
  uint64_t segment_bytes_{0};
  uint64_t pending_records_{0};
  std::deque<std::string> queue_;
  WriterHealth health_;
};

}  // namespace runtime_logger
