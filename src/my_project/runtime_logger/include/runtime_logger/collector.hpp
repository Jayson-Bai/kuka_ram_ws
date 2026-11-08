#pragma once

#include "runtime_logger/integrity.hpp"
#include "runtime_logger/jsonl_writer.hpp"
#include "runtime_logger/session_manifest.hpp"

#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace runtime_logger
{

/*
 * AF_UNIX/SOCK_SEQPACKET collector. Accepts framed records from
 * producers, verifies them, and hands them to the writer thread.
 */
class Collector
{
public:
  Collector() = default;
  ~Collector();

  bool start(
    const std::string & socket_path,
    const std::string & run_dir,
    const std::string & run_id,
    size_t writer_queue_bytes);
  void stop();

  struct Stats
  {
    uint64_t accepted{0};
    uint64_t rejected{0};
    uint64_t gaps{0};
    uint64_t written{0};
    uint64_t durable{0};
    uint64_t write_errors{0};
    uint64_t queued_bytes{0};
    bool session_active{false};
  };
  Stats stats() const;
  Integrity & integrity() {return integrity_;}
  bool all_producers_ended() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return integrity_.all_producers_ended();
  }
  uint64_t total_gaps() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return integrity_.total_gaps();
  }
  uint64_t total_producer_drops() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return integrity_.total_producer_drops();
  }
  uint64_t total_parse_errors() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    uint64_t n = 0;
    for (const auto & kv : integrity_.accounts()) {
      n += kv.second.parse_errors;
    }
    return n;
  }
  std::string integrity_json(bool complete) const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return integrity_.to_json(complete);
  }
  JsonlWriter & writer() {return writer_;}

private:
  void accept_loop();
  void writer_loop();
  bool handle_frame(const uint8_t * data, size_t n);
  bool handle_producer_status(const std::string & body);

  int listen_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  std::thread writer_thread_;
  std::string socket_path_;
  std::string run_dir_;
  std::string run_id_;
  size_t writer_queue_bytes_{8u * 1024u * 1024u};

  mutable std::mutex mu_;
  std::deque<std::string> writer_queue_;
  size_t queued_bytes_{0};
  std::condition_variable cv_;

  Integrity integrity_;
  JsonlWriter writer_;
  uint32_t segment_index_{0};
  uint64_t rotate_bytes_{64ull * 1024ull * 1024ull};
  uint64_t batch_bytes_{64ull * 1024ull};
  int flush_period_ms_{100};
  int sync_period_ms_{1000};

  Stats stats_;
};

}  // namespace runtime_logger
