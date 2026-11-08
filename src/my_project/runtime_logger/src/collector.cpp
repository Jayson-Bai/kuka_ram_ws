#include "runtime_logger/collector.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace runtime_logger
{

namespace
{

constexpr size_t kMaxFrame = 4096;

/* Minimal JSON field extraction for envelope fields we need for
 * accounting. The whitelist/projection already happened producer-side;
 * the collector only needs producer identity and record_seq. */
bool json_string_field(
  const std::string & body, const char * key, std::string & out)
{
  const std::string pat = std::string("\"") + key + "\":\"";
  auto pos = body.find(pat);
  if (pos == std::string::npos) {
    return false;
  }
  pos += pat.size();
  auto end = body.find('"', pos);
  if (end == std::string::npos) {
    return false;
  }
  out = body.substr(pos, end - pos);
  return true;
}

bool json_u64_field(
  const std::string & body, const char * key, uint64_t & out)
{
  const std::string pat = std::string("\"") + key + "\":";
  auto pos = body.find(pat);
  if (pos == std::string::npos) {
    return false;
  }
  pos += pat.size();
  try {
    out = std::stoull(body.substr(pos));
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

Collector::~Collector()
{
  stop();
}

bool Collector::start(
  const std::string & socket_path,
  const std::string & run_dir,
  const std::string & run_id,
  size_t writer_queue_bytes)
{
  socket_path_ = socket_path;
  run_dir_ = run_dir;
  run_id_ = run_id;
  writer_queue_bytes_ = writer_queue_bytes;

  listen_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  if (listen_fd_ < 0) {
    return false;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socket_path_.c_str());
  ::unlink(socket_path_.c_str());
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  ::chmod(socket_path_.c_str(), 0600);
  if (::listen(listen_fd_, 8) != 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  // Open the first segment before producers connect so records
  // are never silently dropped into a queue with no writer.
  if (!writer_.open_segment(run_dir_, 0)) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    ::unlink(socket_path_.c_str());
    return false;
  }
  segment_index_ = 0;
  running_.store(true);
  accept_thread_ = std::thread(&Collector::accept_loop, this);
  writer_thread_ = std::thread(&Collector::writer_loop, this);
  return true;
}

void Collector::stop()
{
  running_.store(false);
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  cv_.notify_all();
  if (accept_thread_.joinable()) {accept_thread_.join();}
  if (writer_thread_.joinable()) {writer_thread_.join();}
  // Seal the last segment: flush any records the writer thread did
  // not get to before joining, then close so the .partial suffix is
  // removed and the file reflects a clean shutdown boundary.
  {
    std::lock_guard<std::mutex> lk(mu_);
    while (!writer_queue_.empty()) {
      writer_.queue_line(std::move(writer_queue_.front()));
      writer_queue_.pop_front();
    }
  }
  writer_.flush();
  writer_.sync();
  if (writer_.is_open()) {
    writer_.close_segment();
  }
  if (!socket_path_.empty()) {
    ::unlink(socket_path_.c_str());
  }
}

Collector::Stats Collector::stats() const
{
  std::lock_guard<std::mutex> lk(mu_);
  Stats s = stats_;
  s.queued_bytes = queued_bytes_;
  return s;
}

bool Collector::handle_frame(const uint8_t * data, size_t n)
{
  std::string body(reinterpret_cast<const char *>(data), n);
  std::string producer_id;
  uint64_t record_seq = 0, capture_seq = 0;
  const bool ok =
    json_string_field(body, "producer_id", producer_id) &&
    json_u64_field(body, "record_seq", record_seq) &&
    json_u64_field(body, "capture_seq", capture_seq);
  if (!ok) {
    std::lock_guard<std::mutex> lk(mu_);
    ++stats_.rejected;
    integrity_.note_parse_error("unknown");
    return false;
  }
  body += '\n';
  std::lock_guard<std::mutex> lk(mu_);
  integrity_.note_record(producer_id, record_seq, capture_seq, true);
  if (queued_bytes_ + body.size() > writer_queue_bytes_) {
    ++stats_.rejected;
    integrity_.note_overflow(1);
    return false;
  }
  writer_queue_.push_back(std::move(body));
  queued_bytes_ += body.size();
  ++stats_.accepted;
  cv_.notify_one();
  return true;
}

void Collector::accept_loop()
{
  while (running_.load()) {
    sockaddr_un peer{};
    socklen_t len = sizeof(peer);
    const int fd = ::accept4(
      listen_fd_, reinterpret_cast<sockaddr *>(&peer), &len,
      SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    uint8_t buf[kMaxFrame];
    while (running_.load()) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (n == 0) {
        break;
      }
      handle_frame(buf, static_cast<size_t>(n));
    }
    ::close(fd);
  }
}

void Collector::writer_loop()
{
  auto last_flush = std::chrono::steady_clock::now();
  auto last_sync = std::chrono::steady_clock::now();
  while (running_.load()) {
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait_for(lk, std::chrono::milliseconds(flush_period_ms_), [&] {
      return !writer_queue_.empty() || !running_.load();
    });
    const auto now = std::chrono::steady_clock::now();
    const bool flush_due =
      now - last_flush >= std::chrono::milliseconds(flush_period_ms_);
    const bool sync_due =
      now - last_sync >= std::chrono::milliseconds(sync_period_ms_);
    const bool rotate_due =
      writer_.is_open() && writer_.segment_bytes() >= rotate_bytes_;
    const bool queue_due = queued_bytes_ >= batch_bytes_;

    if (writer_queue_.empty() && !flush_due && !sync_due && !rotate_due &&
      running_.load())
    {
      continue;
    }

    // Move the deque into the writer's own queue under lock.
    size_t moved = 0;
    (void)moved;
    while (!writer_queue_.empty()) {
      writer_.queue_line(std::move(writer_queue_.front()));
      queued_bytes_ -= writer_queue_.front().size();
      writer_queue_.pop_front();
      ++moved;
    }
    lk.unlock();

    const uint64_t written_before = writer_.health().written_records;
    if (flush_due || queue_due || rotate_due || !running_.load()) {
      writer_.flush();
      last_flush = now;
      const uint64_t written_after = writer_.health().written_records;
      if (written_after > written_before) {
        integrity_.note_write(written_after - written_before);
      }
    }
    if (sync_due || rotate_due || !running_.load()) {
      writer_.sync();
      last_sync = now;
    }
    if (rotate_due) {
      writer_.close_segment();
      writer_.open_segment(run_dir_, ++segment_index_);
    }

    {
      std::lock_guard<std::mutex> lk2(mu_);
      const auto & h = writer_.health();
      stats_.written = h.written_records;
      stats_.durable = h.durable_records;
      stats_.write_errors = h.write_errors + h.sync_errors;
    }
  }
}

}  // namespace runtime_logger
