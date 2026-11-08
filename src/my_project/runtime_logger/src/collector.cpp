#include "runtime_logger/collector.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <unordered_set>
#include <vector>

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

bool json_bool_field(
  const std::string & body, const char * key, bool & out)
{
  const std::string pat = std::string("\"") + key + "\":";
  auto pos = body.find(pat);
  if (pos == std::string::npos) {
    return false;
  }
  pos += pat.size();
  if (body.compare(pos, 4, "true") == 0) {
    out = true;
    return true;
  }
  if (body.compare(pos, 5, "false") == 0) {
    out = false;
    return true;
  }
  return false;
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
  std::string control;
  if (json_string_field(body, "control", control) &&
    control == "producer_status")
  {
    return handle_producer_status(body);
  }
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

bool Collector::handle_producer_status(const std::string & body)
{
  ProducerStatusUpdate update;
  if (!json_string_field(body, "producer_id", update.producer_id) ||
    !json_string_field(body, "phase", update.phase))
  {
    std::lock_guard<std::mutex> lk(mu_);
    ++stats_.rejected;
    integrity_.note_parse_error("unknown");
    return false;
  }
  json_u64_field(body, "producer_epoch", update.producer_epoch);
  json_u64_field(body, "first_record_seq", update.first_record_seq);
  json_u64_field(body, "last_record_seq", update.last_record_seq);
  json_u64_field(body, "sent_records", update.sent_records);
  json_u64_field(body, "ring_dropped", update.ring_dropped);
  json_u64_field(body, "ipc_dropped", update.ipc_dropped);
  json_u64_field(body, "send_errors", update.send_errors);
  json_u64_field(body, "pending_messages", update.pending_messages);
  json_u64_field(body, "pending_bytes", update.pending_bytes);
  json_bool_field(body, "connected", update.connected);
  std::lock_guard<std::mutex> lk(mu_);
  integrity_.note_producer_status(update);
  return true;
}

void Collector::accept_loop()
{
  const int epfd = ::epoll_create1(EPOLL_CLOEXEC);
  if (epfd < 0) {
    return;
  }
  epoll_event listen_ev{};
  listen_ev.events = EPOLLIN;
  listen_ev.data.fd = listen_fd_;
  if (::epoll_ctl(epfd, EPOLL_CTL_ADD, listen_fd_, &listen_ev) != 0) {
    ::close(epfd);
    return;
  }

  std::unordered_set<int> clients;
  uint8_t buf[kMaxFrame];
  constexpr int kMaxEvents = 64;
  constexpr int kMaxFramesPerEvent = 64;
  epoll_event events[kMaxEvents];

  while (running_.load()) {
    const int ready = ::epoll_wait(epfd, events, kMaxEvents, 100);
    if (ready < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    for (int i = 0; i < ready && running_.load(); ++i) {
      const int fd = events[i].data.fd;
      if (fd == listen_fd_) {
        while (running_.load()) {
          const int client = ::accept4(
            listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
          if (client < 0) {
            break;
          }
          epoll_event client_ev{};
          client_ev.events = EPOLLIN;
          client_ev.data.fd = client;
          if (::epoll_ctl(epfd, EPOLL_CTL_ADD, client, &client_ev) != 0) {
            ::close(client);
            continue;
          }
          clients.insert(client);
        }
        continue;
      }

      bool close_client = false;
      for (int k = 0; k < kMaxFramesPerEvent && running_.load(); ++k) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n < 0) {
          if (errno == EINTR) {continue;}
          if (errno != EAGAIN && errno != EWOULDBLOCK) {
            close_client = true;
          }
          break;
        }
        if (n == 0) {
          close_client = true;
          break;
        }
        handle_frame(buf, static_cast<size_t>(n));
      }
      if (close_client) {
        ::epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
        ::close(fd);
        clients.erase(fd);
      }
    }
  }

  for (const int fd : clients) {
    ::epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
    ::close(fd);
  }
  ::close(epfd);
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
      const size_t line_bytes = writer_queue_.front().size();
      writer_.queue_line(std::move(writer_queue_.front()));
      queued_bytes_ -= std::min(queued_bytes_, line_bytes);
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
