#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace runtime_logging
{

/*
 * AF_UNIX/SOCK_SEQPACKET client used by background relay threads.
 * All sends are non-blocking; when the socket is back-pressured the
 * message is kept in a bounded pending deque for the next drain attempt.
 *
 * On a fatal send error the fd is closed so the caller's reconnect path
 * can run again; already-queued messages are retained and retried on the
 * next connection. Every rejected message and every fatal send error is
 * counted so losses never happen silently.
 */
class IpcClient
{
public:
  IpcClient() = default;
  ~IpcClient();

  IpcClient(const IpcClient &) = delete;
  IpcClient & operator=(const IpcClient &) = delete;

  bool connect_to(const std::string & path);
  void close();
  bool connected() const {return fd_ >= 0;}

  // Queue a complete framed message. Never blocks.
  bool queue_message(std::string payload);

  // Attempt to flush pending messages. Returns bytes still pending.
  size_t drain();

  size_t pending() const {return pending_.size();}
  size_t pending_bytes() const {return pending_bytes_;}
  uint64_t sent() const {return sent_;}
  uint64_t sent_bytes() const {return sent_bytes_;}
  uint64_t dropped() const {return dropped_;}
  uint64_t send_errors() const {return send_errors_;}
  int last_errno() const {return last_errno_;}

  void set_max_pending(size_t n) {max_pending_ = n;}
  void set_max_pending_bytes(size_t n) {max_pending_bytes_ = n;}

private:
  int fd_{-1};
  std::vector<std::string> pending_;
  size_t pending_bytes_{0};
  size_t max_pending_{4096};
  size_t max_pending_bytes_{16u * 1024u * 1024u};
  uint64_t sent_{0};
  uint64_t sent_bytes_{0};
  uint64_t dropped_{0};
  uint64_t send_errors_{0};
  int last_errno_{0};
};

}  // namespace runtime_logging
