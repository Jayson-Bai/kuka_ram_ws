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
  uint64_t sent() const {return sent_;}
  uint64_t dropped() const {return dropped_;}

  void set_max_pending(size_t n) {max_pending_ = n;}

private:
  int fd_{-1};
  std::vector<std::string> pending_;
  size_t max_pending_{4096};
  uint64_t sent_{0};
  uint64_t dropped_{0};
};

}  // namespace runtime_logging
