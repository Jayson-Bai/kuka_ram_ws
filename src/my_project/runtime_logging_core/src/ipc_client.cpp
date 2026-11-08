#include "runtime_logging_core/ipc_client.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>

namespace runtime_logging
{

namespace
{

constexpr size_t kMaxMessage = 4096;

}  // namespace

IpcClient::~IpcClient()
{
  close();
}

bool IpcClient::connect_to(const std::string & path)
{
  close();
  if (path.size() >= sizeof(sockaddr_un::sun_path)) {
    return false;
  }
  fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  if (fd_ < 0) {
    return false;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
  if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  last_errno_ = 0;
  return true;
}

void IpcClient::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool IpcClient::queue_message(std::string payload)
{
  if (payload.size() > kMaxMessage) {
    ++dropped_;
    return false;
  }
  if (pending_.size() >= max_pending_ ||
    pending_bytes_ + payload.size() > max_pending_bytes_)
  {
    ++dropped_;
    return false;
  }
  pending_bytes_ += payload.size();
  pending_.push_back(std::move(payload));
  return true;
}

size_t IpcClient::drain()
{
  if (fd_ < 0) {
    return pending_.size();
  }
  while (!pending_.empty()) {
    const std::string & msg = pending_.front();
    const ssize_t n = ::send(fd_, msg.data(), msg.size(), MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      if (errno == EINTR) {
        continue;
      }
      // Fatal: drop the fd but keep pending so a reconnect can retry.
      ++send_errors_;
      last_errno_ = errno;
      close();
      break;
    }
    if (static_cast<size_t>(n) != msg.size()) {
      // SEQPACKET sends are all-or-nothing; a short send means the
      // connection state is no longer trustworthy.
      ++send_errors_;
      last_errno_ = EMSGSIZE;
      close();
      break;
    }
    pending_bytes_ -= msg.size();
    pending_.erase(pending_.begin());
    ++sent_;
    sent_bytes_ += static_cast<uint64_t>(n);
  }
  return pending_.size();
}

}  // namespace runtime_logging
