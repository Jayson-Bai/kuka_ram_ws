#include "runtime_logger/jsonl_writer.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>

namespace runtime_logger
{

namespace
{

std::string segment_name(uint32_t index, bool partial)
{
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "records_%06u.jsonl%s", index,
    partial ? ".partial" : "");
  return buf;
}

}  // namespace

JsonlWriter::~JsonlWriter()
{
  abort();
}

bool JsonlWriter::open_segment(const std::string & dir, uint32_t index)
{
  if (fd_ >= 0) {
    return false;
  }
  dir_ = dir;
  segment_index_ = index;
  partial_path_ = dir + "/" + segment_name(index, true);
  final_path_ = dir + "/" + segment_name(index, false);
  fd_ = ::open(
    partial_path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd_ < 0) {
    health_.last_error = ::strerror(errno);
    ++health_.write_errors;
    return false;
  }
  segment_bytes_ = 0;
  return true;
}

void JsonlWriter::queue_line(std::string line)
{
  queue_.push_back(std::move(line));
}

bool JsonlWriter::flush()
{
  if (fd_ < 0) {
    return false;
  }
  while (!queue_.empty()) {
    const std::string & line = queue_.front();
    const char * p = line.data();
    size_t remaining = line.size();
    while (remaining > 0) {
      const ssize_t n = ::write(fd_, p, remaining);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        health_.last_error = ::strerror(errno);
        ++health_.write_errors;
        return false;
      }
      p += n;
      remaining -= static_cast<size_t>(n);
    }
    segment_bytes_ += line.size();
    health_.bytes_written += line.size();
    ++health_.written_records;
    ++pending_records_;
    queue_.pop_front();
  }
  health_.last_write_mono_ns = 0;  // set by caller via clock
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  health_.last_write_mono_ns =
    static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
  return true;
}

bool JsonlWriter::sync()
{
  if (fd_ < 0) {
    return false;
  }
  if (::fdatasync(fd_) != 0) {
    health_.last_error = ::strerror(errno);
    ++health_.sync_errors;
    return false;
  }
  health_.durable_records += pending_records_;
  pending_records_ = 0;
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  health_.last_sync_mono_ns =
    static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
  return true;
}

bool JsonlWriter::close_segment()
{
  if (fd_ < 0) {
    return false;
  }
  bool ok = flush();
  if (ok) {
    ok = sync();
  }
  ::close(fd_);
  fd_ = -1;
  if (ok && ::rename(partial_path_.c_str(), final_path_.c_str()) != 0) {
    health_.last_error = ::strerror(errno);
    ++health_.write_errors;
    ok = false;
  }
  return ok;
}

void JsonlWriter::abort()
{
  queue_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace runtime_logger
