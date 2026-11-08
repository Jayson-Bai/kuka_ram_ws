#include "runtime_logger/session_manifest.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <sstream>

namespace runtime_logger
{

namespace
{

bool mkdir_p(const std::string & path)
{
  if (::mkdir(path.c_str(), 0755) == 0) {
    return true;
  }
  return errno == EEXIST;
}

}  // namespace

bool SessionManifest::start(
  const std::string & root_dir,
  const std::string & run_id,
  const std::string & code_version,
  const std::string & config_json,
  bool test_run)
{
  run_id_ = run_id;
  run_dir_ = root_dir + "/run_" + run_id;
  if (!mkdir_p(root_dir) || !mkdir_p(run_dir_)) {
    return false;
  }
  if (!acquire_lock()) {
    return false;
  }
  std::ostringstream os;
  os << "{"
     << "\"schema_version\":1"
     << ",\"run_id\":\"" << run_id << "\""
     << ",\"code_version\":\"" << code_version << "\""
     << ",\"test_run\":" << (test_run ? "true" : "false")
     << ",\"config\":" << (config_json.empty() ? "{}" : config_json)
     << ",\"complete\":false"
     << "}";
  if (!write_manifest_atomic(os.str())) {
    release_lock();
    return false;
  }
  active_ = true;
  return true;
}

bool SessionManifest::finalize(
  bool complete,
  const std::string & integrity_json,
  const std::string & end_reason)
{
  if (!active_) {
    return false;
  }
  write_file_atomic(run_dir_ + "/integrity.json", integrity_json);
  std::ostringstream os;
  os << "{"
     << "\"schema_version\":1"
     << ",\"run_id\":\"" << run_id_ << "\""
     << ",\"end_reason\":\"" << end_reason << "\""
     << ",\"complete\":" << (complete ? "true" : "false")
     << "}";
  // Rewrite the manifest with the final flag.
  write_manifest_atomic(os.str());
  active_ = false;
  release_lock();
  return true;
}

bool SessionManifest::acquire_lock()
{
  const std::string path = run_dir_ + "/run_dir.lock";
  lock_fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
  if (lock_fd_ < 0) {
    return false;
  }
  if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(lock_fd_);
    lock_fd_ = -1;
    return false;
  }
  return true;
}

void SessionManifest::release_lock()
{
  if (lock_fd_ >= 0) {
    ::flock(lock_fd_, LOCK_UN);
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

bool SessionManifest::write_manifest_atomic(const std::string & body)
{
  return write_file_atomic(run_dir_ + "/manifest.json", body);
}

bool SessionManifest::write_file_atomic(
  const std::string & path, const std::string & body)
{
  const std::string tmp = path + ".tmp";
  const int fd = ::open(
    tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    return false;
  }
  const char * p = body.data();
  size_t left = body.size();
  while (left > 0) {
    const ssize_t n = ::write(fd, p, left);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      ::close(fd);
      return false;
    }
    p += n;
    left -= static_cast<size_t>(n);
  }
  ::fdatasync(fd);
  ::close(fd);
  return ::rename(tmp.c_str(), path.c_str()) == 0;
}

}  // namespace runtime_logger
