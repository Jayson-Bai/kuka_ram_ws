#pragma once

#include <cstdint>
#include <string>

namespace runtime_logger
{

/*
 * Session manifest + integrity summary. The manifest is written at
 * session start and atomically updated at finalize; integrity.json is
 * written once at end-of-session with the accounting result.
 */
class SessionManifest
{
public:
  bool start(
    const std::string & root_dir,
    const std::string & run_id,
    const std::string & code_version,
    const std::string & config_json,
    bool test_run);
  bool finalize(
    bool complete,
    const std::string & integrity_json,
    const std::string & end_reason);
  const std::string & run_dir() const {return run_dir_;}
  const std::string & run_id() const {return run_id_;}
  bool active() const {return active_;}

  // Take/keep a flock on run_dir.lock so two loggers can't share a run.
  bool acquire_lock();
  void release_lock();

private:
  std::string run_dir_;
  std::string run_id_;
  bool active_{false};
  int lock_fd_{-1};
  bool write_manifest_atomic(const std::string & body);
  bool write_file_atomic(const std::string & path, const std::string & body);
};

}  // namespace runtime_logger
