#include "runtime_logging_core/capture_record.hpp"

#include <time.h>

namespace runtime_logging
{

int64_t clock_monotonic_ns()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
         static_cast<int64_t>(ts.tv_nsec);
}

static bool starts_with(const std::string & s, const char * prefix)
{
  return s.rfind(prefix, 0) == 0;
}

CaptureType classify_line(const std::string & line, Direction dir)
{
  if (dir == Direction::kTx) {
    return CaptureType::kTx;
  }
  if (starts_with(line, "ESTAT_H ")) {return CaptureType::kEstatHot;}
  if (starts_with(line, "ESTAT ")) {return CaptureType::kEstat;}
  if (starts_with(line, "USTAT ")) {return CaptureType::kUstat;}
  if (starts_with(line, "STAT ")) {return CaptureType::kStat;}
  if (starts_with(line, "EACK ")) {return CaptureType::kEack;}
  if (starts_with(line, "EWARN ")) {return CaptureType::kEwarn;}
  if (starts_with(line, "EVACK ") || starts_with(line, "EVDONE ") ||
    starts_with(line, "EVERR "))
  {
    return CaptureType::kEvent;
  }
  return CaptureType::kUnknown;
}

}  // namespace runtime_logging
