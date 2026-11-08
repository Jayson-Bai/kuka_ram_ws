#pragma once

namespace runtime_logging
{

/*
 * Shared shutdown budget for the runtime-log pipeline.
 *
 * Producers get this many milliseconds to drain their capture ring and
 * flush the final producer_status frame. The logger waits this budget
 * plus kShutdownGraceMarginMs before finalizing the session, so a clean
 * shutdown has room for the final status to travel over AF_UNIX.
 */
constexpr int kDefaultShutdownDrainMs = 5000;
constexpr int kShutdownGraceMarginMs = 1000;

}  // namespace runtime_logging
