/**
 * bench_main_linux.c  —  A-core / Linux host port
 *
 * ============================================================
 *  This is the only file added for A-core Linux builds.
 *  It plays the same role as bench_host.c in MCU-side Base Firmware:
 *    • provide the main() entry
 *    • implement HostInterface (timing + output)
 *    • call bench_module_entry()
 *
 *  bench_entry.c / het_bench_core.c are shared with MCU builds as-is.
 *
 *  Output format matches the MCU UART protocol, so the same CI parsers work:
 *    BENCHMARK_START
 *    MODULE|<name>|cases=N
 *    RESULT|<case>|<microseconds>
 *    BENCHMARK_END
 * ============================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <chrono>
#include "core/het_bench_core.h"

/* bench_module_entry is defined by the BENCHMARK_IMPLEMENTATION macro in bench_entry.c */
extern int bench_module_entry(const HostInterface *hostApi);

/**
 * Timing: steady_clock (CLOCK_MONOTONIC on Linux), in microseconds (µs).
 * uint32_t holds ~71 minutes; benchmark cases finish in seconds, so no overflow.
 */
static uint32_t linux_get_ticks(void)
{
    using namespace std::chrono;
    const auto us = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<uint32_t>(us);
}

/**
 * Output: write to stdout, captured directly by the deploy scripts (ADB / SSH).
 */
static void linux_write(const char *data, uint32_t len)
{
    fwrite(data, 1, (size_t)len, stdout);
    fflush(stdout);
}

int main(void)
{
    static const HostInterface host = {linux_get_ticks, linux_write};
    /* bench_module_entry returns 1 = all passed, 0 = some case failed */
    return bench_module_entry(&host) ? 0 : 1;
}
