#include "log_tcp.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "esp_timer.h"
#include "tcp_server.h"

void rs3_tcp_vlogf(const char *fmt, va_list ap)
{
    char msg[256];
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    if (n <= 0) return;
    if (n > (int)sizeof(msg)) n = (int)sizeof(msg);

    // Prefix every line with milliseconds since boot to spot timing gaps/timeouts easily.
    // Example: "[012345.678] ..."
    char out[300];
    const uint64_t us = (uint64_t)esp_timer_get_time();
    const uint32_t ms = (uint32_t)(us / 1000ULL);
    const uint32_t frac = (uint32_t)(us % 1000ULL);
    int h = snprintf(out, sizeof(out), "[%06" PRIu32 ".%03" PRIu32 "] ", ms, frac);
    if (h < 0) return;
    if (h > (int)sizeof(out)) h = (int)sizeof(out);

    size_t to_copy = (size_t)n;
    if ((size_t)h + to_copy > sizeof(out)) {
        to_copy = sizeof(out) - (size_t)h;
    }
    memcpy(out + h, msg, to_copy);
    (void)rs3_tcp_server_send(out, (size_t)h + to_copy);
}

void rs3_tcp_logf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    rs3_tcp_vlogf(fmt, ap);
    va_end(ap);
}


