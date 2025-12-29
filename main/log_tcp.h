#pragma once

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Best-effort printf to the current TCP client (non-blocking, drops if no client).
 */
void rs3_tcp_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief va_list variant.
 */
void rs3_tcp_vlogf(const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif


