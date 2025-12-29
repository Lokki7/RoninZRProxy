#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    bool client_connected;
} rs3_tcp_server_status_t;

typedef void (*rs3_tcp_server_status_cb_t)(const rs3_tcp_server_status_t *status, void *user_ctx);
typedef void (*rs3_tcp_server_rx_cb_t)(const uint8_t *data, size_t len, void *user_ctx);

/**
 * @brief Start TCP server task (listens on CONFIG_RS3_TCP_SERVER_PORT).
 *
 * Requires:
 * - esp_netif_init() already called
 * - esp_event_loop_create_default() already called
 */
esp_err_t rs3_tcp_server_start(void);

/**
 * @brief Enqueue a text line to send to the currently connected client (if any).
 * Non-blocking: drops if queue is full or no client.
 */
esp_err_t rs3_tcp_server_send(const char *data, size_t len);

/**
 * @brief Convenience helper for C strings (no newline added).
 */
static inline esp_err_t rs3_tcp_server_send_str(const char *s)
{
    size_t n = 0;
    while (s && s[n]) n++;
    return rs3_tcp_server_send(s, n);
}

void rs3_tcp_server_set_status_cb(rs3_tcp_server_status_cb_t cb, void *user_ctx);
void rs3_tcp_server_set_rx_cb(rs3_tcp_server_rx_cb_t cb, void *user_ctx);


