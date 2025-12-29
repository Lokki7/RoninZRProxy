#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Start a dedicated TCP server for PTP proxying (binary framed protocol).
 *
 * Listens on CONFIG_RS3_USB_PTP_PROXY_PORT.
 * Single client at a time; new client replaces old one.
 */
esp_err_t rs3_ptp_proxy_server_start(void);

/**
 * @brief Returns true if a proxy client is connected.
 */
bool rs3_ptp_proxy_is_connected(void);

/**
 * @brief Send one framed message to the proxy client.
 *
 * Frame format:
 *   uint32_be length (type byte + payload bytes)
 *   uint8    type
 *   payload...
 */
esp_err_t rs3_ptp_proxy_send_frame(uint8_t type, const uint8_t *payload, size_t payload_len);

/**
 * @brief Receive one framed message from the proxy client.
 *
 * Reads a single frame (blocking up to timeout_ms). Payload is written into out_buf.
 * Returns:
 * - ESP_OK on success
 * - ESP_ERR_TIMEOUT on timeout
 * - ESP_ERR_INVALID_STATE if no client
 * - ESP_ERR_INVALID_SIZE if frame doesn't fit out_buf
 */
esp_err_t rs3_ptp_proxy_recv_frame(uint8_t *out_type,
                                  uint8_t *out_buf,
                                  size_t out_cap,
                                  size_t *out_len,
                                  uint32_t timeout_ms);


