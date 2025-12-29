#pragma once

#include "esp_err.h"
#include "wifi_sta.h"
#include "tcp_server.h"
#include "ota_update.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start UI status task and begin rendering Wi-Fi status on LCD.
 */
esp_err_t rs3_ui_status_start(void);

/**
 * @brief Push Wi-Fi status update to the UI (non-blocking).
 */
esp_err_t rs3_ui_status_set_wifi(const rs3_wifi_sta_status_t *status);

/**
 * @brief Callback adapter for rs3_wifi_sta_set_status_cb().
 */
void rs3_ui_status_wifi_cb(const rs3_wifi_sta_status_t *status, void *user_ctx);

/**
 * @brief Callback adapter for rs3_tcp_server_set_status_cb().
 */
void rs3_ui_status_tcp_cb(const rs3_tcp_server_status_t *status, void *user_ctx);

/**
 * @brief Callback adapter for rs3_ota_set_status_cb().
 */
void rs3_ui_status_ota_cb(const rs3_ota_status_t *st, void *user_ctx);

/**
 * @brief Set current PTP/USB status line on the LCD (non-blocking).
 *
 * This is intended to show the current RS3 communication progress on the LCD (no history).
 */
esp_err_t rs3_ui_status_ptp_line(const char *line);

/**
 * @brief Set current USB PTP implementation mode string on the LCD (non-blocking).
 *
 * Examples: "std", "legacy", "proxy_raw:1235", "off".
 */
esp_err_t rs3_ui_status_ptp_impl(const char *impl);

/**
 * @brief Update recording indicator on the LCD (non-blocking).
 */
esp_err_t rs3_ui_status_set_rec(bool rec_on);

/**
 * @brief Set Bluetooth status line on the LCD (non-blocking).
 */
esp_err_t rs3_ui_status_bt_line(const char *line);

#ifdef __cplusplus
}
#endif


