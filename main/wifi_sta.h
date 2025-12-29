#pragma once

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

typedef enum {
    RS3_WIFI_STA_STATE_DISABLED = 0,
    RS3_WIFI_STA_STATE_CONNECTING,
    RS3_WIFI_STA_STATE_CONNECTED,
    RS3_WIFI_STA_STATE_FAILED,
} rs3_wifi_sta_state_t;

typedef struct {
    rs3_wifi_sta_state_t state;
    int retry_count;
    bool has_ip;
    esp_ip4_addr_t ip;
} rs3_wifi_sta_status_t;

typedef void (*rs3_wifi_sta_status_cb_t)(const rs3_wifi_sta_status_t *status, void *user_ctx);

/**
 * @brief Register a callback invoked from the Wi-Fi event handler context.
 *
 * Keep it short and non-blocking. If you need work, post to a queue/task.
 */
void rs3_wifi_sta_set_status_cb(rs3_wifi_sta_status_cb_t cb, void *user_ctx);

/**
 * @brief Start Wi-Fi in STA mode and auto-connect using sdkconfig credentials.
 *
 * Requires:
 * - nvs_flash_init() already called
 * - esp_netif_init() already called
 * - esp_event_loop_create_default() already called
 */
esp_err_t rs3_wifi_sta_start(void);


