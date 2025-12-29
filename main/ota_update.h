#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    RS3_OTA_STATE_IDLE = 0,
    RS3_OTA_STATE_RUNNING,
    RS3_OTA_STATE_SUCCESS,
    RS3_OTA_STATE_FAILED,
} rs3_ota_state_t;

typedef struct {
    rs3_ota_state_t state;
    esp_err_t last_err;
    // Progress info (best-effort). If total_bytes < 0, server didn't provide Content-Length.
    int32_t bytes_read;
    int32_t total_bytes;
    // -1 if unknown
    int8_t progress_pct;
} rs3_ota_status_t;

typedef void (*rs3_ota_status_cb_t)(const rs3_ota_status_t *st, void *user_ctx);

void rs3_ota_set_status_cb(rs3_ota_status_cb_t cb, void *user_ctx);

/**
 * @brief Start OTA in background task. If url is NULL/empty, uses CONFIG_RS3_OTA_URL.
 */
esp_err_t rs3_ota_start(const char *url);


