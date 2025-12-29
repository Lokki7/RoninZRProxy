#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    RS3_REC_EVT_START = 1,
    RS3_REC_EVT_STOP = 2,
} rs3_rec_evt_kind_t;

typedef struct {
    rs3_rec_evt_kind_t kind;
    bool recording;           // derived state after applying event
    uint32_t tid;             // PTP transaction id (from RS3)
    uint64_t ts_us;           // esp_timer_get_time()
    uint8_t payload[5];       // raw 0x9207 DATA payload (up to 5 bytes, observed)
    size_t payload_len;
} rs3_rec_event_t;

typedef void (*rs3_rec_event_cb_t)(const rs3_rec_event_t *ev, void *user_ctx);

/**
 * @brief Start the recorder event dispatcher task.
 */
esp_err_t rs3_rec_events_start(void);

/**
 * @brief Subscribe to record events (callbacks are invoked from the dispatcher task context).
 */
esp_err_t rs3_rec_events_subscribe(rs3_rec_event_cb_t cb, void *user_ctx);

/**
 * @brief Publish record start/stop event (non-blocking best-effort).
 */
void rs3_rec_events_publish(rs3_rec_evt_kind_t kind,
                            uint32_t tid,
                            const uint8_t *payload,
                            size_t payload_len);


