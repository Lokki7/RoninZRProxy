#include "rec_events.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

enum { RS3_REC_Q_LEN = 8 };
enum { RS3_REC_SUB_MAX = 4 };

typedef struct {
    rs3_rec_event_cb_t cb;
    void *user_ctx;
} sub_t;

static QueueHandle_t s_q = NULL;
static TaskHandle_t s_task = NULL;
static sub_t s_subs[RS3_REC_SUB_MAX] = {0};

static void rec_task(void *arg)
{
    (void)arg;
    rs3_rec_event_t ev;
    while (1) {
        if (xQueueReceive(s_q, &ev, portMAX_DELAY) != pdTRUE) continue;
        for (int i = 0; i < RS3_REC_SUB_MAX; i++) {
            if (s_subs[i].cb) {
                s_subs[i].cb(&ev, s_subs[i].user_ctx);
            }
        }
    }
}

esp_err_t rs3_rec_events_start(void)
{
    if (s_q) return ESP_OK;
    s_q = xQueueCreate(RS3_REC_Q_LEN, sizeof(rs3_rec_event_t));
    if (!s_q) return ESP_ERR_NO_MEM;
    xTaskCreate(rec_task, "rec_events", 3072, NULL, 4, &s_task);
    return ESP_OK;
}

esp_err_t rs3_rec_events_subscribe(rs3_rec_event_cb_t cb, void *user_ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < RS3_REC_SUB_MAX; i++) {
        if (!s_subs[i].cb) {
            s_subs[i].cb = cb;
            s_subs[i].user_ctx = user_ctx;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

void rs3_rec_events_publish(rs3_rec_evt_kind_t kind,
                            uint32_t tid,
                            const uint8_t *payload,
                            size_t payload_len)
{
    if (!s_q) return;
    rs3_rec_event_t ev = {0};
    ev.kind = kind;
    ev.recording = (kind == RS3_REC_EVT_START);
    ev.tid = tid;
    ev.ts_us = (uint64_t)esp_timer_get_time();
    if (payload && payload_len) {
        ev.payload_len = (payload_len > sizeof(ev.payload)) ? sizeof(ev.payload) : payload_len;
        memcpy(ev.payload, payload, ev.payload_len);
    }
    (void)xQueueSend(s_q, &ev, 0);
}


