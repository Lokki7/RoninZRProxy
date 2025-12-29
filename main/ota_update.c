#include "ota_update.h"

#include <string.h>

#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota_update";

static TaskHandle_t s_task = NULL;
static char s_url[256] = {0};

static rs3_ota_status_cb_t s_cb = NULL;
static void *s_cb_ctx = NULL;
static rs3_ota_status_t s_status = {
    .state = RS3_OTA_STATE_IDLE,
    .last_err = ESP_OK,
    .bytes_read = 0,
    .total_bytes = -1,
    .progress_pct = -1,
};

void rs3_ota_set_status_cb(rs3_ota_status_cb_t cb, void *user_ctx)
{
    s_cb = cb;
    s_cb_ctx = user_ctx;
    if (s_cb) s_cb(&s_status, s_cb_ctx);
}

static inline void emit(void)
{
    if (s_cb) s_cb(&s_status, s_cb_ctx);
}

static void ota_task(void *arg)
{
    (void)arg;

    s_status.state = RS3_OTA_STATE_RUNNING;
    s_status.last_err = ESP_OK;
    s_status.bytes_read = 0;
    s_status.total_bytes = -1;
    s_status.progress_pct = -1;
    emit();

    ESP_LOGI(TAG, "Starting OTA from URL: %s", s_url);

    esp_http_client_config_t http_cfg = {
        .url = s_url,
        // Works for HTTPS if the server cert chains to a known root
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ret = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin failed: %s", esp_err_to_name(ret));
        s_status.state = RS3_OTA_STATE_FAILED;
        s_status.last_err = ret;
        emit();
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    int32_t total = (int32_t)esp_https_ota_get_image_size(ota_handle);
    if (total > 0) {
        s_status.total_bytes = total;
        emit();
    }

    int last_pct = -1;
    TickType_t last_emit = xTaskGetTickCount();
    const TickType_t emit_period = pdMS_TO_TICKS(2000);
    while (1) {
        ret = esp_https_ota_perform(ota_handle);
        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;

        int32_t read = (int32_t)esp_https_ota_get_image_len_read(ota_handle);
        s_status.bytes_read = read;

        // total may become known later (after headers)
        if (s_status.total_bytes <= 0) {
            int32_t t2 = (int32_t)esp_https_ota_get_image_size(ota_handle);
            if (t2 > 0) s_status.total_bytes = t2;
        }

        int pct = -1;
        if (s_status.total_bytes > 0) {
            pct = (int)((100LL * s_status.bytes_read) / s_status.total_bytes);
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
        }
        s_status.progress_pct = (int8_t)pct;

        // Throttle UI/log updates: at most once per 2 seconds.
        TickType_t now = xTaskGetTickCount();
        if ((now - last_emit) > emit_period) {
            last_emit = now;
            if (pct != last_pct) {
                last_pct = pct;
                if (pct >= 0 && s_status.total_bytes > 0) {
                    ESP_LOGI(TAG, "OTA %d%% (%ld/%ld)", pct, (long)s_status.bytes_read, (long)s_status.total_bytes);
                } else if (s_status.bytes_read > 0) {
                    ESP_LOGI(TAG, "OTA read %ld bytes", (long)s_status.bytes_read);
                }
            }
            emit();
        }
    }

    if (ret == ESP_OK) {
        ret = esp_https_ota_finish(ota_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "OTA success, restarting...");
            s_status.progress_pct = 100;
            s_status.state = RS3_OTA_STATE_SUCCESS;
            s_status.last_err = ESP_OK;
            emit();
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(ret));
            s_status.state = RS3_OTA_STATE_FAILED;
            s_status.last_err = ret;
            emit();
        }
    } else {
        ESP_LOGE(TAG, "esp_https_ota_perform failed: %s", esp_err_to_name(ret));
        (void)esp_https_ota_abort(ota_handle);
        s_status.state = RS3_OTA_STATE_FAILED;
        s_status.last_err = ret;
        emit();
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t rs3_ota_start(const char *url)
{
#if !CONFIG_RS3_OTA_ENABLE
    (void)url;
    return ESP_OK;
#else
    if (s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *use = url;
    if (!use || strlen(use) == 0) {
        use = CONFIG_RS3_OTA_URL;
    }
    if (!use || strlen(use) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    strlcpy(s_url, use, sizeof(s_url));
    xTaskCreate(ota_task, "ota", 8192, NULL, 5, &s_task);
    return ESP_OK;
#endif
}


