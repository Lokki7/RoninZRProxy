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
static rs3_ota_status_t s_status = { .state = RS3_OTA_STATE_IDLE, .last_err = ESP_OK };

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

    esp_err_t ret = esp_https_ota(&ota_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA success, restarting...");
        s_status.state = RS3_OTA_STATE_SUCCESS;
        s_status.last_err = ESP_OK;
        emit();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
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


