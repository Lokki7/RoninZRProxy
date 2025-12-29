#include "wifi_sta.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "wifi_sta";

static EventGroupHandle_t s_wifi_event_group;
static esp_event_handler_instance_t s_wifi_any_id;
static esp_event_handler_instance_t s_got_ip;

static int s_retry_num = 0;
static rs3_wifi_sta_status_cb_t s_status_cb = NULL;
static void *s_status_cb_ctx = NULL;
static rs3_wifi_sta_status_t s_status = {
    .state = RS3_WIFI_STA_STATE_DISABLED,
    .retry_count = 0,
    .has_ip = false,
};

// Event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void rs3_wifi_sta_set_status_cb(rs3_wifi_sta_status_cb_t cb, void *user_ctx)
{
    s_status_cb = cb;
    s_status_cb_ctx = user_ctx;
    if (s_status_cb) {
        s_status_cb(&s_status, s_status_cb_ctx);
    }
}

static inline void emit_status(void)
{
    if (s_status_cb) {
        s_status_cb(&s_status, s_status_cb_ctx);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_status.state = RS3_WIFI_STA_STATE_CONNECTING;
        s_status.retry_count = s_retry_num;
        s_status.has_ip = false;
        emit_status();
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (CONFIG_RS3_WIFI_MAXIMUM_RETRY == 0 || s_retry_num < CONFIG_RS3_WIFI_MAXIMUM_RETRY) {
            s_retry_num++;
            ESP_LOGW(TAG, "Disconnected, retrying (%d/%d)...", s_retry_num, CONFIG_RS3_WIFI_MAXIMUM_RETRY);
            s_status.state = RS3_WIFI_STA_STATE_CONNECTING;
            s_status.retry_count = s_retry_num;
            s_status.has_ip = false;
            emit_status();
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d retries", s_retry_num);
            s_status.state = RS3_WIFI_STA_STATE_FAILED;
            s_status.retry_count = s_retry_num;
            s_status.has_ip = false;
            emit_status();
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_retry_num = 0;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_status.state = RS3_WIFI_STA_STATE_CONNECTED;
        s_status.retry_count = 0;
        s_status.has_ip = true;
        s_status.ip = event->ip_info.ip;
        emit_status();
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        return;
    }
}

esp_err_t rs3_wifi_sta_start(void)
{
#if !CONFIG_RS3_WIFI_ENABLE
    ESP_LOGI(TAG, "Wi-Fi disabled (CONFIG_RS3_WIFI_ENABLE=0)");
    s_status.state = RS3_WIFI_STA_STATE_DISABLED;
    s_status.retry_count = 0;
    s_status.has_ip = false;
    emit_status();
    return ESP_OK;
#else
    if (strlen(CONFIG_RS3_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "Wi-Fi enabled, but SSID is empty; skip connect. Set it via menuconfig.");
        s_status.state = RS3_WIFI_STA_STATE_DISABLED;
        s_status.retry_count = 0;
        s_status.has_ip = false;
        emit_status();
        return ESP_OK;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (!netif) {
        ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta failed");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &s_wifi_any_id),
                        TAG, "register WIFI_EVENT failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &s_got_ip),
                        TAG, "register IP_EVENT failed");

    wifi_config_t wifi_config = { 0 };
    strlcpy((char *)wifi_config.sta.ssid, CONFIG_RS3_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_RS3_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.authmode = (strlen(CONFIG_RS3_WIFI_PASSWORD) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    ESP_LOGI(TAG, "Connecting to SSID='%s' ...", CONFIG_RS3_WIFI_SSID);
    s_status.state = RS3_WIFI_STA_STATE_CONNECTING;
    s_status.retry_count = 0;
    s_status.has_ip = false;
    emit_status();
    return ESP_OK;
#endif
}


