#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "font5x7.h"
#include "lcd_st7789.h"
#include "pmu_axp2101.h"
#include "wifi_sta.h"
#include "ui_status.h"
#include "tcp_server.h"
#include "cmd_tcp.h"
#include "ota_update.h"
#include "usb_ptp_cam.h"
#include "ptp_proxy_server.h"
#include "rec_events.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_event.h"

static const char *TAG = "rs3proxy";

static void rec_ui_cb(const rs3_rec_event_t *ev, void *ctx)
{
    (void)ctx;
    (void)rs3_ui_status_set_rec(ev->recording);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello from ESP-IDF!");
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);

    // ---- System services ----
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG,
             "Chip: model=%d, cores=%d, revision=%d, features=0x%" PRIx32,
             chip_info.model,
             chip_info.cores,
             chip_info.revision,
             chip_info.features);

    uint32_t flash_size_mb = 0;
    if (esp_flash_get_size(NULL, &flash_size_mb) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %" PRIu32 "MB", flash_size_mb / (1024 * 1024));
    } else {
        ESP_LOGW(TAG, "Flash: size unknown");
    }

    #if CONFIG_SPIRAM
    size_t psram_size = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM: %zu bytes", psram_size);
    #else
    ESP_LOGI(TAG, "PSRAM: disabled (CONFIG_SPIRAM is not set)");
    #endif

    // ---- PMU power (AXP2101) ----
    ret = rs3_pmu_init_and_enable_lcd_power();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PMU: init/power enable failed (%s). LCD may stay off.", esp_err_to_name(ret));
    }

    // ---- LCD init (ST7789) ----
    ESP_ERROR_CHECK(rs3_lcd_init());

    // ---- UI status (LCD) ----
    ESP_ERROR_CHECK(rs3_ui_status_start());
    rs3_wifi_sta_set_status_cb(rs3_ui_status_wifi_cb, NULL);

    // ---- Recording events (RS3 start/stop record) ----
    ESP_ERROR_CHECK(rs3_rec_events_start());
    // UI subscriber: show REC: ON/OFF. (Bluetooth can subscribe later as well.)
    ESP_ERROR_CHECK(rs3_rec_events_subscribe(rec_ui_cb, NULL));

    // ---- USB PTP camera emulation ----
    ESP_ERROR_CHECK(rs3_usb_ptp_cam_start());
    {
        // Show current USB PTP implementation mode on the LCD.
        char impl[32] = {0};
#if !CONFIG_RS3_USB_PTP_ENABLE
        snprintf(impl, sizeof(impl), "off");
#else
    #if CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW
        snprintf(impl, sizeof(impl), "proxy_raw:%d", CONFIG_RS3_USB_PTP_PROXY_PORT);
    #elif CONFIG_RS3_USB_PTP_IMPL_LEGACY
        snprintf(impl, sizeof(impl), "legacy");
    #elif CONFIG_RS3_USB_PTP_IMPL_STD
        snprintf(impl, sizeof(impl), "std");
    #else
        snprintf(impl, sizeof(impl), "?");
    #endif
#endif
        (void)rs3_ui_status_ptp_impl(impl);
    }

    // ---- TCP server ----
    rs3_tcp_server_set_status_cb(rs3_ui_status_tcp_cb, NULL);
    ESP_ERROR_CHECK(rs3_tcp_server_start());
    ESP_ERROR_CHECK(rs3_cmd_tcp_start());

    // ---- PTP proxy TCP (separate port) ----
    ESP_ERROR_CHECK(rs3_ptp_proxy_server_start());

    // ---- Wi-Fi (STA) ----
    ESP_ERROR_CHECK(rs3_wifi_sta_start());

    // ---- OTA status ----
    rs3_ota_set_status_cb(rs3_ui_status_ota_cb, NULL);

    for (;;) {
        ESP_LOGI(TAG, "tick");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


