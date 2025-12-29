#include "cmd_tcp.h"

#include <ctype.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ota_update.h"
#include "tcp_server.h"

static const char *TAG = "cmd_tcp";

static char s_line[256];
static size_t s_line_len = 0;

static void handle_line(char *line)
{
    // trim leading spaces
    while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
    if (*line == 0) return;

    // lowercase command token (in-place)
    char *cmd = line;
    char *arg = line;
    while (*arg && *arg != ' ' && *arg != '\t') {
        *arg = (char)tolower((unsigned char)*arg);
        arg++;
    }
    if (*arg) {
        *arg++ = 0;
        while (*arg == ' ' || *arg == '\t') arg++;
    }

    if (strcmp(cmd, "ota") == 0) {
        const char *url = (*arg) ? arg : NULL;
        esp_err_t ret = rs3_ota_start(url);
        if (ret == ESP_OK) {
            rs3_tcp_server_send_str("OTA: started\r\n");
        } else {
            rs3_tcp_server_send_str("OTA: failed to start\r\n");
        }
        return;
    }

    if (strcmp(cmd, "reboot") == 0 || strcmp(cmd, "restart") == 0 || strcmp(cmd, "reset") == 0) {
        rs3_tcp_server_send_str("OK: rebooting\r\n");
        vTaskDelay(pdMS_TO_TICKS(150));
        esp_restart();
        return;
    }

    rs3_tcp_server_send_str("ERR: unknown cmd\r\n");
}

static void rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
    (void)user_ctx;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            s_line[s_line_len] = 0;
            handle_line(s_line);
            s_line_len = 0;
            continue;
        }
        if (c == '\r') continue;
        if (s_line_len + 1 < sizeof(s_line)) {
            s_line[s_line_len++] = c;
        }
    }
}

esp_err_t rs3_cmd_tcp_start(void)
{
    rs3_tcp_server_set_rx_cb(rx_cb, NULL);
    ESP_LOGI(TAG, "TCP command handler ready (send: ota <url>, reboot)");
    return ESP_OK;
}


