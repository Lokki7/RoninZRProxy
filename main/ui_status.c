#include "ui_status.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "font5x7.h"
#include "lcd_st7789.h"
#include "log_tcp.h"
#include "ota_update.h"
#include "touch_cst816.h"
#include "nikon_bt.h"

#include "esp_system.h"

static const char *TAG = "ui_status";

enum { PTP_UI_LINE_MAX = 32 };

typedef enum {
    UI_MSG_WIFI,
    UI_MSG_TCP,
    UI_MSG_OTA,
    UI_MSG_PTP_IMPL,
    UI_MSG_PTP_LINE,
    UI_MSG_REC,
    UI_MSG_BT_LINE,
} ui_msg_kind_t;

typedef struct {
    ui_msg_kind_t kind;
    union {
        rs3_wifi_sta_status_t wifi;
        rs3_tcp_server_status_t tcp;
        rs3_ota_status_t ota;
        char ptp_impl[PTP_UI_LINE_MAX];
        char ptp_line[PTP_UI_LINE_MAX];
        char bt_line[PTP_UI_LINE_MAX];
        bool rec_on;
    };
} ui_msg_t;

static QueueHandle_t s_q = NULL;
static uint16_t *s_fb = NULL;
static rs3_lcd_info_t s_lcd;
static rs3_wifi_sta_status_t s_last_wifi;
static rs3_tcp_server_status_t s_last_tcp;
static bool s_has_tcp = false;
static rs3_ota_status_t s_last_ota = {
    .state = RS3_OTA_STATE_IDLE,
    .last_err = ESP_OK,
    .bytes_read = 0,
    .total_bytes = -1,
    .progress_pct = -1,
};
static bool s_has_ota = true;
static char s_ptp_impl[PTP_UI_LINE_MAX] = {0};
static char s_ptp_status[PTP_UI_LINE_MAX] = {0};
static char s_bt_status[PTP_UI_LINE_MAX] = {0};
static bool s_rec_on = false;
static bool s_has_rec = false;

// Simple touch buttons (bottom area)
typedef struct {
    int x, y, w, h;
    const char *label;
} ui_btn_t;

static ui_btn_t s_btn_ota = { .x = 0, .y = 0, .w = 0, .h = 36, .label = "Update FW" };
static ui_btn_t s_btn_rst = { .x = 0, .y = 0, .w = 0, .h = 36, .label = "Restart MCU" };
static ui_btn_t s_btn_pair = { .x = 0, .y = 0, .w = 0, .h = 36, .label = "Pair Nikon" };
static ui_btn_t s_btn_shut = { .x = 0, .y = 0, .w = 0, .h = 36, .label = "Shutter" };

static bool s_touch_prev = false;

enum {
    UI_BTN_MARGIN_X = 10,
    UI_BTN_GAP_Y = 12,
    UI_BTN_HIT_PAD = 10, // expand clickable area around the button
    UI_BTN_GAP_X = 12,
};

static void fb_fill(uint16_t *fb, int w, int h, uint16_t color)
{
    for (int i = 0; i < w * h; i++) fb[i] = color;
}

static void fb_fill_rect(uint16_t *fb, int fb_w, int fb_h, int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > fb_w) w = fb_w - x;
    if (y + h > fb_h) h = fb_h - y;
    if (w <= 0 || h <= 0) return;
    for (int yy = 0; yy < h; yy++) {
        uint16_t *row = &fb[(y + yy) * fb_w + x];
        for (int xx = 0; xx < w; xx++) row[xx] = color;
    }
}

static void fb_rect_border(uint16_t *fb, int fb_w, int fb_h, int x, int y, int w, int h, uint16_t color)
{
    // top/bottom
    fb_fill_rect(fb, fb_w, fb_h, x, y, w, 1, color);
    fb_fill_rect(fb, fb_w, fb_h, x, y + h - 1, w, 1, color);
    // left/right
    fb_fill_rect(fb, fb_w, fb_h, x, y, 1, h, color);
    fb_fill_rect(fb, fb_w, fb_h, x + w - 1, y, 1, h, color);
}

static void draw_button(const ui_btn_t *b)
{
    const uint16_t BLACK = 0x0000;
    const uint16_t WHITE = 0xFFFF;
    const uint16_t GRAY = 0x8410; // ~50% gray
    const int scale = 2;
    fb_fill_rect(s_fb, s_lcd.w, s_lcd.h, b->x, b->y, b->w, b->h, BLACK);
    fb_rect_border(s_fb, s_lcd.w, s_lcd.h, b->x, b->y, b->w, b->h, GRAY);
    rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, b->x + 8, b->y + 4, b->label, WHITE, BLACK, scale);
}

static bool btn_hit(const ui_btn_t *b, int tx, int ty)
{
    return (tx >= (b->x - UI_BTN_HIT_PAD) &&
            tx < (b->x + b->w + UI_BTN_HIT_PAD) &&
            ty >= (b->y - UI_BTN_HIT_PAD) &&
            ty < (b->y + b->h + UI_BTN_HIT_PAD));
}

static void render_all(void)
{
    const uint16_t BLACK = 0x0000;
    const uint16_t WHITE = 0xFFFF;
    const int scale = 2;

    fb_fill(s_fb, s_lcd.w, s_lcd.h, BLACK);

    char line1[64] = {0};
    char line2[64] = {0};
    char line3[64] = {0};
    char line4[64] = {0};
    char line_bt[64] = {0};

    switch (s_last_wifi.state) {
        case RS3_WIFI_STA_STATE_DISABLED:
            snprintf(line1, sizeof(line1), "WiFi: off");
            break;
        case RS3_WIFI_STA_STATE_CONNECTING:
            snprintf(line1, sizeof(line1), "WiFi: conn");
            break;
        case RS3_WIFI_STA_STATE_CONNECTED:
            snprintf(line1, sizeof(line1), "WiFi: ok");
            if (s_last_wifi.has_ip) {
                snprintf(line2, sizeof(line2), "IP: " IPSTR, IP2STR(&s_last_wifi.ip));
            }
            break;
        case RS3_WIFI_STA_STATE_FAILED:
            snprintf(line1, sizeof(line1), "WiFi: fail");
            break;
        default:
            snprintf(line1, sizeof(line1), "WiFi: ?");
            break;
    }

    if (CONFIG_RS3_TCP_SERVER_ENABLE) {
        snprintf(line3, sizeof(line3), "TCP:%d %s", CONFIG_RS3_TCP_SERVER_PORT,
                 (s_has_tcp && s_last_tcp.client_connected) ? "cli" : "wait");
    }

    if (CONFIG_RS3_OTA_ENABLE && s_has_ota) {
        const char *ota_s = "?";
        switch (s_last_ota.state) {
            case RS3_OTA_STATE_IDLE: ota_s = "idle"; break;
            case RS3_OTA_STATE_RUNNING: ota_s = "run"; break;
            case RS3_OTA_STATE_SUCCESS: ota_s = "ok"; break;
            case RS3_OTA_STATE_FAILED: ota_s = "fail"; break;
            default: break;
        }
        if (s_last_ota.state == RS3_OTA_STATE_RUNNING) {
            if (s_last_ota.progress_pct >= 0) {
                snprintf(line4, sizeof(line4), "OTA: %s %d%%", ota_s, (int)s_last_ota.progress_pct);
            } else if (s_last_ota.bytes_read > 0) {
                snprintf(line4, sizeof(line4), "OTA: %s %ld", ota_s, (long)s_last_ota.bytes_read);
            } else {
                snprintf(line4, sizeof(line4), "OTA: %s", ota_s);
            }
        } else {
            snprintf(line4, sizeof(line4), "OTA: %s", ota_s);
        }
    }

    if (s_bt_status[0]) {
        snprintf(line_bt, sizeof(line_bt), "%s", s_bt_status);
    }

    const int x = 10;
    const int y1 = 10;
    const int y2 = y1 + (7 + 2) * scale;
    const int y3 = y2 + (7 + 2) * scale;
    const int y4 = y3 + (7 + 2) * scale;
    const int y5 = y4 + (7 + 2) * scale;
    const int y6 = y5 + (7 + 4) * scale;
    const int y7 = y6 + (7 + 2) * scale;
    const int y8 = y7 + (7 + 2) * scale;
    rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y1, line1, WHITE, BLACK, scale);
    if (strlen(line2)) {
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y2, line2, WHITE, BLACK, scale);
    }
    if (strlen(line3)) {
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y3, line3, WHITE, BLACK, scale);
    }
    if (strlen(line4)) {
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y4, line4, WHITE, BLACK, scale);
    }
    if (strlen(line_bt)) {
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y5, line_bt, WHITE, BLACK, scale);
    }

    // USB/PTP impl + current status (no history)
    if (s_ptp_impl[0]) {
        char ptp_impl_line[48];
        snprintf(ptp_impl_line, sizeof(ptp_impl_line), "PTP impl: %s", s_ptp_impl);
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y6, ptp_impl_line, WHITE, BLACK, scale);
    }
    if (s_has_rec) {
        char rec_line[48];
        snprintf(rec_line, sizeof(rec_line), "REC: %s", s_rec_on ? "ON" : "OFF");
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, y7, rec_line, WHITE, BLACK, scale);
    }
    if (s_ptp_status[0]) {
        char ptp_line[48];
        snprintf(ptp_line, sizeof(ptp_line), "PTP: %s", s_ptp_status);
        rs3_draw_text_5x7(s_fb, s_lcd.w, s_lcd.h, x, s_has_rec ? y8 : y7, ptp_line, WHITE, BLACK, scale);
    }

    // Touch buttons
    draw_button(&s_btn_pair);
    draw_button(&s_btn_shut);
    draw_button(&s_btn_ota);
    draw_button(&s_btn_rst);

    ESP_ERROR_CHECK(rs3_lcd_draw_full(s_fb));
}

static void ui_task(void *arg)
{
    ui_msg_t msg = {0};
    // Default screen
    s_last_wifi = (rs3_wifi_sta_status_t){
        .state = RS3_WIFI_STA_STATE_DISABLED,
        .retry_count = 0,
        .has_ip = false,
    };
    render_all();

    while (1) {
        // Poll touch regularly, but still react to status updates promptly.
        if (xQueueReceive(s_q, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
            switch (msg.kind) {
                case UI_MSG_WIFI:
                    s_last_wifi = msg.wifi;
                    break;
                case UI_MSG_TCP:
                    s_last_tcp = msg.tcp;
                    s_has_tcp = true;
                    break;
                case UI_MSG_OTA:
                    s_last_ota = msg.ota;
                    break;
                case UI_MSG_PTP_IMPL:
                    snprintf(s_ptp_impl, sizeof(s_ptp_impl), "%s", msg.ptp_impl);
                    break;
                case UI_MSG_PTP_LINE:
                    snprintf(s_ptp_status, sizeof(s_ptp_status), "%s", msg.ptp_line);
                    break;
                case UI_MSG_REC:
                    s_rec_on = msg.rec_on;
                    s_has_rec = true;
                    break;
                case UI_MSG_BT_LINE:
                    snprintf(s_bt_status, sizeof(s_bt_status), "%s", msg.bt_line);
                    break;
                default:
                    break;
            }
            render_all();
        }

        int tx = 0, ty = 0;
        bool t = rs3_touch_get_point(&tx, &ty);
        if (t && !s_touch_prev) {
            // Rising edge = click
            if (btn_hit(&s_btn_pair, tx, ty)) {
                rs3_tcp_logf("[UI] Pair Nikon pressed x=%d y=%d\r\n", tx, ty);
                (void)rs3_ui_status_bt_line("BT: pair pressed");
                (void)rs3_nikon_bt_pair_start();
            } else if (btn_hit(&s_btn_shut, tx, ty)) {
                rs3_tcp_logf("[UI] Shutter pressed x=%d y=%d\r\n", tx, ty);
                (void)rs3_ui_status_bt_line("BT: shutter pressed");
                (void)rs3_nikon_bt_shutter_click();
            } else if (btn_hit(&s_btn_ota, tx, ty)) {
                // Fixed URL as requested
                (void)rs3_ota_start("http://192.168.1.246:8000/rs3proxy_hello.bin");
            } else if (btn_hit(&s_btn_rst, tx, ty)) {
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        }
        s_touch_prev = t;
    }
}

esp_err_t rs3_ui_status_start(void)
{
    if (s_q) return ESP_OK;

    ESP_RETURN_ON_ERROR(rs3_lcd_init(), TAG, "lcd init failed");
    s_lcd = rs3_lcd_get_info();

    // Layout buttons at the bottom (after we know display height).
    // Two rows:
    //   row 1: [ Pair Nikon ] [ Shutter ]
    //   row 2: [ Update FW ] [ Restart MCU ]
    const int row2_y = s_lcd.h - s_btn_ota.h - UI_BTN_GAP_Y;
    const int row1_y = row2_y - s_btn_pair.h - UI_BTN_GAP_Y;
    const int avail_w = s_lcd.w - 2 * UI_BTN_MARGIN_X;
    const int btn_w = (avail_w - UI_BTN_GAP_X) / 2;

    s_btn_pair.x = UI_BTN_MARGIN_X;
    s_btn_pair.y = row1_y;
    s_btn_pair.w = btn_w;

    s_btn_shut.x = UI_BTN_MARGIN_X + btn_w + UI_BTN_GAP_X;
    s_btn_shut.y = row1_y;
    s_btn_shut.w = btn_w;

    s_btn_ota.x = UI_BTN_MARGIN_X;
    s_btn_ota.y = row2_y;
    s_btn_ota.w = btn_w;

    s_btn_rst.x = UI_BTN_MARGIN_X + btn_w + UI_BTN_GAP_X;
    s_btn_rst.y = row2_y;
    s_btn_rst.w = btn_w;

    s_fb = heap_caps_malloc(s_lcd.w * s_lcd.h * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_fb) return ESP_ERR_NO_MEM;

    // Touch init (best-effort; UI still works without touch)
    esp_err_t tr = rs3_touch_init();
    if (tr != ESP_OK) {
        ESP_LOGW(TAG, "Touch init failed (%s)", esp_err_to_name(tr));
    }

    s_q = xQueueCreate(4, sizeof(ui_msg_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    xTaskCreate(ui_task, "ui_status", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "UI status started");
    return ESP_OK;
}

esp_err_t rs3_ui_status_set_wifi(const rs3_wifi_sta_status_t *status)
{
    if (!s_q || !status) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_WIFI, .wifi = *status };
    // non-blocking: drop if queue is full
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}

void rs3_ui_status_wifi_cb(const rs3_wifi_sta_status_t *status, void *user_ctx)
{
    (void)user_ctx;
    (void)rs3_ui_status_set_wifi(status);
}

static esp_err_t rs3_ui_status_set_tcp(const rs3_tcp_server_status_t *status)
{
    if (!s_q || !status) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_TCP, .tcp = *status };
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}

void rs3_ui_status_tcp_cb(const rs3_tcp_server_status_t *status, void *user_ctx)
{
    (void)user_ctx;
    (void)rs3_ui_status_set_tcp(status);
}

static esp_err_t rs3_ui_status_set_ota(const rs3_ota_status_t *st)
{
    if (!s_q || !st) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_OTA, .ota = *st };
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}

void rs3_ui_status_ota_cb(const rs3_ota_status_t *st, void *user_ctx)
{
    (void)user_ctx;
    (void)rs3_ui_status_set_ota(st);
}

esp_err_t rs3_ui_status_ptp_line(const char *line)
{
    if (!s_q || !line) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_PTP_LINE };
    snprintf(msg.ptp_line, sizeof(msg.ptp_line), "%s", line);
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}

esp_err_t rs3_ui_status_set_rec(bool rec_on)
{
    if (!s_q) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_REC, .rec_on = rec_on };
    return (xQueueSend(s_q, &msg, 0) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t rs3_ui_status_ptp_impl(const char *impl)
{
    if (!s_q || !impl) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_PTP_IMPL };
    snprintf(msg.ptp_impl, sizeof(msg.ptp_impl), "%s", impl);
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}

esp_err_t rs3_ui_status_bt_line(const char *line)
{
    if (!s_q || !line) return ESP_ERR_INVALID_STATE;
    ui_msg_t msg = { .kind = UI_MSG_BT_LINE };
    snprintf(msg.bt_line, sizeof(msg.bt_line), "%s", line);
    (void)xQueueSend(s_q, &msg, 0);
    return ESP_OK;
}


