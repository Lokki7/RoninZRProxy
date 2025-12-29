#include "tcp_server.h"

#include <errno.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

static const char *TAG = "tcp_server";

typedef struct {
    size_t len;
    char buf[512];
} out_msg_t;

static QueueHandle_t s_out_q = NULL;
static TaskHandle_t s_task = NULL;
static int s_client_fd = -1;

static rs3_tcp_server_status_cb_t s_status_cb = NULL;
static void *s_status_ctx = NULL;
static rs3_tcp_server_status_t s_status = { .client_connected = false };
static rs3_tcp_server_rx_cb_t s_rx_cb = NULL;
static void *s_rx_ctx = NULL;

void rs3_tcp_server_set_status_cb(rs3_tcp_server_status_cb_t cb, void *user_ctx)
{
    s_status_cb = cb;
    s_status_ctx = user_ctx;
    if (s_status_cb) s_status_cb(&s_status, s_status_ctx);
}

void rs3_tcp_server_set_rx_cb(rs3_tcp_server_rx_cb_t cb, void *user_ctx)
{
    s_rx_cb = cb;
    s_rx_ctx = user_ctx;
}

static inline void emit_status(void)
{
    if (s_status_cb) s_status_cb(&s_status, s_status_ctx);
}

esp_err_t rs3_tcp_server_send(const char *data, size_t len)
{
#if !CONFIG_RS3_TCP_SERVER_ENABLE
    (void)data; (void)len;
    return ESP_OK;
#else
    if (!s_out_q || !data || len == 0) return ESP_ERR_INVALID_STATE;
    if (len > sizeof(((out_msg_t *)0)->buf)) len = sizeof(((out_msg_t *)0)->buf);
    out_msg_t msg = { .len = len };
    memcpy(msg.buf, data, len);
    // non-blocking: drop if full
    if (xQueueSend(s_out_q, &msg, 0) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
#endif
}

static void close_client(void)
{
    if (s_client_fd >= 0) {
        shutdown(s_client_fd, SHUT_RDWR);
        close(s_client_fd);
        s_client_fd = -1;
    }
    if (s_status.client_connected) {
        s_status.client_connected = false;
        emit_status();
    }
}

static void server_task(void *arg)
{
    (void)arg;

    const int port = CONFIG_RS3_TCP_SERVER_PORT;

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind(%d) failed: errno=%d", port, errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening on TCP port %d", port);

    for (;;) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        int maxfd = listen_fd;
        if (s_client_fd >= 0) {
            FD_SET(s_client_fd, &rfds);
            if (s_client_fd > maxfd) maxfd = s_client_fd;
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 100 * 1000 };
        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            ESP_LOGW(TAG, "select() errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (FD_ISSET(listen_fd, &rfds)) {
            struct sockaddr_in6 source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int fd = accept(listen_fd, (struct sockaddr *)&source_addr, &addr_len);
            if (fd >= 0) {
                close_client();
                s_client_fd = fd;
                s_status.client_connected = true;
                emit_status();

                const char *banner = "rs3proxy: connected\r\n";
                (void)send(s_client_fd, banner, strlen(banner), 0);
                ESP_LOGI(TAG, "Client connected");
            }
        }

        if (s_client_fd >= 0 && FD_ISSET(s_client_fd, &rfds)) {
            char rx[128];
            int n = recv(s_client_fd, rx, sizeof(rx) - 1, 0);
            if (n <= 0) {
                ESP_LOGI(TAG, "Client disconnected");
                close_client();
            } else {
                if (s_rx_cb) {
                    s_rx_cb((const uint8_t *)rx, (size_t)n, s_rx_ctx);
                }
            }
        }

        // Drain outgoing queue (best-effort)
        if (s_client_fd >= 0) {
            out_msg_t msg;
            while (xQueueReceive(s_out_q, &msg, 0) == pdTRUE) {
                int sent = send(s_client_fd, msg.buf, msg.len, 0);
                if (sent < 0) {
                    ESP_LOGW(TAG, "send() failed: errno=%d", errno);
                    close_client();
                    break;
                }
            }
        } else {
            // no client: drop queued messages
            out_msg_t msg;
            while (xQueueReceive(s_out_q, &msg, 0) == pdTRUE) {}
        }
    }
}

esp_err_t rs3_tcp_server_start(void)
{
#if !CONFIG_RS3_TCP_SERVER_ENABLE
    ESP_LOGI(TAG, "TCP server disabled");
    return ESP_OK;
#else
    if (s_task) return ESP_OK;

    s_out_q = xQueueCreate(8, sizeof(out_msg_t));
    if (!s_out_q) return ESP_ERR_NO_MEM;

    xTaskCreate(server_task, "tcp_server", 4096, NULL, 4, &s_task);
    return ESP_OK;
#endif
}


