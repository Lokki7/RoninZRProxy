#include "ptp_proxy_server.h"

#include "sdkconfig.h"

#include <errno.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "log_tcp.h"

static const char *TAG = "ptp_proxy";

static TaskHandle_t s_task = NULL;
static int s_client_fd = -1;

#if CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

static inline void close_client(void)
{
    if (s_client_fd >= 0) {
        shutdown(s_client_fd, SHUT_RDWR);
        close(s_client_fd);
        s_client_fd = -1;
    }
}

bool rs3_ptp_proxy_is_connected(void)
{
    return s_client_fd >= 0;
}

static esp_err_t sock_send_all(int fd, const uint8_t *buf, size_t len)
{
    size_t off = 0;
    while (off < len) {
        int n = send(fd, buf + off, len - off, 0);
        if (n < 0) return ESP_FAIL;
        off += (size_t)n;
    }
    return ESP_OK;
}

static esp_err_t sock_recv_all_timeout(int fd, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t off = 0;
    while (off < len) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = {
            .tv_sec = (int)(timeout_ms / 1000U),
            .tv_usec = (int)((timeout_ms % 1000U) * 1000U),
        };
        int r = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (r == 0) return ESP_ERR_TIMEOUT;
        if (r < 0) return ESP_FAIL;
        if (!FD_ISSET(fd, &rfds)) continue;
        int n = recv(fd, buf + off, len - off, 0);
        if (n <= 0) return ESP_FAIL;
        off += (size_t)n;
    }
    return ESP_OK;
}

esp_err_t rs3_ptp_proxy_send_frame(uint8_t type, const uint8_t *payload, size_t payload_len)
{
#if !(CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW)
    (void)type; (void)payload; (void)payload_len;
    return ESP_ERR_INVALID_STATE;
#else
    if (s_client_fd < 0) return ESP_ERR_INVALID_STATE;
    const uint32_t total = (uint32_t)(payload_len + 1);
    uint8_t hdr[5];
    hdr[0] = (uint8_t)((total >> 24) & 0xFF);
    hdr[1] = (uint8_t)((total >> 16) & 0xFF);
    hdr[2] = (uint8_t)((total >> 8) & 0xFF);
    hdr[3] = (uint8_t)(total & 0xFF);
    hdr[4] = type;

    ESP_RETURN_ON_ERROR(sock_send_all(s_client_fd, hdr, sizeof(hdr)), TAG, "send hdr failed");
    if (payload_len) {
        ESP_RETURN_ON_ERROR(sock_send_all(s_client_fd, payload, payload_len), TAG, "send payload failed");
    }
    return ESP_OK;
#endif
}

esp_err_t rs3_ptp_proxy_recv_frame(uint8_t *out_type,
                                  uint8_t *out_buf,
                                  size_t out_cap,
                                  size_t *out_len,
                                  uint32_t timeout_ms)
{
#if !(CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW)
    (void)out_type; (void)out_buf; (void)out_cap; (void)out_len; (void)timeout_ms;
    return ESP_ERR_INVALID_STATE;
#else
    if (!out_type || !out_buf || !out_len) return ESP_ERR_INVALID_ARG;
    if (s_client_fd < 0) return ESP_ERR_INVALID_STATE;

    uint8_t hdr[5];
    esp_err_t r = sock_recv_all_timeout(s_client_fd, hdr, sizeof(hdr), timeout_ms);
    if (r != ESP_OK) return r;

    uint32_t total = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) | ((uint32_t)hdr[2] << 8) | (uint32_t)hdr[3];
    uint8_t type = hdr[4];
    if (total == 0) return ESP_FAIL;
    size_t payload_len = (size_t)(total - 1);
    if (payload_len > out_cap) return ESP_ERR_INVALID_SIZE;

    if (payload_len) {
        r = sock_recv_all_timeout(s_client_fd, out_buf, payload_len, timeout_ms);
        if (r != ESP_OK) return r;
    }

    *out_type = type;
    *out_len = payload_len;
    return ESP_OK;
#endif
}

static void server_task(void *arg)
{
    (void)arg;

    const int port = CONFIG_RS3_USB_PTP_PROXY_PORT;
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

    ESP_LOGI(TAG, "Listening on proxy TCP port %d", port);

    for (;;) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        int maxfd = listen_fd;
        if (s_client_fd >= 0) {
            FD_SET(s_client_fd, &rfds);
            if (s_client_fd > maxfd) maxfd = s_client_fd;
        }

        // Keep this tight so we accept the Python client promptly before the first RS3 BULK OUT arrives.
        struct timeval tv = { .tv_sec = 0, .tv_usec = 20 * 1000 };
        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (FD_ISSET(listen_fd, &rfds)) {
            struct sockaddr_in6 source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int fd = accept(listen_fd, (struct sockaddr *)&source_addr, &addr_len);
            if (fd >= 0) {
                close_client();
                s_client_fd = fd;
                ESP_LOGI(TAG, "Proxy client connected");
                rs3_tcp_logf("[PTP-PROXY] client connected\r\n");
            }
        }

        if (s_client_fd >= 0 && FD_ISSET(s_client_fd, &rfds)) {
            // If remote closed, drop.
            char tmp[1];
            int n = recv(s_client_fd, tmp, sizeof(tmp), MSG_PEEK);
            if (n <= 0) {
                ESP_LOGI(TAG, "Proxy client disconnected");
                close_client();
                rs3_tcp_logf("[PTP-PROXY] client disconnected\r\n");
            }
        }
    }
}

esp_err_t rs3_ptp_proxy_server_start(void)
{
    if (s_task) return ESP_OK;
    // Slightly higher prio so accept() isn't starved by USB traffic at plug-in time.
    xTaskCreate(server_task, "ptp_proxy", 4096, NULL, 6, &s_task);
    return ESP_OK;
}

#else // CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

bool rs3_ptp_proxy_is_connected(void)
{
    return false;
}

esp_err_t rs3_ptp_proxy_send_frame(uint8_t type, const uint8_t *payload, size_t payload_len)
{
    (void)type; (void)payload; (void)payload_len;
    return ESP_ERR_INVALID_STATE;
}

esp_err_t rs3_ptp_proxy_recv_frame(uint8_t *out_type,
                                  uint8_t *out_buf,
                                  size_t out_cap,
                                  size_t *out_len,
                                  uint32_t timeout_ms)
{
    (void)out_type; (void)out_buf; (void)out_cap; (void)out_len; (void)timeout_ms;
    return ESP_ERR_INVALID_STATE;
}

esp_err_t rs3_ptp_proxy_server_start(void)
{
    ESP_LOGI(TAG, "PTP proxy disabled");
    return ESP_OK;
}

#endif // CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

