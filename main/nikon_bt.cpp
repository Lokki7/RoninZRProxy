#include "nikon_bt.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "log_tcp.h"
#include "ui_status.h"

#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
extern "C" {
#include "esp_nimble_hci.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/ble_sm.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "nvs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include "esp_timer.h"
}
#endif

static const char *TAG = "nikon_bt";

namespace {

#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED

constexpr char kNvsNs[] = "rs3_bt";
constexpr char kNvsKeyLastPeer[] = "nikon_peer";  // blob: [type:1][addr:6]

// Nikon "remote" service UUID (from furble).
static const ble_uuid128_t kNikonServiceUuid =
    BLE_UUID128_INIT(0x61, 0x55, 0xbd, 0xb9, 0xc7, 0x6d, 0x62, 0x8d,
                     0x55, 0x42, 0xd4, 0x3d, 0x00, 0xde, 0x00, 0x00);

// Nikon manufacturer company ID (from furble).
static constexpr uint16_t kNikonCompanyId = 0x0399;

// Nikon characteristic UUIDs (remote mode).
static const ble_uuid128_t kNikonChrPairRemoteUuid =
    BLE_UUID128_INIT(0x61, 0x55, 0xbd, 0xb9, 0xc7, 0x6d, 0x62, 0x8d,
                     0x55, 0x42, 0xd4, 0x3d, 0x87, 0x20, 0x00, 0x00);  // 0x2087
static const ble_uuid128_t kNikonChrShutterUuid =
    BLE_UUID128_INIT(0x61, 0x55, 0xbd, 0xb9, 0xc7, 0x6d, 0x62, 0x8d,
                     0x55, 0x42, 0xd4, 0x3d, 0x83, 0x20, 0x00, 0x00);  // 0x2083
static const ble_uuid128_t kNikonChrRemoteInd1Uuid =
    BLE_UUID128_INIT(0x61, 0x55, 0xbd, 0xb9, 0xc7, 0x6d, 0x62, 0x8d,
                     0x55, 0x42, 0xd4, 0x3d, 0x84, 0x20, 0x00, 0x00);  // 0x2084

struct StoredPeer {
    uint8_t addr_type = 0;
    uint8_t addr[6] = {0};
    uint8_t has_device_id = 0;
    uint32_t device_id_le = 0;
    uint8_t has_nonce = 0;
    uint32_t nonce_le = 0;
};

static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static ble_addr_t s_last_peer{};
static bool s_have_last_peer = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static esp_timer_handle_t s_reconnect_timer = nullptr;
static uint32_t s_backoff_ms = 1000;

static ble_addr_t s_scan_candidate{};
static bool s_scan_have_candidate = false;
static uint8_t s_scan_candidate_has_device_id = 0;
static uint32_t s_scan_candidate_device_id_le = 0;

static uint8_t s_pref_has_device_id = 0;
static uint32_t s_pref_device_id_le = 0;
static uint8_t s_pref_has_nonce = 0;
static uint32_t s_pref_nonce_le = 0;

static bool s_mode_pairing = false;
static bool s_do_pair_after_connect = false;
static bool s_pairing_in_progress = false;
static bool s_remote_session_ready = false;

// GATT discovered handles
static uint16_t s_svc_start = 0;
static uint16_t s_svc_end = 0;
static uint16_t s_pair_val_handle = 0;
static uint16_t s_shutter_val_handle = 0;
static uint16_t s_pair_cccd_handle = 0;
static uint16_t s_ind1_val_handle = 0;
static uint16_t s_ind1_cccd_handle = 0;
static uint16_t s_pair_end_handle = 0;
static uint16_t s_ind1_end_handle = 0;
static uint16_t s_shutter_end_handle = 0;

// Pairing exchange message (remote mode).
typedef struct __attribute__((packed)) {
    uint8_t stage;
    uint64_t timestamp;
    union __attribute__((packed)) {
        struct __attribute__((packed)) {
            uint32_t device;
            uint32_t nonce;
        } id;
        char serial[8];
    };
} nikon_pair_msg_t;

static_assert(sizeof(nikon_pair_msg_t) == 17, "nikon_pair_msg_t size must match Nikon remote pairing payload");

static QueueHandle_t s_cmd_q = nullptr;
static QueueHandle_t s_pair_rx_q = nullptr;
static SemaphoreHandle_t s_gatt_sem = nullptr;
static int s_gatt_rc = 0;
static bool s_app_task_started = false;
static uint16_t s_mtu = 0;
static SemaphoreHandle_t s_enc_sem = nullptr;
static int s_last_enc_status = -1;

typedef enum {
    CMD_PAIR_START,
    CMD_DO_PAIR_HANDSHAKE,
    CMD_SHUTTER_CLICK,
    CMD_CONNECT_CANDIDATE,
    CMD_REMOTE_SESSION_INIT,
} nikon_cmd_kind_t;

typedef struct {
    nikon_cmd_kind_t kind;
} nikon_cmd_t;

typedef struct {
    nikon_pair_msg_t msg;
    size_t len;
} nikon_pair_rx_t;

static void ui_bt_line(const char *s)
{
    (void)rs3_ui_status_bt_line(s);
}

static void bt_tcp_vlogf(const char *fmt, va_list ap)
{
    rs3_tcp_vlogf(fmt, ap);
}

static void bt_tcp_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void bt_tcp_logf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bt_tcp_vlogf(fmt, ap);
    va_end(ap);
}

static void nvs_save_last_peer(const ble_addr_t &peer)
{
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(kNvsNs, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        bt_tcp_logf("[BT] nvs_open failed: %s\r\n", esp_err_to_name(err));
        return;
    }

    StoredPeer sp{};
    sp.addr_type = peer.type;
    memcpy(sp.addr, peer.val, sizeof(sp.addr));
    sp.has_device_id = s_pref_has_device_id;
    sp.device_id_le = s_pref_device_id_le;
    sp.has_nonce = s_pref_has_nonce;
    sp.nonce_le = s_pref_nonce_le;

    err = nvs_set_blob(h, kNvsKeyLastPeer, &sp, sizeof(sp));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
        bt_tcp_logf("[BT] nvs_set_blob failed: %s\r\n", esp_err_to_name(err));
        (void)nvs_close(h);
        return;
    }

    err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        bt_tcp_logf("[BT] nvs_commit failed: %s\r\n", esp_err_to_name(err));
    }
    nvs_close(h);
}

static bool nvs_load_last_peer(ble_addr_t &peer_out)
{
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(kNvsNs, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return false;
    }

    // Backwards compatible read (older firmware stored a shorter blob).
    StoredPeer sp{};
    size_t len = 0;
    err = nvs_get_blob(h, kNvsKeyLastPeer, nullptr, &len);
    if (err != ESP_OK || len == 0) {
        nvs_close(h);
        return false;
    }
    if (len > sizeof(sp)) {
        len = sizeof(sp);
    }
    err = nvs_get_blob(h, kNvsKeyLastPeer, &sp, &len);
    nvs_close(h);

    if (err != ESP_OK) {
        return false;
    }

    peer_out.type = sp.addr_type;
    memcpy(peer_out.val, sp.addr, sizeof(sp.addr));
    s_pref_has_device_id = sp.has_device_id;
    s_pref_device_id_le = sp.device_id_le;
    s_pref_has_nonce = sp.has_nonce;
    s_pref_nonce_le = sp.nonce_le;
    return true;
}

static void log_addr(const char *prefix, const ble_addr_t &a)
{
    // NimBLE doesn't provide a stable addr->string helper across IDF versions; format manually.
    // Print MSB..LSB like common BLE MAC format.
    char buf[32];
    snprintf(buf, sizeof(buf),
             "%02X:%02X:%02X:%02X:%02X:%02X (t=%u)",
             a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0],
             (unsigned)a.type);
    ESP_LOGI(TAG, "%s%s", prefix, buf);
    bt_tcp_logf("[BT] %s%s\r\n", prefix, buf);
}

static void schedule_reconnect(uint32_t delay_ms);
static void stop_reconnect(void);
static void start_scan_for_nikon(uint32_t duration_ms);
static int connect_peer(const ble_addr_t &peer);
static void nikon_bt_task(void *arg);
static bool gatt_discover_all(uint16_t conn_handle);
static bool nikon_remote_pair(uint16_t conn_handle);
static bool nikon_remote_session_init(uint16_t conn_handle);
static bool nikon_shutter_click(uint16_t conn_handle);
static bool gatt_exchange_mtu(uint16_t conn_handle);
static void bt_security_start(uint16_t conn_handle);
static bool gatt_wait(uint32_t timeout_ms, const char *what);
static bool bt_wait_encryption(uint32_t timeout_ms);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_backoff_ms = 1000;
            s_remote_session_ready = false;
            ESP_LOGI(TAG, "connected (handle=%u)", s_conn_handle);
            bt_tcp_logf("[BT] connected handle=%u\r\n", s_conn_handle);
            ui_bt_line("BT: connected");

            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(s_conn_handle, &desc) == 0) {
                s_last_peer = desc.peer_id_addr;
                s_have_last_peer = true;
                nvs_save_last_peer(s_last_peer);
                log_addr("peer: ", s_last_peer);
            }

            if (s_do_pair_after_connect && s_cmd_q) {
                // Enqueue once; avoid duplicate pairing starts.
                s_do_pair_after_connect = false;
                nikon_cmd_t cmd = {.kind = CMD_DO_PAIR_HANDSHAKE};
                (void)xQueueSend(s_cmd_q, &cmd, 0);
            } else if (!s_mode_pairing && s_pref_has_device_id && s_cmd_q) {
                nikon_cmd_t cmd = {.kind = CMD_REMOTE_SESSION_INIT};
                (void)xQueueSend(s_cmd_q, &cmd, 0);
            }
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(TAG, "connect failed: status=%d", event->connect.status);
            bt_tcp_logf("[BT] connect failed status=%d\r\n", event->connect.status);
            ui_bt_line("BT: connect failed");
            schedule_reconnect(s_backoff_ms);
            s_backoff_ms = (s_backoff_ms < 30000) ? (s_backoff_ms * 2) : 30000;
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGW(TAG, "disconnected: reason=%d", event->disconnect.reason);
        bt_tcp_logf("[BT] disconnected reason=%d\r\n", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_pairing_in_progress = false;
        s_remote_session_ready = false;
        ui_bt_line("BT: disconnected");
        schedule_reconnect(s_backoff_ms);
        s_backoff_ms = (s_backoff_ms < 30000) ? (s_backoff_ms * 2) : 30000;
        return 0;
    }
    case BLE_GAP_EVENT_DISC: {
        // Scan result.
        const auto *d = &event->disc;
        struct ble_hs_adv_fields fields;
        memset(&fields, 0, sizeof(fields));
        if (ble_hs_adv_parse_fields(&fields, d->data, d->length_data) != 0) {
            return 0;
        }

        // Must advertise Nikon "remote" service UUID (128-bit).
        bool svc_ok = false;
        for (int i = 0; i < fields.num_uuids128; i++) {
            if (ble_uuid_cmp((const ble_uuid_t *)&fields.uuids128[i],
                             (const ble_uuid_t *)&kNikonServiceUuid) == 0) {
                svc_ok = true;
                break;
            }
        }
        if (!svc_ok) {
            return 0;
        }

        // Optional: manufacturer data helps disambiguate (and is required for "reconnect" matching).
        uint8_t has_dev_id = 0;
        uint32_t dev_id_le = 0;
        if (fields.mfg_data != nullptr && fields.mfg_data_len >= 7) {
            uint16_t company = (uint16_t)fields.mfg_data[0] | ((uint16_t)fields.mfg_data[1] << 8);
            if (company == kNikonCompanyId) {
                // Layout (little-endian) per furble: [company:u16][device:u32][zero:u8]
                dev_id_le = (uint32_t)fields.mfg_data[2] |
                            ((uint32_t)fields.mfg_data[3] << 8) |
                            ((uint32_t)fields.mfg_data[4] << 16) |
                            ((uint32_t)fields.mfg_data[5] << 24);
                has_dev_id = 1;
            }
        }

        if (!s_mode_pairing) {
            // Normal reconnect mode: If we have a preferred device_id, only accept matching one.
            if (s_pref_has_device_id && has_dev_id) {
                if (dev_id_le != s_pref_device_id_le) {
                    return 0;
                }
            } else if (s_pref_has_device_id && !has_dev_id) {
                // Can't prove it's our last paired camera.
                return 0;
            }
        }

        // Accept first suitable candidate and stop scanning.
        if (!s_scan_have_candidate) {
            s_scan_candidate = d->addr;
            s_scan_have_candidate = true;
            s_scan_candidate_has_device_id = has_dev_id;
            s_scan_candidate_device_id_le = dev_id_le;

            log_addr("scan match, addr=", s_scan_candidate);
            if (has_dev_id) {
                ESP_LOGI(TAG, "scan match device_id_le=0x%08" PRIx32, dev_id_le);
                bt_tcp_logf("[BT] scan match device_id_le=0x%08" PRIx32 "\r\n", dev_id_le);
            }

            // Stop any pending reconnect loop now that we have a candidate.
            stop_reconnect();

            // Ask app task to connect (avoids relying on DISC_COMPLETE timing).
            if (s_cmd_q) {
                nikon_cmd_t cmd = {.kind = CMD_CONNECT_CANDIDATE};
                (void)xQueueSend(s_cmd_q, &cmd, 0);
            }

            (void)ble_gap_disc_cancel();
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE: {
        if (!s_scan_have_candidate) {
            // No candidate found — try again with backoff.
            ui_bt_line("BT: scan timeout");
            bt_tcp_logf("[BT] scan timeout\r\n");
            schedule_reconnect(s_backoff_ms);
            s_backoff_ms = (s_backoff_ms < 30000) ? (s_backoff_ms * 2) : 30000;
        }
        return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        // Notifications / indications arrive here.
        const auto *n = &event->notify_rx;
        if (n->om != nullptr && OS_MBUF_PKTLEN(n->om) >= 1) {
            nikon_pair_rx_t rx{};
            rx.len = OS_MBUF_PKTLEN(n->om);
            if (rx.len > sizeof(rx.msg)) rx.len = sizeof(rx.msg);
            (void)os_mbuf_copydata(n->om, 0, rx.len, &rx.msg);
            // Always log first bytes for debugging.
            uint8_t b0 = rx.msg.stage;
            bt_tcp_logf("[BT] notify_rx handle=%u len=%u b0=0x%02X\r\n",
                        (unsigned)n->attr_handle, (unsigned)OS_MBUF_PKTLEN(n->om), b0);

            if (n->attr_handle == s_pair_val_handle && s_pair_rx_q != nullptr) {
                (void)xQueueSend(s_pair_rx_q, &rx, 0);
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption changed: status=%d", event->enc_change.status);
        bt_tcp_logf("[BT] enc_change status=%d\r\n", event->enc_change.status);
        s_last_enc_status = event->enc_change.status;
        if (s_enc_sem) {
            xSemaphoreGive(s_enc_sem);
        }
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        // We'll use this later when we add Nikon shutter/record GATT pieces.
        return 0;
    default:
        return 0;
    }
}

static int connect_peer(const ble_addr_t &peer)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "already connected (handle=%u)", s_conn_handle);
        bt_tcp_logf("[BT] already connected handle=%u\r\n", s_conn_handle);
        return 0;
    }

    // Ensure any scan is stopped before attempting connection.
    (void)ble_gap_disc_cancel();

    struct ble_gap_conn_params params;
    memset(&params, 0, sizeof(params));
    params.scan_itvl = 0x0010;
    params.scan_window = 0x0010;
    params.itvl_min = 0x0018;
    params.itvl_max = 0x0028;
    params.latency = 0;
    params.supervision_timeout = 0x0100;
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    log_addr("connecting to: ", peer);
    int rc = ble_gap_connect(s_own_addr_type, &peer, 30000 /*ms*/, &params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_connect rc=%d", rc);
        bt_tcp_logf("[BT] ble_gap_connect rc=%d\r\n", rc);
    }
    return rc;
}

static void reconnect_timer_cb(void *arg)
{
    (void)arg;

    // Prefer scanning: Nikon cameras may use rotating private addresses.
    start_scan_for_nikon(30000);
}

static void stop_reconnect(void)
{
    if (s_reconnect_timer) {
        (void)esp_timer_stop(s_reconnect_timer);
    }
}

static void schedule_reconnect(uint32_t delay_ms)
{
    if (s_reconnect_timer == nullptr) {
        const esp_timer_create_args_t args = {
            .callback = &reconnect_timer_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "nikon_reconn",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&args, &s_reconnect_timer));
    }

    (void)esp_timer_stop(s_reconnect_timer);
    ESP_LOGI(TAG, "reconnect in %" PRIu32 " ms", delay_ms);
    bt_tcp_logf("[BT] reconnect in %" PRIu32 " ms\r\n", delay_ms);
    ESP_ERROR_CHECK(esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000ULL));
}

static void start_scan_for_nikon(uint32_t duration_ms)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    s_scan_have_candidate = false;
    s_scan_candidate_has_device_id = 0;
    s_scan_candidate_device_id_le = 0;

    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    params.passive = 0;
    params.itvl = 0x0010;
    params.window = 0x0010;
    params.filter_duplicates = 1;

    ESP_LOGI(TAG, "scan for Nikon (%" PRIu32 " ms)%s%s", duration_ms,
             s_mode_pairing ? " [pairing]" : "",
             (!s_mode_pairing && s_pref_has_device_id) ? " [match last device_id]" : "");
    ui_bt_line(s_mode_pairing ? "BT: scanning (pair)" : "BT: scanning");
    bt_tcp_logf("[BT] scan start duration=%" PRIu32 "ms%s%s\r\n",
                duration_ms,
                s_mode_pairing ? " pairing" : "",
                (!s_mode_pairing && s_pref_has_device_id) ? " match_last_device_id" : "");

    int rc = ble_gap_disc(s_own_addr_type, (int32_t)duration_ms, &params, gap_event, nullptr);
    if (rc == BLE_HS_EALREADY) {
        // Scan already active; treat as success.
        bt_tcp_logf("[BT] scan already active\r\n");
        return;
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc rc=%d", rc);
        bt_tcp_logf("[BT] ble_gap_disc rc=%d\r\n", rc);
        schedule_reconnect(s_backoff_ms);
        s_backoff_ms = (s_backoff_ms < 30000) ? (s_backoff_ms * 2) : 30000;
    }
}

static void on_sync(void)
{
    // Figure out address type (public/random).
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }

    // Useful name for debugging / system menus.
    ble_svc_gap_device_name_set("rs3proxy");

    bt_tcp_logf("[BT] cfg: SECURITY_ENABLE=%d SM_LEGACY=%d SM_SC=%d NVS_PERSIST=%d\r\n",
                (int)CONFIG_BT_NIMBLE_SECURITY_ENABLE,
                (int)CONFIG_BT_NIMBLE_SM_LEGACY,
                (int)CONFIG_BT_NIMBLE_SM_SC,
                (int)CONFIG_BT_NIMBLE_NVS_PERSIST);

    // Load last peer/device_id preference (if present), then scan+connect.
    ble_addr_t peer{};
    if (nvs_load_last_peer(peer)) {
        s_last_peer = peer;
        s_have_last_peer = true;
        log_addr("last peer (nvs): ", s_last_peer);
        if (s_pref_has_device_id) {
            ESP_LOGI(TAG, "last device_id_le=0x%08" PRIx32, s_pref_device_id_le);
            bt_tcp_logf("[BT] last device_id_le=0x%08" PRIx32 "\r\n", s_pref_device_id_le);
        }
    }
    start_scan_for_nikon(30000);
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "reset; reason=%d", reason);
    bt_tcp_logf("[BT] reset reason=%d\r\n", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

class NikonBtManager {
public:
    esp_err_t start()
    {
        // Initialize controller + NimBLE host stack.
        // In ESP-IDF 6.x, nimble_port_init() handles controller init internally.
        int rc = nimble_port_init();
        if (rc != 0) {
            ESP_LOGE(TAG, "nimble_port_init failed: rc=%d", rc);
            bt_tcp_logf("[BT] nimble_port_init failed rc=%d\r\n", rc);
            return ESP_FAIL;
        }

        // Standard GAP/GATT services + NVS-backed store (if enabled in sdkconfig).
        ble_svc_gap_init();
        ble_svc_gatt_init();

        // Enable NimBLE store backend so Security Manager (SMP) can read/write peer keys.
        // In upstream NimBLE this is done by ble_store_config_init() via sysinit, but in ESP-IDF
        // we wire the callbacks directly to avoid SYSINIT_ASSERT_ACTIVE() issues.
        ble_hs_cfg.store_read_cb = ble_store_config_read;
        ble_hs_cfg.store_write_cb = ble_store_config_write;
        ble_hs_cfg.store_delete_cb = ble_store_config_delete;

        ble_hs_cfg.reset_cb = on_reset;
        ble_hs_cfg.sync_cb = on_sync;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        // Pairing/bonding defaults (we'll refine once we implement Nikon specifics).
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_sc = 1;
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
        // Request encryption (Just Works) and distribute keys; some peers won't encrypt without this.
        ble_hs_cfg.sm_sec_lvl = 2;  // unauthenticated pairing with encryption
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        if (s_cmd_q == nullptr) s_cmd_q = xQueueCreate(8, sizeof(nikon_cmd_t));
        if (s_pair_rx_q == nullptr) s_pair_rx_q = xQueueCreate(8, sizeof(nikon_pair_rx_t));
        if (s_gatt_sem == nullptr) s_gatt_sem = xSemaphoreCreateBinary();
        if (s_enc_sem == nullptr) s_enc_sem = xSemaphoreCreateBinary();

        // App task (UI commands -> BLE actions) — start once.
        if (!s_app_task_started) {
            s_app_task_started = true;
            xTaskCreate(nikon_bt_task, "nikon_bt", 6144, nullptr, 4, nullptr);
        }

        nimble_port_freertos_init(host_task);
        ESP_LOGI(TAG, "nimble started");
        ui_bt_line("BT: init");
        bt_tcp_logf("[BT] nimble started\r\n");
        return ESP_OK;
    }
};

static NikonBtManager s_mgr;

static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (error->status == 0 && svc) {
        s_svc_start = svc->start_handle;
        s_svc_end = svc->end_handle;
    }
    if (error->status == BLE_HS_EDONE) {
        s_gatt_rc = 0;
        xSemaphoreGive(s_gatt_sem);
    } else if (error->status != 0) {
        s_gatt_rc = error->status;
        xSemaphoreGive(s_gatt_sem);
    }
    return 0;
}

static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn_handle;
    const ble_uuid_t *want = (const ble_uuid_t *)arg;
    if (error->status == 0 && chr && want) {
        if (ble_uuid_cmp(&chr->uuid.u, want) == 0) {
            // Record the discovered characteristic value handle.
            if (ble_uuid_cmp(want, (const ble_uuid_t *)&kNikonChrPairRemoteUuid) == 0) {
                s_pair_val_handle = chr->val_handle;
            } else if (ble_uuid_cmp(want, (const ble_uuid_t *)&kNikonChrShutterUuid) == 0) {
                s_shutter_val_handle = chr->val_handle;
            } else if (ble_uuid_cmp(want, (const ble_uuid_t *)&kNikonChrRemoteInd1Uuid) == 0) {
                s_ind1_val_handle = chr->val_handle;
            }
        }
    }
    if (error->status == BLE_HS_EDONE) {
        s_gatt_rc = 0;
        xSemaphoreGive(s_gatt_sem);
    } else if (error->status != 0) {
        s_gatt_rc = error->status;
        xSemaphoreGive(s_gatt_sem);
    }
    return 0;
}

typedef struct {
    uint16_t def_handle;
    uint16_t val_handle;
    ble_uuid_any_t uuid;
} chr_info_t;

static chr_info_t s_chrs[24];
static size_t s_chrs_len = 0;

static int on_disc_all_chrs(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (error->status == 0 && chr) {
        if (s_chrs_len < (sizeof(s_chrs) / sizeof(s_chrs[0]))) {
            s_chrs[s_chrs_len].def_handle = chr->def_handle;
            s_chrs[s_chrs_len].val_handle = chr->val_handle;
            s_chrs[s_chrs_len].uuid = chr->uuid;
            s_chrs_len++;
        }
    }
    if (error->status == BLE_HS_EDONE) {
        s_gatt_rc = 0;
        xSemaphoreGive(s_gatt_sem);
    } else if (error->status != 0) {
        s_gatt_rc = error->status;
        xSemaphoreGive(s_gatt_sem);
    }
    return 0;
}

static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)conn_handle;
    (void)chr_val_handle;
    const char *which = (const char *)arg;
    if (error->status == 0 && dsc) {
        // CCCD = 0x2902
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            if (which && strcmp(which, "ind1") == 0) {
                s_ind1_cccd_handle = dsc->handle;
            } else {
                s_pair_cccd_handle = dsc->handle;
            }
        }
    }
    if (error->status == BLE_HS_EDONE) {
        s_gatt_rc = 0;
        xSemaphoreGive(s_gatt_sem);
    } else if (error->status != 0) {
        s_gatt_rc = error->status;
        xSemaphoreGive(s_gatt_sem);
    }
    return 0;
}

static int on_write(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    s_gatt_rc = error->status;
    xSemaphoreGive(s_gatt_sem);
    return 0;
}

static nikon_pair_rx_t s_last_read{};

static int on_read(uint16_t conn_handle, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (error->status == 0 && attr && attr->om) {
        size_t len = OS_MBUF_PKTLEN(attr->om);
        if (len > sizeof(s_last_read.msg)) len = sizeof(s_last_read.msg);
        memset(&s_last_read, 0, sizeof(s_last_read));
        s_last_read.len = len;
        (void)os_mbuf_copydata(attr->om, 0, len, &s_last_read.msg);
        bt_tcp_logf("[BT] read(pair) len=%u stage=0x%02X\r\n", (unsigned)OS_MBUF_PKTLEN(attr->om), s_last_read.msg.stage);
        s_gatt_rc = 0;
    } else {
        bt_tcp_logf("[BT] read(pair) failed rc=%d\r\n", error->status);
        s_gatt_rc = error->status;
    }
    xSemaphoreGive(s_gatt_sem);
    return 0;
}

static bool gatt_read_pair(uint16_t conn_handle, uint32_t timeout_ms)
{
    s_gatt_rc = 0;
    int rc = ble_gattc_read(conn_handle, s_pair_val_handle, on_read, nullptr);
    if (rc != 0) {
        bt_tcp_logf("[BT] gattc_read rc=%d\r\n", rc);
        return false;
    }
    return gatt_wait(timeout_ms, "read(pair)");
}

static int on_mtu(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (error->status == 0) {
        s_mtu = mtu;
        bt_tcp_logf("[BT] mtu=%u\r\n", (unsigned)mtu);
        s_gatt_rc = 0;
    } else {
        bt_tcp_logf("[BT] mtu exch failed rc=%d\r\n", error->status);
        s_gatt_rc = error->status;
    }
    xSemaphoreGive(s_gatt_sem);
    return 0;
}

static bool gatt_exchange_mtu(uint16_t conn_handle)
{
    s_gatt_rc = 0;
    int rc = ble_gattc_exchange_mtu(conn_handle, on_mtu, nullptr);
    if (rc == BLE_HS_EALREADY) {
        // MTU already exchanged / procedure already active; not an error.
        bt_tcp_logf("[BT] mtu exch already active\r\n");
        return true;
    }
    if (rc != 0) {
        bt_tcp_logf("[BT] mtu exch start rc=%d\r\n", rc);
        return false;
    }
    return gatt_wait(3000, "mtu");
}

static void bt_security_start(uint16_t conn_handle)
{
    // Best-effort; some cameras require encryption/bonding before sending indications.
    s_last_enc_status = -1;
    if (s_enc_sem) {
        (void)xSemaphoreTake(s_enc_sem, 0);
    }
    int rc = ble_gap_security_initiate(conn_handle);
    bt_tcp_logf("[BT] security_initiate rc=%d\r\n", rc);
    if (rc == BLE_HS_ENOTSUP) {
        bt_tcp_logf("[BT] security not supported (ENOTSUP). build cfg: SECURITY_ENABLE=%d SM_LEGACY=%d SM_SC=%d\r\n",
                    (int)CONFIG_BT_NIMBLE_SECURITY_ENABLE,
                    (int)CONFIG_BT_NIMBLE_SM_LEGACY,
                    (int)CONFIG_BT_NIMBLE_SM_SC);
        bt_tcp_logf("[BT] if these are 1, then ENOTSUP is coming from host state (e.g. not synced) or API usage.\r\n");
    }
}

static bool bt_wait_encryption(uint32_t timeout_ms)
{
    if (!s_enc_sem) return false;
    if (xSemaphoreTake(s_enc_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        bt_tcp_logf("[BT] enc_wait timeout\r\n");
        return false;
    }
    bt_tcp_logf("[BT] enc_wait status=%d\r\n", s_last_enc_status);
    return (s_last_enc_status == 0);
}

static bool gatt_wait(uint32_t timeout_ms, const char *what)
{
    if (s_gatt_sem == nullptr) return false;
    if (xSemaphoreTake(s_gatt_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "%s: timeout", what);
        return false;
    }
    if (s_gatt_rc != 0) {
        ESP_LOGW(TAG, "%s: rc=%d", what, s_gatt_rc);
        return false;
    }
    return true;
}

static bool gatt_discover_all(uint16_t conn_handle)
{
    s_svc_start = s_svc_end = 0;
    s_pair_val_handle = s_shutter_val_handle = 0;
    s_pair_cccd_handle = 0;
    s_ind1_val_handle = 0;
    s_ind1_cccd_handle = 0;
    s_pair_end_handle = 0;
    s_ind1_end_handle = 0;
    s_shutter_end_handle = 0;
    s_chrs_len = 0;

    // Discover Nikon service.
    s_gatt_rc = 0;
    int rc = ble_gattc_disc_svc_by_uuid(conn_handle, (const ble_uuid_t *)&kNikonServiceUuid,
                                        on_disc_svc, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_svc rc=%d", rc);
        return false;
    }
    if (!gatt_wait(5000, "disc_svc")) return false;
    if (s_svc_start == 0 || s_svc_end == 0) {
        ESP_LOGW(TAG, "nikon service not found");
        return false;
    }

    // Discover all characteristics to compute correct descriptor ranges per characteristic.
    s_gatt_rc = 0;
    rc = ble_gattc_disc_all_chrs(conn_handle, s_svc_start, s_svc_end, on_disc_all_chrs, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_all_chrs rc=%d", rc);
        return false;
    }
    if (!gatt_wait(5000, "disc_all_chrs")) return false;

    // Find target characteristics and their end handle (= next def_handle - 1).
    for (size_t i = 0; i < s_chrs_len; i++) {
        const uint16_t end = (i + 1 < s_chrs_len) ? (uint16_t)(s_chrs[i + 1].def_handle - 1) : s_svc_end;
        const ble_uuid_t *u = &s_chrs[i].uuid.u;
        if (ble_uuid_cmp(u, (const ble_uuid_t *)&kNikonChrPairRemoteUuid) == 0) {
            s_pair_val_handle = s_chrs[i].val_handle;
            s_pair_end_handle = end;
        } else if (ble_uuid_cmp(u, (const ble_uuid_t *)&kNikonChrShutterUuid) == 0) {
            s_shutter_val_handle = s_chrs[i].val_handle;
            s_shutter_end_handle = end;
        } else if (ble_uuid_cmp(u, (const ble_uuid_t *)&kNikonChrRemoteInd1Uuid) == 0) {
            s_ind1_val_handle = s_chrs[i].val_handle;
            s_ind1_end_handle = end;
        }
    }

    if (s_pair_val_handle == 0 || s_pair_end_handle == 0) {
        ESP_LOGW(TAG, "pair characteristic not found");
        return false;
    }
    if (s_shutter_val_handle == 0 || s_shutter_end_handle == 0) {
        ESP_LOGW(TAG, "shutter characteristic not found");
        return false;
    }

    // Discover CCCD for pairing characteristic (search until end of service).
    s_gatt_rc = 0;
    rc = ble_gattc_disc_all_dscs(conn_handle, s_pair_val_handle, s_pair_end_handle, on_disc_dsc, (void *)"pair");
    if (rc != 0) {
        ESP_LOGW(TAG, "disc_dsc rc=%d", rc);
        return false;
    }
    if (!gatt_wait(5000, "disc_dsc")) return false;
    if (s_pair_cccd_handle == 0) {
        ESP_LOGW(TAG, "pair CCCD not found");
        return false;
    }

    // Discover CCCD for ind1 characteristic, if present.
    if (s_ind1_val_handle != 0 && s_ind1_end_handle != 0) {
        s_gatt_rc = 0;
        rc = ble_gattc_disc_all_dscs(conn_handle, s_ind1_val_handle, s_ind1_end_handle, on_disc_dsc, (void *)"ind1");
        if (rc != 0) {
            ESP_LOGW(TAG, "disc_dsc(ind1) rc=%d", rc);
            return false;
        }
        if (!gatt_wait(5000, "disc_dsc(ind1)")) return false;
    }

    ESP_LOGI(TAG, "gatt ok: svc=[%u..%u] pair=%u cccd=%u shutter=%u ind1=%u",
             s_svc_start, s_svc_end, s_pair_val_handle, s_pair_cccd_handle, s_shutter_val_handle, s_ind1_val_handle);
    bt_tcp_logf("[BT] gatt ok svc=[%u..%u] pair=%u cccd=%u shutter=%u ind1=%u\r\n",
                s_svc_start, s_svc_end, s_pair_val_handle, s_pair_cccd_handle, s_shutter_val_handle, s_ind1_val_handle);
    return true;
}

static bool gatt_write_flat(uint16_t conn_handle, uint16_t handle, const void *data, uint16_t len,
                            uint32_t timeout_ms, const char *what)
{
    s_gatt_rc = 0;
    int rc = ble_gattc_write_flat(conn_handle, handle, data, len, on_write, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "%s: write_flat rc=%d", what, rc);
        return false;
    }
    return gatt_wait(timeout_ms, what);
}

static bool nikon_remote_handshake(uint16_t conn_handle, const char *what, bool persist_ids, bool force_new_ids)
{
    if (s_pairing_in_progress) {
        bt_tcp_logf("[BT] %s already in progress; skip\r\n", what);
        return false;
    }
    s_pairing_in_progress = true;
    s_remote_session_ready = false;

    ui_bt_line(persist_ids ? "BT: pairing..." : "BT: session...");
    bt_tcp_logf("[BT] %s start\r\n", what);

    (void)gatt_exchange_mtu(conn_handle);

    if (!gatt_discover_all(conn_handle)) {
        ui_bt_line("BT: fail (gatt)");
        bt_tcp_logf("[BT] %s failed: gatt discovery\r\n", what);
        s_pairing_in_progress = false;
        return false;
    }

    // furble subscribes to INDICATIONS for remote mode.
    const uint8_t cccd_indicate[2] = {0x02, 0x00};
    if (!gatt_write_flat(conn_handle, s_pair_cccd_handle, cccd_indicate, sizeof(cccd_indicate),
                         5000, "cccd(pair)")) {
        ui_bt_line("BT: fail (cccd)");
        bt_tcp_logf("[BT] %s failed: enable indications\r\n", what);
        s_pairing_in_progress = false;
        return false;
    }
    bt_tcp_logf("[BT] cccd(pair)=ok handle=%u\r\n", (unsigned)s_pair_cccd_handle);
    if (s_ind1_cccd_handle != 0) {
        if (gatt_write_flat(conn_handle, s_ind1_cccd_handle, cccd_indicate, sizeof(cccd_indicate),
                            5000, "cccd(ind1)")) {
            bt_tcp_logf("[BT] cccd(ind1)=ok handle=%u\r\n", (unsigned)s_ind1_cccd_handle);
        }
    }

    // Determine the remote IDs to use.
    uint32_t device_le = 0;
    uint32_t nonce_le = 0;
    if (force_new_ids || !s_pref_has_device_id) {
        uint32_t device_host = esp_random();
        device_host = (device_host & 0x00FFFFFFu) | 0x01000000u;
        uint32_t nonce_host = esp_random();
        device_le = __builtin_bswap32(device_host);
        nonce_le = __builtin_bswap32(nonce_host);
    } else {
        device_le = s_pref_device_id_le;
        if (s_pref_has_nonce) {
            nonce_le = s_pref_nonce_le;
        } else {
            // Older firmware didn't persist nonce. Try a random one; if it works, store it.
            nonce_le = __builtin_bswap32(esp_random());
        }
    }

    bt_tcp_logf("[BT] %s ids device_id_le=0x%08" PRIx32 " nonce_le=0x%08" PRIx32 "%s\r\n",
                what, device_le, nonce_le, (s_pref_has_nonce ? "" : " (nonce new)"));

    if (persist_ids) {
        s_pref_has_device_id = 1;
        s_pref_device_id_le = device_le;
        s_pref_has_nonce = 1;
        s_pref_nonce_le = nonce_le;
        if (s_have_last_peer) {
            nvs_save_last_peer(s_last_peer);
        }
    }

    // Flush any stale pairing messages.
    if (s_pair_rx_q) {
        nikon_pair_rx_t tmp;
        while (xQueueReceive(s_pair_rx_q, &tmp, 0) == pdTRUE) {}
    }

    // Stage 1: timestamp endianness differs across models; try both variants.
    const uint64_t ts_try[2] = {__builtin_bswap64(0x01ull), 0x01ull};
    nikon_pair_rx_t rx{};
    bool got_stage2 = false;
    nikon_pair_msg_t tx{};
    for (int attempt = 0; attempt < 2 && !got_stage2; attempt++) {
        if (s_pair_rx_q) {
            nikon_pair_rx_t tmp;
            while (xQueueReceive(s_pair_rx_q, &tmp, 0) == pdTRUE) {}
        }

        memset(&tx, 0, sizeof(tx));
        tx.stage = 0x01;
        tx.timestamp = ts_try[attempt];
        tx.id.device = device_le;
        tx.id.nonce = nonce_le;

        bt_tcp_logf("[BT] %s stage1 try=%d ts=0x%016" PRIx64 " payload=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                    what,
                    attempt,
                    (uint64_t)tx.timestamp,
                    ((const uint8_t *)&tx)[0], ((const uint8_t *)&tx)[1], ((const uint8_t *)&tx)[2],
                    ((const uint8_t *)&tx)[3], ((const uint8_t *)&tx)[4], ((const uint8_t *)&tx)[5],
                    ((const uint8_t *)&tx)[6], ((const uint8_t *)&tx)[7], ((const uint8_t *)&tx)[8],
                    ((const uint8_t *)&tx)[9], ((const uint8_t *)&tx)[10], ((const uint8_t *)&tx)[11],
                    ((const uint8_t *)&tx)[12], ((const uint8_t *)&tx)[13], ((const uint8_t *)&tx)[14],
                    ((const uint8_t *)&tx)[15], ((const uint8_t *)&tx)[16]);

        if (!gatt_write_flat(conn_handle, s_pair_val_handle, &tx, sizeof(tx), 5000, "pair(stage1)")) {
            ui_bt_line("BT: fail (s1)");
            bt_tcp_logf("[BT] %s failed: stage1 write\r\n", what);
            s_pairing_in_progress = false;
            return false;
        }
        bt_tcp_logf("[BT] %s stage1 sent\r\n", what);

        if (xQueueReceive(s_pair_rx_q, &rx, pdMS_TO_TICKS(1500)) == pdTRUE && rx.msg.stage == 0x02) {
            got_stage2 = true;
            bt_tcp_logf("[BT] %s stage2 ok (notify)\r\n", what);
        } else {
            got_stage2 = false;
            const int polls = 50;
            for (int i = 0; i < polls && !got_stage2; i++) {
                if (gatt_read_pair(conn_handle, 2000) && s_last_read.msg.stage == 0x02) {
                    got_stage2 = true;
                    bt_tcp_logf("[BT] %s stage2 ok (read)\r\n", what);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            if (!got_stage2) {
                bt_tcp_logf("[BT] %s stage2 timeout (try=%d)\r\n", what, attempt);
            }
        }
    }

    if (!got_stage2) {
        ui_bt_line("BT: fail (s2)");
        bt_tcp_logf("[BT] %s failed: stage2 timeout\r\n", what);
        s_pairing_in_progress = false;
        return false;
    }

    // Stage 3: all zeros except stage (furble remote mode).
    memset(&tx, 0, sizeof(tx));
    tx.stage = 0x03;
    if (!gatt_write_flat(conn_handle, s_pair_val_handle, &tx, sizeof(tx), 5000, "pair(stage3)")) {
        ui_bt_line("BT: fail (s3)");
        bt_tcp_logf("[BT] %s failed: stage3 write\r\n", what);
        s_pairing_in_progress = false;
        return false;
    }

    // Wait for stage 4 (contains serial).
    bool got_stage4 = false;
    if (xQueueReceive(s_pair_rx_q, &rx, pdMS_TO_TICKS(1500)) == pdTRUE && rx.msg.stage == 0x04) {
        got_stage4 = true;
    } else {
        const int polls = 50;
        for (int i = 0; i < polls && !got_stage4; i++) {
            if (gatt_read_pair(conn_handle, 2000) && s_last_read.msg.stage == 0x04) {
                rx.msg = s_last_read.msg;
                rx.len = s_last_read.len;
                got_stage4 = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    if (!got_stage4 || rx.len < offsetof(nikon_pair_msg_t, serial) + 8) {
        ui_bt_line("BT: fail (s4)");
        bt_tcp_logf("[BT] %s failed: stage4 timeout/mismatch\r\n", what);
        s_pairing_in_progress = false;
        return false;
    }

    char serial[9] = {0};
    memcpy(serial, rx.msg.serial, 8);
    bt_tcp_logf("[BT] %s ok camera_serial=%s\r\n", what, serial);

    // If nonce wasn't persisted (old fw) and this worked, store it now.
    if (!persist_ids && s_pref_has_device_id && !s_pref_has_nonce) {
        s_pref_has_nonce = 1;
        s_pref_nonce_le = nonce_le;
        if (s_have_last_peer) {
            nvs_save_last_peer(s_last_peer);
        }
        bt_tcp_logf("[BT] %s stored nonce_le=0x%08" PRIx32 "\r\n", what, nonce_le);
    }

    s_remote_session_ready = true;
    ui_bt_line(persist_ids ? "BT: paired" : "BT: ready");
    bt_tcp_logf("[BT] %s done\r\n", what);
    s_pairing_in_progress = false;
    return true;
}

static bool nikon_remote_pair(uint16_t conn_handle)
{
    return nikon_remote_handshake(conn_handle, "pairing", true, true);
}

static bool nikon_remote_session_init(uint16_t conn_handle)
{
    if (!s_pref_has_device_id) {
        bt_tcp_logf("[BT] session init skipped: no saved device_id\r\n");
        return false;
    }
    return nikon_remote_handshake(conn_handle, "session", false, false);
}

static bool nikon_shutter_click(uint16_t conn_handle)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ui_bt_line("BT: not connected");
        bt_tcp_logf("[BT] shutter: not connected\r\n");
        return false;
    }
    if (s_shutter_val_handle == 0) {
        // Lazy discovery if needed.
        if (!gatt_discover_all(conn_handle)) {
            ui_bt_line("BT: shutter fail (gatt)");
            bt_tcp_logf("[BT] shutter: gatt discovery failed\r\n");
            return false;
        }
    }

    // After reboot/reconnect Nikon expects the remote handshake again before accepting shutter writes.
    if (!s_remote_session_ready && s_pref_has_device_id) {
        bt_tcp_logf("[BT] shutter: session not ready -> init\r\n");
        if (!nikon_remote_session_init(conn_handle)) {
            ui_bt_line("BT: need pair");
            bt_tcp_logf("[BT] shutter: session init failed\r\n");
            return false;
        }
    }

    // Nikon shutter command: {MODE_SHUTTER=0x02, CMD_PRESS=0x02}, then release {0x02, 0x00}.
    const uint8_t press[2] = {0x02, 0x02};
    const uint8_t release[2] = {0x02, 0x00};
    if (!gatt_write_flat(conn_handle, s_shutter_val_handle, press, sizeof(press), 3000, "shutter(press)")) {
        ui_bt_line("BT: shutter fail (press)");
        bt_tcp_logf("[BT] shutter: press failed\r\n");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    if (!gatt_write_flat(conn_handle, s_shutter_val_handle, release, sizeof(release), 3000, "shutter(release)")) {
        ui_bt_line("BT: shutter fail (release)");
        bt_tcp_logf("[BT] shutter: release failed\r\n");
        return false;
    }
    ui_bt_line("BT: shutter");
    bt_tcp_logf("[BT] shutter: click ok\r\n");
    return true;
}

static void nikon_bt_task(void *arg)
{
    (void)arg;
    nikon_cmd_t cmd{};
    while (true) {
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (cmd.kind) {
        case CMD_PAIR_START:
            s_mode_pairing = true;
            s_do_pair_after_connect = true;
            ui_bt_line("BT: pair start");
            bt_tcp_logf("[BT] pair button: cancel scan/reconnect\r\n");
            stop_reconnect();
            (void)ble_gap_disc_cancel();
            // If already connected, just do handshake.
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                nikon_remote_pair(s_conn_handle);
                s_mode_pairing = false;
                s_do_pair_after_connect = false;
            } else {
                start_scan_for_nikon(60000);
            }
            break;
        case CMD_DO_PAIR_HANDSHAKE:
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                (void)nikon_remote_pair(s_conn_handle);
            }
            s_mode_pairing = false;
            s_do_pair_after_connect = false;
            break;
        case CMD_REMOTE_SESSION_INIT:
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                (void)nikon_remote_session_init(s_conn_handle);
            }
            break;
        case CMD_SHUTTER_CLICK:
            (void)nikon_shutter_click(s_conn_handle);
            break;
        case CMD_CONNECT_CANDIDATE: {
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) break;
            if (!s_scan_have_candidate) break;

            // Capture device_id for future "last camera" matching.
            if (s_scan_candidate_has_device_id) {
                s_pref_has_device_id = 1;
                s_pref_device_id_le = s_scan_candidate_device_id_le;
            }

            ble_addr_t peer = s_scan_candidate;
            s_scan_have_candidate = false;

            (void)ble_gap_disc_cancel();
            vTaskDelay(pdMS_TO_TICKS(50));
            (void)connect_peer(peer);
            break;
        }
        default:
            break;
        }
    }
}

#endif  // CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED

}  // namespace

extern "C" esp_err_t rs3_nikon_bt_start(void)
{
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    return s_mgr.start();
#else
    ESP_LOGW(TAG, "Bluetooth/NimBLE disabled in sdkconfig; not starting");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

extern "C" esp_err_t rs3_nikon_bt_pair_start(void)
{
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    if (s_cmd_q == nullptr) return ESP_ERR_INVALID_STATE;
    nikon_cmd_t cmd = {.kind = CMD_PAIR_START};
    return (xQueueSend(s_cmd_q, &cmd, 0) == pdTRUE) ? ESP_OK : ESP_FAIL;
#else
    rs3_tcp_logf("[BT] pair_start: BT disabled in sdkconfig\r\n");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

extern "C" esp_err_t rs3_nikon_bt_shutter_click(void)
{
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
    if (s_cmd_q == nullptr) return ESP_ERR_INVALID_STATE;
    nikon_cmd_t cmd = {.kind = CMD_SHUTTER_CLICK};
    return (xQueueSend(s_cmd_q, &cmd, 0) == pdTRUE) ? ESP_OK : ESP_FAIL;
#else
    rs3_tcp_logf("[BT] shutter_click: BT disabled in sdkconfig\r\n");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

