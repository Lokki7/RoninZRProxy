#include "usb_ptp_cam.h"

#include "sdkconfig.h"

#ifndef CONFIG_RS3_USB_PTP_ENABLE
#define CONFIG_RS3_USB_PTP_ENABLE 0
#endif

// Kconfig symbol may be absent when hidden by depends-on; keep build stable.
#ifndef CONFIG_RS3_USB_PTP_BCD_DEVICE
#define CONFIG_RS3_USB_PTP_BCD_DEVICE 0x0100
#endif

#if CONFIG_RS3_USB_PTP_ENABLE

#if CONFIG_RS3_USB_PTP_IMPL_LEGACY && !CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

#include <inttypes.h>
#include <string.h>

#include "esp_check.h"

#include "tinyusb.h"
#include "tusb.h"

#include "device/usbd_pvt.h"

#include "log_tcp.h"
#include "tcp_server.h"
#include "ui_status.h"
#include "rec_events.h"
// (no TCP proxying in this module; see `usb_ptp_proxy.c` for raw passthrough proxy)

#include <stdarg.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -----------------------------
// PTP/MTP constants (subset)
// -----------------------------

// PTP Container types
#define PTP_CT_COMMAND  1
#define PTP_CT_DATA     2
#define PTP_CT_RESPONSE 3
#define PTP_CT_EVENT    4

// PTP Operation codes (subset)
#define PTP_OC_GET_DEVICE_INFO 0x1001
#define PTP_OC_OPEN_SESSION    0x1002
#define PTP_OC_CLOSE_SESSION   0x1003
#define PTP_OC_GET_STORAGE_IDS     0x1004
#define PTP_OC_GET_STORAGE_INFO    0x1005
#define PTP_OC_GET_NUM_OBJECTS     0x1006
#define PTP_OC_GET_OBJECT_HANDLES  0x1007
// Vendor ops observed from RS3 when talking to Sony ILCE-5100
#define PTP_OC_SONY_9201 0x9201
#define PTP_OC_SONY_9202 0x9202
#define PTP_OC_SONY_9207 0x9207 // 2-stage: COMMAND then host->device DATA (5 bytes). Used for start/stop recording.
#define PTP_OC_SONY_9209 0x9209 // returns a large dataset (~1KB)

// PTP Response codes (subset)
#define PTP_RC_OK                    0x2001
#define PTP_RC_OPERATION_NOT_SUPPORTED 0x2005

typedef struct __attribute__((packed)) {
    uint32_t len;
    uint16_t type;
    uint16_t code;
    uint32_t trans_id;
} ptp_hdr_t;

typedef struct __attribute__((packed)) {
    ptp_hdr_t hdr;
    uint32_t params[5];
} ptp_cmd_t;

// Endpoints (Full-speed)
// Match the real Sony ILCE-5100 enumeration: bulk IN/OUT only (no interrupt/event endpoint).
static const uint8_t EP_BULK_IN = 0x81;
static const uint8_t EP_BULK_OUT= 0x02;

// Still Image (PTP) class-specific requests on EP0 (RS3 uses these; do NOT stall)
#define PTP_REQ_CANCEL             0x64
#define PTP_REQ_GET_EXT_EVENT_DATA 0x65
#define PTP_REQ_RESET              0x66
#define PTP_REQ_GET_DEVICE_STATUS  0x67

static uint8_t s_rx_buf[64];
// Must fit: header (9..12 bytes depending on RS3 layout) + payload (DeviceInfo can be ~250 bytes).
static uint8_t s_tx_buf[512];
static uint8_t s_ctrl_buf[64];

// Pending TX state for multi-stage operations (DATA -> optional ZLP).
// NOTE: For streamed DATA we use s_tx_stream_send_ok_after to emit RESP OK; do NOT keep a second "pending OK" flag,
// otherwise RS3 can receive unexpected extra responses and reset the session.
static bool s_pending_zlp = false;
static uint32_t s_session_id = 0;

// -----------------------------
// RS3 full emulation state
// -----------------------------

// Large DATA containers (e.g. op=0x9209) must be streamed in multiple USB IN transfers.
static bool s_tx_stream_active = false;
static uint16_t s_tx_stream_code = 0;
static uint32_t s_tx_stream_tid = 0;
static const uint8_t *s_tx_stream_payload = NULL; // payload bytes only (after 12-byte std header)
static size_t s_tx_stream_payload_len = 0;
static size_t s_tx_stream_payload_off = 0;
static bool s_tx_stream_need_zlp = false;
static bool s_tx_stream_zlp_sent = false;
static bool s_tx_stream_send_ok_after = false;

// Some vendor operations are 2-stage (COMMAND then host->device DATA).
static bool s_waiting_data = false;
static uint16_t s_waiting_data_code = 0;
static uint32_t s_waiting_data_tid = 0;
static uint32_t s_waiting_data_p0 = 0;

static bool s_recording = false;

static uint8_t s_itf_num = 0;
static bool s_mounted = false;
typedef enum {
    RS3_PTP_LAYOUT_STD_LEN = 0,      // len32,type16,code16,tid32
    RS3_PTP_LAYOUT_ALT_LEN = 1,      // len32,code16,tid32,type16
    RS3_PTP_LAYOUT_DJI_PAD16_NOLEN = 2, // 0x0000 + type16,code16,tid32,(params...)
    RS3_PTP_LAYOUT_DJI_PAD8_NOLEN = 3,  // 0x00 + type16,code16,tid32,(params...)
    // 0x00 0x00 0x00 + type16@3, code16@5, tid32@7, (params...)@11
    // Note: RS3 often appends an extra 0x01 byte after tid; treat it as part of params/padding.
    RS3_PTP_LAYOUT_DJI_PAD24_NOLEN = 4,
} rs3_ptp_layout_t;

static rs3_ptp_layout_t s_ptp_layout = RS3_PTP_LAYOUT_STD_LEN;
static rs3_ptp_layout_t s_last_rx_layout = RS3_PTP_LAYOUT_STD_LEN;

// -----------------------------
// Debug helpers
// -----------------------------

static esp_timer_handle_t s_ep_probe_timer = NULL;
static uint8_t s_ep_probe_rhport = 0;

static void ep_probe_timer_cb(void *arg)
{
    (void)arg;
    const uint8_t rhport = s_ep_probe_rhport;
    const bool in_busy = usbd_edpt_busy(rhport, EP_BULK_IN);
    const bool out_busy = usbd_edpt_busy(rhport, EP_BULK_OUT);
    const bool in_stall = usbd_edpt_stalled(rhport, EP_BULK_IN);
    const bool out_stall = usbd_edpt_stalled(rhport, EP_BULK_OUT);
}

static void ep_probe_schedule(uint8_t rhport, uint32_t delay_us)
{
    s_ep_probe_rhport = rhport;
    if (s_ep_probe_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = &ep_probe_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ptp_ep_probe",
            .skip_unhandled_events = true,
        };
        (void)esp_timer_create(&args, &s_ep_probe_timer);
    }
    if (s_ep_probe_timer) {
        (void)esp_timer_stop(s_ep_probe_timer);
        (void)esp_timer_start_once(s_ep_probe_timer, delay_us);
    }
}

// -----------------------------
// Captured response payloads (Sony ILCE-5100)
// -----------------------------
// These are raw container payload bytes (i.e. bytes after the standard 12-byte PTP header).
// They were extracted from `full-start-stop.log` during RS3 start/stop recording tracing.

static const uint8_t k_devinfo_payload_1001[247] = {
  0x64, 0x00, 0x11, 0x00, 0x00, 0x00, 0x64, 0x00, 0x14, 0x53, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x79,
  0x00, 0x20, 0x00, 0x50, 0x00, 0x54, 0x00, 0x50, 0x00, 0x20, 0x00, 0x45, 0x00, 0x78, 0x00, 0x74,
  0x00, 0x65, 0x00, 0x6E, 0x00, 0x73, 0x00, 0x69, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x73, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x10, 0x03, 0x10, 0x01, 0x10, 0x04, 0x10, 0x05,
  0x10, 0x06, 0x10, 0x07, 0x10, 0x08, 0x10, 0x09, 0x10, 0x0A, 0x10, 0x1B, 0x10, 0x01, 0x92, 0x02,
  0x92, 0x05, 0x92, 0x07, 0x92, 0x09, 0x92, 0x03, 0x00, 0x00, 0x00, 0x01, 0xC2, 0x02, 0xC2, 0x03,
  0xC2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x38, 0x01,
  0xB3, 0x01, 0xB1, 0x11, 0x53, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x79, 0x00, 0x20, 0x00, 0x43, 0x00,
  0x6F, 0x00, 0x72, 0x00, 0x70, 0x00, 0x6F, 0x00, 0x72, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00,
  0x6F, 0x00, 0x6E, 0x00, 0x00, 0x00, 0x0A, 0x49, 0x00, 0x4C, 0x00, 0x43, 0x00, 0x45, 0x00, 0x2D,
  0x00, 0x35, 0x00, 0x31, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x04, 0x33, 0x00, 0x2E, 0x00,
  0x30, 0x00, 0x00, 0x00, 0x21, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30,
  0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30,
  0x00, 0x30, 0x00, 0x30, 0x00, 0x33, 0x00, 0x32, 0x00, 0x38, 0x00, 0x32, 0x00, 0x37, 0x00, 0x36,
  0x00, 0x33, 0x00, 0x30, 0x00, 0x30, 0x00, 0x33, 0x00, 0x38, 0x00, 0x35, 0x00, 0x39, 0x00, 0x30,
  0x00, 0x38, 0x00, 0x37, 0x00, 0x00, 0x00,
};

static const uint8_t k_storageids_payload_1004[8] = {
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
};

static const uint8_t k_vendor_9201_payload[8] = { 0 };

static const uint8_t k_vendor_9202_payload[84] = {
  0xC8, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x04, 0x50, 0x05, 0x50, 0x07, 0x50, 0x0A, 0x50, 0x0B, 0x50,
  0x0C, 0x50, 0x0E, 0x50, 0x10, 0x50, 0x13, 0x50, 0x00, 0xD2, 0x01, 0xD2, 0x03, 0xD2, 0x0D, 0xD2,
  0x0E, 0xD2, 0x0F, 0xD2, 0x10, 0xD2, 0x1C, 0xD2, 0x11, 0xD2, 0x13, 0xD2, 0x1E, 0xD2, 0x1B, 0xD2,
  0x1D, 0xD2, 0x1F, 0xD2, 0x17, 0xD2, 0x18, 0xD2, 0x19, 0xD2, 0x12, 0xD2, 0x21, 0xD2, 0x14, 0xD2,
  0x15, 0xD2, 0x20, 0xD2, 0x06, 0x00, 0x00, 0x00, 0xC1, 0xD2, 0xC2, 0xD2, 0xC3, 0xD2, 0xC8, 0xD2,
  0xC5, 0xD2, 0xC7, 0xD2,
};

// op=0x9209 payload (response container total len is 1023 bytes incl header)
static const uint8_t k_vendor_9209_payload[1011] = {
  0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x50, 0x02, 0x00, 0x01, 0x00, 0x02, 0x03,
  0x02, 0x07, 0x00, 0x01, 0x02, 0x03, 0x10, 0x13, 0x20, 0x23, 0x05, 0x50, 0x04, 0x00, 0x01, 0x01,
  0x02, 0x00, 0x02, 0x00, 0x02, 0x0C, 0x00, 0x02, 0x00, 0x04, 0x00, 0x11, 0x80, 0x10, 0x80, 0x06,
  0x00, 0x01, 0x80, 0x02, 0x80, 0x03, 0x80, 0x04, 0x80, 0x30, 0x80, 0x12, 0x80, 0x23, 0x80, 0x07,
  0x50, 0x04, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xC8, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0x00,
  0x0A, 0x50, 0x04, 0x00, 0x00, 0x02, 0x01, 0x00, 0x04, 0x80, 0x02, 0x07, 0x00, 0x01, 0x00, 0x02,
  0x00, 0x03, 0x00, 0x04, 0x80, 0x05, 0x80, 0x06, 0x80, 0x07, 0x80, 0x0B, 0x50, 0x04, 0x00, 0x00,
  0x02, 0x01, 0x00, 0x01, 0x00, 0x02, 0x03, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x80, 0x0C, 0x50,
  0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x09, 0x00, 0x02, 0x00, 0x01, 0x00, 0x04,
  0x00, 0x03, 0x00, 0x05, 0x00, 0x01, 0x80, 0x03, 0x80, 0x31, 0x80, 0x32, 0x80, 0x0E, 0x50, 0x04,
  0x00, 0x00, 0x02, 0x01, 0x00, 0x51, 0x80, 0x02, 0x15, 0x00, 0x00, 0x80, 0x01, 0x80, 0x02, 0x00,
  0x03, 0x00, 0x04, 0x00, 0x01, 0x00, 0x50, 0x80, 0x51, 0x80, 0x52, 0x80, 0x53, 0x80, 0x54, 0x80,
  0x41, 0x80, 0x07, 0x00, 0x11, 0x80, 0x15, 0x80, 0x14, 0x80, 0x12, 0x80, 0x13, 0x80, 0x16, 0x80,
  0x17, 0x80, 0x18, 0x80, 0x10, 0x50, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0xD4, 0xFE, 0x02, 0x2B,
  0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x88, 0x13, 0x5C, 0x12, 0x94, 0x11, 0xCC, 0x10, 0xA0,
  0x0F, 0x74, 0x0E, 0xAC, 0x0D, 0xE4, 0x0C, 0xB8, 0x0B, 0x8C, 0x0A, 0xC4, 0x09, 0xFC, 0x08, 0xD0,
  0x07, 0xA4, 0x06, 0xDC, 0x05, 0x14, 0x05, 0xE8, 0x03, 0xBC, 0x02, 0xF4, 0x01, 0x2C, 0x01, 0xD4,
  0xFE, 0x0C, 0xFE, 0x44, 0xFD, 0x18, 0xFC, 0xEC, 0xFA, 0x24, 0xFA, 0x5C, 0xF9, 0x30, 0xF8, 0x04,
  0xF7, 0x3C, 0xF6, 0x74, 0xF5, 0x48, 0xF4, 0x1C, 0xF3, 0x54, 0xF2, 0x8C, 0xF1, 0x60, 0xF0, 0x34,
  0xEF, 0x6C, 0xEE, 0xA4, 0xED, 0x78, 0xEC, 0x13, 0x50, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
  0x00, 0x02, 0x1D, 0x00, 0x01, 0x00, 0x02, 0x00, 0x12, 0x80, 0x05, 0x80, 0x04, 0x80, 0x08, 0x80,
  0x09, 0x80, 0x37, 0x83, 0x37, 0x85, 0x57, 0x83, 0x57, 0x85, 0x77, 0x83, 0x77, 0x85, 0x11, 0x83,
  0x21, 0x83, 0x31, 0x83, 0x36, 0x83, 0x36, 0x85, 0x56, 0x83, 0x56, 0x85, 0x76, 0x83, 0x76, 0x85,
  0x10, 0x83, 0x20, 0x83, 0x30, 0x83, 0x18, 0x80, 0x28, 0x80, 0x19, 0x80, 0x29, 0x80, 0x00, 0xD2,
  0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02,
  0x00, 0xB8, 0x0B, 0x8C, 0x0A, 0xC4, 0x09, 0xFC, 0x08, 0xD0, 0x07, 0xA4, 0x06, 0xDC, 0x05, 0x14,
  0x05, 0xE8, 0x03, 0xBC, 0x02, 0xF4, 0x01, 0x2C, 0x01, 0xD4, 0xFE, 0x0C, 0xFE, 0x44, 0xFD, 0x18,
  0xFC, 0xEC, 0xFA, 0x24, 0xFA, 0x5C, 0xF9, 0x30, 0xF8, 0x04, 0xF7, 0x3C, 0xF6, 0x74, 0xF5, 0x48,
  0xF4, 0x01, 0xD2, 0x02, 0x00, 0x01, 0x01, 0x01, 0x01, 0x02, 0x07, 0x00, 0x01, 0x1F, 0x11, 0x12,
  0x13, 0x14, 0x15, 0x03, 0xD2, 0x02, 0x00, 0x01, 0x01, 0x04, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02,
  0x03, 0x0D, 0xD2, 0x06, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x64, 0x00, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x0E, 0xD2, 0x02, 0x00,
  0x00, 0x02, 0x01, 0x05, 0x02, 0x0A, 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0A, 0x04, 0x05, 0x06,
  0x07, 0x0F, 0xD2, 0x04, 0x00, 0x01, 0x00, 0x7C, 0x15, 0x00, 0x00, 0x01, 0xC4, 0x09, 0xAC, 0x26,
  0x64, 0x00, 0x10, 0xD2, 0x02, 0x00, 0x01, 0x01, 0x80, 0x80, 0x01, 0x79, 0x87, 0x01, 0x1C, 0xD2,
  0x02, 0x00, 0x01, 0x01, 0x80, 0x80, 0x01, 0x79, 0x87, 0x01, 0x11, 0xD2, 0x02, 0x00, 0x01, 0x01,
  0x01, 0x02, 0x02, 0x02, 0x00, 0x01, 0x02, 0x13, 0xD2, 0x02, 0x00, 0x00, 0x02, 0x01, 0x01, 0x02,
  0x06, 0x00, 0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x1E, 0xD2, 0x06, 0x00, 0x00, 0x01, 0xFF, 0xFF,
  0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0x1D, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x19, 0x00, 0x00,
  0x00, 0x64, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00,
  0x00, 0xFA, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0xF4, 0x01, 0x00,
  0x00, 0x80, 0x02, 0x00, 0x00, 0x20, 0x03, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0xE2, 0x04, 0x00,
  0x00, 0x40, 0x06, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0xC4, 0x09, 0x00, 0x00, 0x80, 0x0C, 0x00,
  0x00, 0xA0, 0x0F, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x40, 0x1F, 0x00,
  0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x20, 0x4E, 0x00,
  0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x10, 0x27, 0x00, 0x01, 0x1B, 0xD2, 0x04,
  0x00, 0x01, 0x01, 0x00, 0x80, 0x00, 0x80, 0x02, 0x10, 0x00, 0x00, 0x80, 0x01, 0x80, 0x02, 0x80,
  0x03, 0x80, 0x04, 0x80, 0x05, 0x80, 0x10, 0x80, 0x20, 0x80, 0x21, 0x80, 0x30, 0x80, 0x40, 0x80,
  0x50, 0x80, 0x51, 0x80, 0x52, 0x80, 0x53, 0x80, 0x60, 0x80, 0x1D, 0xD2, 0x02, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x01, 0x00, 0x02, 0x01, 0x1F, 0xD2, 0x02, 0x00, 0x00, 0x02, 0x01, 0x00, 0x02, 0x00,
  0x00, 0x17, 0xD2, 0x02, 0x00, 0x00, 0x02, 0x01, 0x01, 0x02, 0x02, 0x00, 0x02, 0x01, 0x18, 0xD2,
  0x01, 0x00, 0x00, 0x02, 0xFF, 0x31, 0x01, 0xFF, 0x64, 0x01, 0x19, 0xD2, 0x02, 0x00, 0x00, 0x02,
  0x01, 0x02, 0x02, 0x02, 0x00, 0x02, 0x01, 0xC1, 0xD2, 0x04, 0x00, 0x81, 0x01, 0x01, 0x00, 0x01,
  0x00, 0x02, 0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0xC2, 0xD2, 0x04, 0x00, 0x81, 0x01, 0x01, 0x00,
  0x01, 0x00, 0x02, 0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0xC3, 0xD2, 0x04, 0x00, 0x81, 0x01, 0x01,
  0x00, 0x01, 0x00, 0x02, 0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0xC8, 0xD2, 0x04, 0x00, 0x81, 0x01,
  0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0x12, 0xD2, 0x02, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x01, 0x00, 0x0F, 0x01, 0x21, 0xD2, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02,
  0x03, 0x00, 0x00, 0x01, 0x02, 0x14, 0xD2, 0x06, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00,
  0x15, 0xD2, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0x01,
  0x00, 0xC5, 0xD2, 0x04, 0x00, 0x83, 0x01, 0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x00, 0x01, 0x00,
  0x02, 0x00, 0xC7, 0xD2, 0x04, 0x00, 0x81, 0x01, 0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x00, 0x01,
  0x00, 0x02, 0x00,
};

// -----------------------------
// USB descriptors
// -----------------------------

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_ITF,
};

static char const *s_str_desc[] = {
    (const char[]) { 0x09, 0x04 }, // English (0x0409)
    CONFIG_RS3_USB_PTP_MANUFACTURER,
    CONFIG_RS3_USB_PTP_PRODUCT,
    CONFIG_RS3_USB_PTP_SERIAL,
    "PTP",
};

static tusb_desc_device_t const s_dev_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = CONFIG_RS3_USB_PTP_VID,
    .idProduct          = CONFIG_RS3_USB_PTP_PID,
    .bcdDevice          = CONFIG_RS3_USB_PTP_BCD_DEVICE,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 0x01
};

// Still Image (PTP) interface descriptor:
// Interface class: 0x06 (Still Imaging), subclass 0x01, protocol 0x01 (PTP)
// Sony ILCE-5100 reports only 2 endpoints (bulk IN/OUT).
#define PTP_ITF_CLASS   0x06
#define PTP_ITF_SUBCLASS 0x01
#define PTP_ITF_PROTOCOL 0x01

#define CFG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7 + 7)

static uint8_t const s_fs_cfg_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CFG_TOTAL_LEN, 0x80, 100),

    // Interface descriptor
    9, TUSB_DESC_INTERFACE,
    0x00, 0x00, // itf num, alt
    0x02,       // num endpoints
    PTP_ITF_CLASS, PTP_ITF_SUBCLASS, PTP_ITF_PROTOCOL,
    STRID_ITF,

    // Endpoint Bulk OUT (commands/data)
    7, TUSB_DESC_ENDPOINT,
    EP_BULK_OUT, TUSB_XFER_BULK, 0x40, 0x00,
    0x00,

    // Endpoint Bulk IN (responses/data)
    7, TUSB_DESC_ENDPOINT,
    EP_BULK_IN, TUSB_XFER_BULK, 0x40, 0x00,
    0x00,
};

// -----------------------------
// PTP DeviceInfo builder (minimal)
// -----------------------------

// (legacy helper removed: used an alternate uint16-based string encoding, but we now emit byte-level PTP strings)

static size_t build_device_info(uint8_t *out, size_t cap)
{
    // DEBUG: byte-for-byte reference DeviceInfo dataset captured from a real Sony camera.
    // This intentionally ignores CONFIG_RS3_USB_PTP_* strings to isolate content/format issues.
    static const uint8_t k_device_info_etalon[] = {
        0x64,0x00,0x11,0x00,0x00,0x00,0x64,0x00,0x14,0x53,0x00,0x6f,0x00,0x6e,0x00,0x79,
        0x00,0x20,0x00,0x50,0x00,0x54,0x00,0x50,0x00,0x20,0x00,0x45,0x00,0x78,0x00,0x74,
        0x00,0x65,0x00,0x6e,0x00,0x73,0x00,0x69,0x00,0x6f,0x00,0x6e,0x00,0x73,0x00,0x00,
        0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x02,0x10,0x03,0x10,0x01,0x10,0x04,0x10,0x05,
        0x10,0x06,0x10,0x07,0x10,0x08,0x10,0x09,0x10,0x0a,0x10,0x1b,0x10,0x01,0x92,0x02,
        0x92,0x05,0x92,0x07,0x92,0x09,0x92,0x03,0x00,0x00,0x00,0x01,0xc2,0x02,0xc2,0x03,
        0xc2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x01,0x38,0x01,
        0xb3,0x01,0xb1,0x11,0x53,0x00,0x6f,0x00,0x6e,0x00,0x79,0x00,0x20,0x00,0x43,0x00,
        0x6f,0x00,0x72,0x00,0x70,0x00,0x6f,0x00,0x72,0x00,0x61,0x00,0x74,0x00,0x69,0x00,
        0x6f,0x00,0x6e,0x00,0x00,0x00,0x0a,0x49,0x00,0x4c,0x00,0x43,0x00,0x45,0x00,0x2d,
        0x00,0x35,0x00,0x31,0x00,0x30,0x00,0x30,0x00,0x00,0x00,0x04,0x33,0x00,0x2e,0x00,
        0x30,0x00,0x00,0x00,0x21,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,
        0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,
        0x00,0x30,0x00,0x33,0x00,0x32,0x00,0x38,0x00,0x32,0x00,0x37,0x00,0x36,0x00,0x33,
        0x00,0x30,0x00,0x30,0x00,0x33,0x00,0x38,0x00,0x35,0x00,0x39,0x00,0x30,0x00,0x38,
        0x00,0x37,0x00,0x00,0x00
    };

    if (cap < sizeof(k_device_info_etalon)) return 0;
    memcpy(out, k_device_info_etalon, sizeof(k_device_info_etalon));
    return sizeof(k_device_info_etalon);
}

static inline size_t ptp_hdr_bytes_for_layout(rs3_ptp_layout_t layout)
{
    switch (layout) {
        case RS3_PTP_LAYOUT_DJI_PAD24_NOLEN: return 11;
        case RS3_PTP_LAYOUT_DJI_PAD16_NOLEN: return 10;
        case RS3_PTP_LAYOUT_DJI_PAD8_NOLEN:  return 9;
        case RS3_PTP_LAYOUT_ALT_LEN:         return 12;
        case RS3_PTP_LAYOUT_STD_LEN:         return 12;
        default:                             return 12;
    }
}

static void ptp_write_string_bytes(uint8_t **pp, const char *s)
{
    uint8_t *p = *pp;
    uint8_t n = 0;
    if (s) {
        size_t len = strlen(s);
        if (len > 254) len = 254; // keep in uint8
        n = (uint8_t)(len + 1);
        *p++ = n;
        for (uint8_t i = 0; i < n - 1; i++) { *p++ = (uint8_t)s[i]; *p++ = 0; }
        *p++ = 0; *p++ = 0;
    } else {
        *p++ = 0;
    }
    *pp = p;
}

static size_t build_storage_ids(uint8_t *out, size_t cap)
{
    if (cap < 8) return 0;
    uint8_t *p = out;
    // array of uint32: count then values
    p[0]=1; p[1]=0; p[2]=0; p[3]=0; // count=1
    // storage id 0x00010001
    p[4]=0x01; p[5]=0x00; p[6]=0x01; p[7]=0x00;
    return 8;
}

static size_t build_storage_info(uint8_t *out, size_t cap)
{
    // PTP StorageInfo dataset:
    // u16 StorageType, u16 FilesystemType, u16 AccessCapability,
    // u64 MaxCapacity, u64 FreeSpaceInBytes, u32 FreeSpaceInImages,
    // string StorageDescription, string VolumeLabel
    uint8_t *p = out;
    if (cap < 64) return 0;

    #define W16B(v) do { uint16_t _v=(uint16_t)(v); *p++=(uint8_t)(_v&0xFF); *p++=(uint8_t)(_v>>8); } while(0)
    #define W32B(v) do { uint32_t _v=(uint32_t)(v); *p++=(uint8_t)(_v&0xFF); *p++=(uint8_t)((_v>>8)&0xFF); *p++=(uint8_t)((_v>>16)&0xFF); *p++=(uint8_t)((_v>>24)&0xFF); } while(0)
    #define W64B(v) do { uint64_t _v=(uint64_t)(v); \
        *p++=(uint8_t)(_v&0xFF); *p++=(uint8_t)((_v>>8)&0xFF); *p++=(uint8_t)((_v>>16)&0xFF); *p++=(uint8_t)((_v>>24)&0xFF); \
        *p++=(uint8_t)((_v>>32)&0xFF); *p++=(uint8_t)((_v>>40)&0xFF); *p++=(uint8_t)((_v>>48)&0xFF); *p++=(uint8_t)((_v>>56)&0xFF); } while(0)

    W16B(0x0002); // StorageType: Fixed RAM
    W16B(0x0002); // FilesystemType: Generic hierarchical
    W16B(0x0000); // AccessCapability: ReadWrite
    W64B(32ULL * 1024ULL * 1024ULL * 1024ULL);  // 32 GiB
    W64B(31ULL * 1024ULL * 1024ULL * 1024ULL);  // 31 GiB free
    W32B(0xFFFFFFFF); // FreeSpaceInImages: unknown

    ptp_write_string_bytes(&p, "Internal Storage");
    ptp_write_string_bytes(&p, "SONY");

    #undef W16B
    #undef W32B
    #undef W64B
    return (size_t)(p - out);
}

// -----------------------------
// Custom TinyUSB class driver
// -----------------------------

static void ptp_init(void) {}
static bool ptp_deinit(void) { return true; }
static void ptp_reset(uint8_t rhport)
{
    (void)rhport;
    s_mounted = false;
    // Best-effort: stop probe timer so we don't log stale state after reset.
    if (s_ep_probe_timer) (void)esp_timer_stop(s_ep_probe_timer);
}

static uint16_t ptp_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    (void)max_len;
    if (itf_desc->bInterfaceClass != PTP_ITF_CLASS ||
        itf_desc->bInterfaceSubClass != PTP_ITF_SUBCLASS ||
        itf_desc->bInterfaceProtocol != PTP_ITF_PROTOCOL) {
        return 0;
    }

    s_itf_num = itf_desc->bInterfaceNumber;

    // Endpoints are the descriptors following the interface descriptor
    uint16_t len = itf_desc->bLength;
    uint8_t const *p = (uint8_t const *)itf_desc + itf_desc->bLength;
    for (int i = 0; i < itf_desc->bNumEndpoints; i++) {
        tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
        if (ep->bDescriptorType == TUSB_DESC_ENDPOINT) {
            usbd_edpt_open(rhport, ep);
        }
        len += ep->bLength;
        p += ep->bLength;
    }

    // Start first OUT transfer
    usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
    s_mounted = true;
    return len;
}

static bool ptp_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    // Some hosts (RS3) send standard ENDPOINT requests (e.g. CLEAR_FEATURE HALT) and expect them to succeed.
    // Handle the subset we observe to avoid triggering device reset / re-enumeration.
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {
        // CLEAR_FEATURE(ENDPOINT_HALT)
        if (request->bRequest == TUSB_REQ_CLEAR_FEATURE && tu_le16toh(request->wValue) == 0) {
            uint8_t ep = (uint8_t)tu_le16toh(request->wIndex);
            if (stage == CONTROL_STAGE_SETUP) {
                if (usbd_edpt_stalled(rhport, ep)) usbd_edpt_clear_stall(rhport, ep);
            }
            return tud_control_status(rhport, request);
        }
        // Default: ACK standard endpoint requests to stay robust.
        return tud_control_status(rhport, request);
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;
    if (request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE) return false;
    if ((uint8_t)tu_le16toh(request->wIndex) != s_itf_num) return false;

    switch (request->bRequest) {
        case PTP_REQ_GET_DEVICE_STATUS: {
            // DeviceStatus: uint16 length (=4), uint16 status (PTP response code)
            if (stage == CONTROL_STAGE_SETUP) {
                s_ctrl_buf[0] = 0x04; s_ctrl_buf[1] = 0x00; // length=4
                s_ctrl_buf[2] = (uint8_t)(PTP_RC_OK & 0xFF);
                s_ctrl_buf[3] = (uint8_t)(PTP_RC_OK >> 8);
                return tud_control_xfer(rhport, request, s_ctrl_buf, 4);
            }
            return true;
        }

        case PTP_REQ_CANCEL: {
            // Host sends a small structure (code + transaction_id). Accept and ignore.
            if (stage == CONTROL_STAGE_SETUP) {
                uint16_t wlen = tu_le16toh(request->wLength);
                if (wlen > sizeof(s_ctrl_buf)) wlen = sizeof(s_ctrl_buf);
                return tud_control_xfer(rhport, request, s_ctrl_buf, wlen);
            }
            return true;
        }

        case PTP_REQ_RESET: {
            // Clear stalls and reset state.
            if (stage == CONTROL_STAGE_SETUP) {
                if (usbd_edpt_stalled(rhport, EP_BULK_OUT)) usbd_edpt_clear_stall(rhport, EP_BULK_OUT);
                if (usbd_edpt_stalled(rhport, EP_BULK_IN))  usbd_edpt_clear_stall(rhport, EP_BULK_IN);
                s_waiting_data = false;
                s_waiting_data_code = 0;
                s_waiting_data_tid = 0;
                s_session_id = 0;
                s_tx_stream_active = false;
                s_tx_stream_send_ok_after = false;
                s_tx_stream_payload = NULL;
                s_tx_stream_payload_len = 0;
                s_tx_stream_payload_off = 0;
                s_tx_stream_need_zlp = false;
                s_tx_stream_zlp_sent = false;
                // PTP RESET has no data stage: must ACK the control transfer.
                return tud_control_status(rhport, request);
            }
            return true;
        }

        case PTP_REQ_GET_EXT_EVENT_DATA: {
            // Return zeros (some hosts query it; don't stall).
            if (stage == CONTROL_STAGE_SETUP) {
                uint16_t wlen = tu_le16toh(request->wLength);
                if (wlen > sizeof(s_ctrl_buf)) wlen = sizeof(s_ctrl_buf);
                memset(s_ctrl_buf, 0, wlen);
                return tud_control_xfer(rhport, request, s_ctrl_buf, wlen);
            }
            return true;
        }

        default:
            // Unknown still-image class request: acknowledge with status to avoid host rejecting the device.
            if (stage == CONTROL_STAGE_SETUP) {
                return tud_control_status(rhport, request);
            }
            return true;
    }
}


static inline void wr_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void wr_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// (proxy helpers removed from this module)

static const char *layout_name(rs3_ptp_layout_t l)
{
    switch (l) {
        case RS3_PTP_LAYOUT_STD_LEN: return "std_len";
        case RS3_PTP_LAYOUT_ALT_LEN: return "alt_len";
        case RS3_PTP_LAYOUT_DJI_PAD16_NOLEN: return "dji_pad16";
        case RS3_PTP_LAYOUT_DJI_PAD8_NOLEN: return "dji_pad8";
        case RS3_PTP_LAYOUT_DJI_PAD24_NOLEN: return "dji_pad24";
        default: return "unknown";
    }
}

typedef struct {
    rs3_ptp_layout_t layout;
    uint16_t type;
    uint16_t code;
    uint32_t tid;
    uint32_t params[5];
    int param_count;
    size_t header_bytes; // bytes before params/payload
} rs3_ptp_cmd_parsed_t;

static bool parse_rs3_ptp_cmd(const uint8_t *buf, size_t n, rs3_ptp_cmd_parsed_t *out)
{
    if (!out || !buf || n < 8) return false;
    memset(out, 0, sizeof(*out));

    // Heuristic 0: DJI "no-len" with pad24 (3x 0x00):
    // 00 00 00 [type16le@3] [code16le@5] [tid32le@7] [params...@11]
    // Seen as 16-byte packets like:
    // 00 00 00 01 00 02 10 00 00 00 00 01 00 00 00 00  => type=1 op=0x1002 tid=0 p0=1
    if (n >= 11 && buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x00) {
        uint16_t type16 = rd_le16(buf + 3);
        uint16_t code16 = rd_le16(buf + 5);
        uint32_t tid32 = rd_le32(buf + 7);
        if (type16 >= 1 && type16 <= 4) {
            out->layout = RS3_PTP_LAYOUT_DJI_PAD24_NOLEN;
            out->type = type16;
            out->code = code16;
            out->tid = tid32;
            out->header_bytes = 11;
            goto decode_params;
        }
    }

    // Heuristic 1: DJI "no-len" with pad16: 00 00 [type16] [code16] [tid32] [params...]
    if (n >= 10) {
        uint16_t pad16 = rd_le16(buf + 0);
        uint16_t type16 = rd_le16(buf + 2);
        uint16_t code16 = rd_le16(buf + 4);
        uint32_t tid32 = rd_le32(buf + 6);
        if (pad16 == 0x0000 && type16 >= 1 && type16 <= 4) {
            out->layout = RS3_PTP_LAYOUT_DJI_PAD16_NOLEN;
            out->type = type16;
            out->code = code16;
            out->tid = tid32;
            out->header_bytes = 10;
            goto decode_params;
        }
    }

    // Heuristic 2: DJI "no-len" with pad8: 00 [type16] [code16] [tid32] [params...]
    if (n >= 9) {
        uint8_t pad8 = buf[0];
        uint16_t type16 = rd_le16(buf + 1);
        uint16_t code16 = rd_le16(buf + 3);
        uint32_t tid32 = rd_le32(buf + 5);
        if (pad8 == 0x00 && type16 >= 1 && type16 <= 4) {
            out->layout = RS3_PTP_LAYOUT_DJI_PAD8_NOLEN;
            out->type = type16;
            out->code = code16;
            out->tid = tid32;
            out->header_bytes = 9;
            goto decode_params;
        }
    }

    // Standard PTP/MTP: len32,type16,code16,tid32
    if (n >= 12) {
        uint16_t type_std = rd_le16(buf + 4);
        if (type_std >= 1 && type_std <= 4) {
            out->layout = RS3_PTP_LAYOUT_STD_LEN;
            out->type = type_std;
            out->code = rd_le16(buf + 6);
            out->tid = rd_le32(buf + 8);
            out->header_bytes = 12;
            goto decode_params;
        }
        // Alt DJI observed: len32,code16,tid32,type16
        uint16_t type_alt = rd_le16(buf + 10);
        if (type_alt >= 1 && type_alt <= 4) {
            out->layout = RS3_PTP_LAYOUT_ALT_LEN;
            out->type = type_alt;
            out->code = rd_le16(buf + 4);
            out->tid = rd_le32(buf + 6);
            out->header_bytes = 12;
            goto decode_params;
        }
    }

    return false;

decode_params:
    // Decode params from actual received bytes after header (4-byte each), up to 5
    out->param_count = 0;
    if (n > out->header_bytes) {
        size_t avail = n - out->header_bytes;
        size_t want = avail / 4;
        if (want > 5) want = 5;
        out->param_count = (int)want;
        for (int i = 0; i < out->param_count; i++) {
            size_t off = out->header_bytes + (size_t)i * 4;
            out->params[i] = rd_le32(buf + off);
        }
    }
    return true;
}

// Write header matching the detected RS3 layout (TX follows the RX layout for now).
// STD_LEN:  len32, type16, code16, tid32
// ALT_LEN:  len32, code16, tid32, type16
// DJI_PAD24_NOLEN: 00 00 00 + type16, code16, tid32
// DJI_PAD16_NOLEN: 00 00 + type16, code16, tid32
// DJI_PAD8_NOLEN:  00 + type16, code16, tid32
static size_t write_ptp_hdr(uint8_t *dst, uint32_t len, uint16_t type, uint16_t code, uint32_t tid)
{
    switch (s_ptp_layout) {
        case RS3_PTP_LAYOUT_DJI_PAD24_NOLEN:
            dst[0] = 0x00;
            dst[1] = 0x00;
            dst[2] = 0x00;
            wr_le16(dst + 3, type);
            wr_le16(dst + 5, code);
            wr_le32(dst + 7, tid);
            return 11;
        case RS3_PTP_LAYOUT_DJI_PAD16_NOLEN:
            wr_le16(dst + 0, 0x0000);
            wr_le16(dst + 2, type);
            wr_le16(dst + 4, code);
            wr_le32(dst + 6, tid);
            return 10;
        case RS3_PTP_LAYOUT_DJI_PAD8_NOLEN:
            dst[0] = 0x00;
            wr_le16(dst + 1, type);
            wr_le16(dst + 3, code);
            wr_le32(dst + 5, tid);
            return 9;
        case RS3_PTP_LAYOUT_ALT_LEN:
            wr_le32(dst + 0, len);
            wr_le16(dst + 4, code);
            wr_le32(dst + 6, tid);
            wr_le16(dst + 10, type);
            return 12;
        case RS3_PTP_LAYOUT_STD_LEN:
        default:
            wr_le32(dst + 0, len);
            wr_le16(dst + 4, type);
            wr_le16(dst + 6, code);
            wr_le32(dst + 8, tid);
            return 12;
    }
}

static void ui_ptp_linef(const char *fmt, ...);
static uint16_t s_ui_last_op = 0;

static void ui_ptp_progress_rx(uint16_t op_code)
{
    s_ui_last_op = op_code;
    switch (op_code) {
        case PTP_OC_OPEN_SESSION:        ui_ptp_linef("open sess"); break;
        case PTP_OC_GET_DEVICE_INFO:     ui_ptp_linef("get info"); break;
        case PTP_OC_GET_STORAGE_IDS:     ui_ptp_linef("stor ids"); break;
        case PTP_OC_GET_STORAGE_INFO:    ui_ptp_linef("stor info"); break;
        case PTP_OC_GET_NUM_OBJECTS:     ui_ptp_linef("num objs"); break;
        case PTP_OC_GET_OBJECT_HANDLES:  ui_ptp_linef("obj hndl"); break;
        case PTP_OC_CLOSE_SESSION:       ui_ptp_linef("close"); break;
        default:
            if (op_code < 0x1000) ui_ptp_linef("vendor");
            else ui_ptp_linef("other");
            break;
    }
}

static void ui_ptp_progress_tx_resp(uint16_t resp_code)
{
    const bool ok = (resp_code == PTP_RC_OK);
    switch (resp_code) {
        case PTP_RC_OK:
            switch (s_ui_last_op) {
                case PTP_OC_OPEN_SESSION: ui_ptp_linef("open ok"); break;
                case PTP_OC_GET_DEVICE_INFO: ui_ptp_linef("info ok"); break;
                case PTP_OC_GET_STORAGE_IDS: ui_ptp_linef("ids ok"); break;
                case PTP_OC_GET_STORAGE_INFO: ui_ptp_linef("stor ok"); break;
                case PTP_OC_GET_NUM_OBJECTS: ui_ptp_linef("num ok"); break;
                case PTP_OC_GET_OBJECT_HANDLES: ui_ptp_linef("hndl ok"); break;
                case PTP_OC_CLOSE_SESSION: ui_ptp_linef("close ok"); break;
                default: ui_ptp_linef("ok"); break;
            }
            break;
        case PTP_RC_OPERATION_NOT_SUPPORTED: ui_ptp_linef("unsup"); break;
        default: ui_ptp_linef("resp"); break;
    }
    (void)ok;
}

static void ui_ptp_progress_tx_data(uint16_t op_code)
{
    switch (op_code) {
        case PTP_OC_GET_DEVICE_INFO: ui_ptp_linef("send info"); break;
        case PTP_OC_GET_STORAGE_IDS: ui_ptp_linef("send ids"); break;
        case PTP_OC_GET_STORAGE_INFO: ui_ptp_linef("send stor"); break;
        case PTP_OC_GET_NUM_OBJECTS: ui_ptp_linef("send num"); break;
        case PTP_OC_GET_OBJECT_HANDLES: ui_ptp_linef("send hndl"); break;
        default: ui_ptp_linef("send"); break;
    }
}

static void send_response(uint8_t rhport, uint16_t resp_code, uint32_t trans_id)
{
    // RS3 accepts standard PTP (camera-style) containers on bulk IN; prefer std_len for all responses.
    // (This matches the proxy's winning mode: --rs3-in-layout camera.)
    rs3_ptp_layout_t saved = s_ptp_layout;
    s_ptp_layout = RS3_PTP_LAYOUT_STD_LEN;
    size_t hdr_bytes = write_ptp_hdr(s_tx_buf, (uint32_t)sizeof(ptp_hdr_t), PTP_CT_RESPONSE, resp_code, trans_id);
    s_ptp_layout = saved;

    bool ok = usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, (uint16_t)hdr_bytes);
    ui_ptp_progress_tx_resp(resp_code);
}

static void tx_stream_start(uint8_t rhport, uint16_t op_code, uint32_t trans_id, const uint8_t *payload, size_t payload_len, bool send_ok_after)
{
    // Stream as standard PTP container (len32,type16,code16,tid32 + payload) regardless of RS3 RX layout.
    s_tx_stream_active = true;
    s_tx_stream_code = op_code;
    s_tx_stream_tid = trans_id;
    s_tx_stream_payload = payload;
    s_tx_stream_payload_len = payload_len;
    s_tx_stream_payload_off = 0;
    s_tx_stream_need_zlp = (((12 + payload_len) % 64) == 0);
    s_tx_stream_zlp_sent = false;
    s_tx_stream_send_ok_after = send_ok_after;

    const size_t first_payload = (payload_len > (sizeof(s_tx_buf) - 12)) ? (sizeof(s_tx_buf) - 12) : payload_len;
    const uint32_t len_field = (uint32_t)(12 + payload_len);
    (void)write_ptp_hdr(s_tx_buf, len_field, PTP_CT_DATA, op_code, trans_id);
    if (first_payload) {
        memcpy(s_tx_buf + 12, payload, first_payload);
        s_tx_stream_payload_off = first_payload;
    }

    // Log EXACT bytes we are about to send to RS3 over bulk IN (first chunk includes std header).
    bool ok = usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, (uint16_t)(12 + first_payload));
    ui_ptp_progress_tx_data(op_code);
}

static const char *ptp_op_hex_name(uint16_t op)
{
    // Safe: return pointer to a static ring buffer (avoids returning stack memory).
    // Not thread-safe, but good enough for logging in this module.
    static char s_buf[8][7]; // "0xFFFF" + NUL
    static unsigned s_idx = 0;
    unsigned idx = (s_idx++) & 7u;
    (void)snprintf(s_buf[idx], sizeof(s_buf[idx]), "0x%04X", (unsigned)op);
    return s_buf[idx];
}

static const char *ptp_op_name(uint16_t op)
{
    switch (op) {
        case PTP_OC_OPEN_SESSION:    return "OpenSession";
        case PTP_OC_GET_DEVICE_INFO: return "GetDeviceInfo";
        case PTP_OC_GET_STORAGE_IDS: return "GetStorageIDs";
        case PTP_OC_GET_STORAGE_INFO: return "GetStorageInfo";
        case PTP_OC_SONY_9201:       return "Sony 0x9201";
        case PTP_OC_SONY_9202:       return "Sony 0x9202";
        case PTP_OC_SONY_9207:       return "Sony REC";
        case PTP_OC_SONY_9209:       return "Sony 0x9209";
        default:                     return ptp_op_hex_name(op);
    }
}

static void log_cmd_banner(uint16_t op, uint32_t tid)
{
    const char *name = ptp_op_name(op);
    if (!name) return;
    (void)tid;

    if (op == PTP_OC_SONY_9209) {
        return;
    }

    rs3_tcp_logf("[PTP CMD] %s op=0x%04X tid=%" PRIu32 "\r\n",
                 name, op, tid);
}

static void ui_ptp_linef(const char *fmt, ...)
{
    char line[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    (void)rs3_ui_status_ptp_line(line);
}

static void send_data_and_ok(uint8_t rhport, uint16_t op_code, uint32_t trans_id, const uint8_t *payload, size_t payload_len)
{
    // Always stream as std_len to match RS3 expectations.
    tx_stream_start(rhport, op_code, trans_id, payload, payload_len, true);
}

static bool ptp_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    const bool is_in = (ep_addr & 0x80) != 0;
    const uint8_t ep_num = (uint8_t)(ep_addr & 0x7F);

    if (!is_in && ep_num == (EP_BULK_OUT & 0x7F)) {
        const size_t n = (size_t)xferred_bytes;
        if (n >= 8) {
            rs3_ptp_cmd_parsed_t cmd;
            if (!parse_rs3_ptp_cmd(s_rx_buf, n, &cmd)) {
                usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
                return true;
            }

            // Keep TX layout strict (std_len). RX layout is only informational/logging.
            uint16_t type = cmd.type;
            uint16_t code = cmd.code;
            uint32_t tid  = cmd.tid;
            s_last_rx_layout = cmd.layout;
            ui_ptp_progress_rx(code);

            // Handle host->device DATA stage (e.g. vendor 0x9207).
            if (type == PTP_CT_DATA) {
                const size_t payload_len = (n > cmd.header_bytes) ? (n - cmd.header_bytes) : 0;
                const uint8_t *payload = (payload_len > 0) ? (s_rx_buf + cmd.header_bytes) : NULL;

                if (s_waiting_data && code == s_waiting_data_code && tid == s_waiting_data_tid) {
                    // 0x9207: RS3 record start/stop (observed host->device DATA payload is 5 bytes).
                    // In `full-start-stop.log` payload[0] is:
                    //   - 0x02 for start
                    //   - 0x01 for stop
                    // Publish event for UI/Bluetooth.

                    rs3_tcp_logf("[PTP] 0x9207 DATA: p0=%08" PRIx32 " payload_len=%u payload=%02X\r\n",
                                 s_waiting_data_p0, (unsigned)payload_len,
                                 (unsigned)((payload_len >= 1 && payload) ? payload[0] : 0));

                    // Button press level is encoded in COMMAND param0 (p0):
                    //  - 0x0000D2C1: half-press (ignore for recording)
                    //  - 0x0000D2C8: full-press (trigger recording)
                    const bool full_press = (s_waiting_data_p0 == 0x0000D2C8);
                    if (full_press) {
                        if (payload_len >= 1 && payload) {
                            if (payload[0] == 0x02) {
                                s_recording = true;
                                rs3_rec_events_publish(RS3_REC_EVT_START, tid, payload, payload_len);
                                (void)rs3_ui_status_ptp_line("rec start");
                            } else if (payload[0] == 0x01) {
                                s_recording = false;
                                rs3_rec_events_publish(RS3_REC_EVT_STOP, tid, payload, payload_len);
                                (void)rs3_ui_status_ptp_line("rec stop");
                            } else {
                                rs3_tcp_logf("[PTP] 0x9207 DATA(full): unknown payload0=0x%02X\r\n", payload[0]);
                            }
                        } else {
                            rs3_tcp_logf("[PTP] 0x9207 DATA(full): empty payload\r\n");
                        }
                    } else {
                        // Ignore half-press (or unknown p0) for recording logic.
                        rs3_tcp_logf("[PTP] 0x9207 DATA: ignoring (not full press)\r\n");
                    }
                    s_waiting_data = false;
                    s_waiting_data_code = 0;
                    s_waiting_data_tid = 0;
                    s_waiting_data_p0 = 0;
                    send_response(rhport, PTP_RC_OK, tid);
                }
                usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
                return true;
            }

            // Only treat COMMAND containers as ops
            if (type != PTP_CT_COMMAND) {
                rs3_tcp_logf("[PTP] ignoring container type=0x%04X\r\n", type);
                usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
                return true;
            }

            log_cmd_banner(code, tid);

            uint32_t *params = cmd.params;
            int param_count = cmd.param_count;

            if (code == PTP_OC_GET_DEVICE_INFO) {
                send_data_and_ok(rhport, code, tid, k_devinfo_payload_1001, sizeof(k_devinfo_payload_1001));
            } else if (code == PTP_OC_GET_STORAGE_IDS) {
                send_data_and_ok(rhport, code, tid, k_storageids_payload_1004, sizeof(k_storageids_payload_1004));
            } else if (code == PTP_OC_GET_STORAGE_INFO) {
                // params[0] is storage id
                uint8_t data[160];
                size_t n = build_storage_info(data, sizeof(data));
                if (n > 0) {
                    send_data_and_ok(rhport, code, tid, data, n);
                } else {
                    send_response(rhport, PTP_RC_OPERATION_NOT_SUPPORTED, tid);
                }
            } else if (code == PTP_OC_GET_NUM_OBJECTS) {
                // Return 0 objects
                uint8_t data[4] = {0, 0, 0, 0};
                send_data_and_ok(rhport, code, tid, data, sizeof(data));
            } else if (code == PTP_OC_GET_OBJECT_HANDLES) {
                // Return empty handle array: count=0
                uint8_t data[4] = {0, 0, 0, 0};
                send_data_and_ok(rhport, code, tid, data, sizeof(data));
            } else if (code == PTP_OC_OPEN_SESSION) {
                // params[0] is session id
                s_session_id = (param_count >= 1) ? params[0] : 0;
                send_response(rhport, PTP_RC_OK, tid);
                // Probe endpoint state only in debug mode (touches TinyUSB internals; avoid timing side-effects).
                ep_probe_schedule(rhport, 200000);

            } else if (code == PTP_OC_CLOSE_SESSION) {
                rs3_tcp_logf("[PTP] CloseSession sid=%" PRIu32 "\r\n", s_session_id);
                s_session_id = 0;
                send_response(rhport, PTP_RC_OK, tid);
            } else {
                // Safer default: if we reply OK without sending required DATA, host can hang waiting.
                if (code == PTP_OC_SONY_9201) {
                    send_data_and_ok(rhport, code, tid, k_vendor_9201_payload, sizeof(k_vendor_9201_payload));
                } else if (code == PTP_OC_SONY_9202) {
                    send_data_and_ok(rhport, code, tid, k_vendor_9202_payload, sizeof(k_vendor_9202_payload));
                } else if (code == PTP_OC_SONY_9209) {
                    send_data_and_ok(rhport, code, tid, k_vendor_9209_payload, sizeof(k_vendor_9209_payload));
                } else if (code == PTP_OC_SONY_9207) {
                    // Start/stop record: RS3 will send a DATA stage next. Don't reply yet.
                    s_waiting_data = true;
                    s_waiting_data_code = code;
                    s_waiting_data_tid = tid;
                    s_waiting_data_p0 = (param_count >= 1 ? params[0] : 0);
                    rs3_tcp_logf("[PTP] vendor 0x9207 waiting DATA tid=%" PRIu32 " p0=%08" PRIx32 "\r\n",
                                 tid, (param_count >= 1 ? params[0] : 0));
                } else {
                    send_response(rhport, PTP_RC_OPERATION_NOT_SUPPORTED, tid);
                }
            }
        }
        // re-arm OUT
        usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
        return true;
    }

    if (ep_num == (EP_BULK_IN & 0x7F)) {
        // Streaming DATA (e.g. 0x9209)
        if (s_tx_stream_active) {
            if (s_tx_stream_payload_off < s_tx_stream_payload_len) {
                const size_t rem = s_tx_stream_payload_len - s_tx_stream_payload_off;
                const size_t chunk = (rem > sizeof(s_tx_buf)) ? sizeof(s_tx_buf) : rem;
                memcpy(s_tx_buf, s_tx_stream_payload + s_tx_stream_payload_off, chunk);
                s_tx_stream_payload_off += chunk;
                // Log EXACT bytes we are about to send to RS3 over bulk IN (continuation chunk, no header).
                (void)usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, (uint16_t)chunk);
                return true;
            }

            if (s_tx_stream_need_zlp && !s_tx_stream_zlp_sent) {
                s_tx_stream_zlp_sent = true;
                (void)usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, 0);
                return true;
            }

            const bool send_ok = s_tx_stream_send_ok_after;
            const uint32_t tid = s_tx_stream_tid;
            s_tx_stream_active = false;
            s_tx_stream_send_ok_after = false;
            s_tx_stream_payload = NULL;
            s_tx_stream_payload_len = 0;
            s_tx_stream_payload_off = 0;
            s_tx_stream_need_zlp = false;
            s_tx_stream_zlp_sent = false;

            if (send_ok) {
                send_response(rhport, PTP_RC_OK, tid);
            }
            return true;
        }

        return true;
    }

    return true;
}

static usbd_class_driver_t const s_ptp_driver = {
    .name = "ptp_cam",
    .init = ptp_init,
    .deinit = ptp_deinit,
    .reset = ptp_reset,
    .open = ptp_open,
    .control_xfer_cb = ptp_control_xfer_cb,
    .xfer_cb = ptp_xfer_cb,
    .xfer_isr = NULL,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &s_ptp_driver;
}

// -----------------------------
// Public API
// -----------------------------

esp_err_t rs3_usb_ptp_cam_start(void)
{
    tinyusb_config_t tusb_cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = {
            .skip_setup = false,
            .self_powered = false,
            .vbus_monitor_io = -1,
        },
        .task = {
            .size = 4096,
            .priority = 5,
            .xCoreID = 0,
        },
        .descriptor = {
            .device = &s_dev_desc,
            .qualifier = NULL,
            .string = s_str_desc,
            .string_count = (int)(sizeof(s_str_desc) / sizeof(s_str_desc[0])),
            .full_speed_config = s_fs_cfg_desc,
            .high_speed_config = NULL,
        },
        .event_cb = NULL,
        .event_arg = NULL,
    };

    rs3_tcp_logf("[USB] Starting USB PTP device VID=0x%04X PID=0x%04X\r\n",
                 (unsigned)CONFIG_RS3_USB_PTP_VID, (unsigned)CONFIG_RS3_USB_PTP_PID);
    return tinyusb_driver_install(&tusb_cfg);
}

#endif // CONFIG_RS3_USB_PTP_IMPL_LEGACY && !CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

#else

esp_err_t rs3_usb_ptp_cam_start(void)
{
    return ESP_OK;
}

#endif
