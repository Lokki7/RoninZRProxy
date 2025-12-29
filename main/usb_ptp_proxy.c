#include "usb_ptp_cam.h"

#include "sdkconfig.h"

#if CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW

// Kconfig symbol may be absent when hidden by depends-on; keep build stable.
#ifndef CONFIG_RS3_USB_PTP_BCD_DEVICE
#define CONFIG_RS3_USB_PTP_BCD_DEVICE 0x0100
#endif

#include <inttypes.h>
#include <string.h>

#include "esp_check.h"

#include "tinyusb.h"
#include "tusb.h"

#include "log_tcp.h"
#include "ptp_proxy_server.h"
#include "tcp_server.h"

// Raw proxy protocol frame types (ESP <-> PC), sent over ptp_proxy_server framing:
// - ESP -> PC: RAW_OUT (exact bytes received from RS3 over bulk OUT)
// - PC  -> ESP: RAW_IN  (exact bytes to send back to RS3 over bulk IN)
// The ESP does not parse or modify bytes.
#define RS3_PTP_RAW_PROXY_T_RAW_OUT  0x10
#define RS3_PTP_RAW_PROXY_T_RAW_IN   0x11
// - PC  -> ESP: RAW_DONE (end-of-reply marker for one RS3 OUT command; no payload)
#define RS3_PTP_RAW_PROXY_T_RAW_DONE 0x12

// Endpoints (Full-speed). Match the real Sony camera as closely as possible:
// the ILCE-5100 interface reports only 2 endpoints (bulk IN/OUT), no interrupt/event endpoint.
static const uint8_t EP_BULK_IN = 0x81;
static const uint8_t EP_BULK_OUT= 0x02;

// Still Image (PTP) class-specific requests on EP0
#define PTP_REQ_CANCEL             0x64
#define PTP_REQ_GET_EXT_EVENT_DATA 0x65
#define PTP_REQ_RESET              0x66
#define PTP_REQ_GET_DEVICE_STATUS  0x67

// PTP Response code OK
#define PTP_RC_OK 0x2001

static uint8_t s_rx_buf[64];
static uint8_t s_tx_buf[512];
static uint8_t s_ctrl_buf[64];

static uint8_t s_itf_num = 0;
static bool s_mounted = false;

static bool s_pending_zlp = false;
static bool s_in_busy = false;

// Queue of pending IN frames received from PC
typedef struct {
    size_t len;
    uint8_t buf[512];
} in_frame_t;

static in_frame_t s_in_q[8];
static int s_in_q_count = 0;
static int s_in_q_idx = 0;

static void log_hex8(const char *prefix, const uint8_t *buf, size_t n)
{
    char line[96];
    size_t off = 0;
    off += (size_t)snprintf(line + off, sizeof(line) - off, "%s", prefix);
    size_t k = (n > 8) ? 8 : n;
    for (size_t i = 0; i < k && off + 4 < sizeof(line); i++) {
        off += (size_t)snprintf(line + off, sizeof(line) - off, "%02X%s", buf[i], (i + 1 < k) ? " " : "");
    }
    off += (size_t)snprintf(line + off, sizeof(line) - off, "\r\n");
    (void)rs3_tcp_server_send(line, off);
}

// -----------------------------
// USB descriptors (reuse from usb_ptp_cam legacy)
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

#define PTP_ITF_CLASS   0x06
#define PTP_ITF_SUBCLASS 0x01
#define PTP_ITF_PROTOCOL 0x01

#define CFG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7 + 7)

static uint8_t const s_fs_cfg_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CFG_TOTAL_LEN, 0x80, 100),

    9, TUSB_DESC_INTERFACE,
    0x00, 0x00, // itf num, alt
    0x02,       // num endpoints
    PTP_ITF_CLASS, PTP_ITF_SUBCLASS, PTP_ITF_PROTOCOL,
    STRID_ITF,

    7, TUSB_DESC_ENDPOINT,
    EP_BULK_OUT, TUSB_XFER_BULK, 0x40, 0x00,
    0x00,

    7, TUSB_DESC_ENDPOINT,
    EP_BULK_IN, TUSB_XFER_BULK, 0x40, 0x00,
    0x00,
};

// -----------------------------
// Custom TinyUSB class driver (raw passthrough)
// -----------------------------

#include "device/usbd_pvt.h"

static void ptp_init(void) {}
static bool ptp_deinit(void) { return true; }
static void ptp_reset(uint8_t rhport) { (void)rhport; s_mounted = false; }

static uint16_t ptp_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    (void)max_len;
    if (itf_desc->bInterfaceClass != PTP_ITF_CLASS ||
        itf_desc->bInterfaceSubClass != PTP_ITF_SUBCLASS ||
        itf_desc->bInterfaceProtocol != PTP_ITF_PROTOCOL) {
        return 0;
    }

    s_itf_num = itf_desc->bInterfaceNumber;

    uint16_t len = itf_desc->bLength;
    uint8_t const *p = (uint8_t const *)itf_desc + itf_desc->bLength;
    for (int i = 0; i < (int)itf_desc->bNumEndpoints; i++) {
        tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
        if (ep->bDescriptorType == TUSB_DESC_ENDPOINT) {
            usbd_edpt_open(rhport, ep);
        }
        len += ep->bLength;
        p += ep->bLength;
    }

    s_in_q_count = 0;
    s_in_q_idx = 0;
    s_pending_zlp = false;
    s_in_busy = false;

    // Start first OUT transfer
    usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
    s_mounted = true;
    rs3_tcp_logf("[USB] PTP RAW PROXY opened (itf=%u) proxy_port=%d\r\n", s_itf_num, CONFIG_RS3_USB_PTP_PROXY_PORT);
    return len;
}

static bool ptp_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    // Handle Still Image class requests on EP0. Do NOT stall: RS3 may require these to accept the camera.
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;
    if (request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE) return false;
    if ((uint8_t)tu_le16toh(request->wIndex) != s_itf_num) return false;

    if (stage == CONTROL_STAGE_SETUP) {
        rs3_tcp_logf("[RAW][EP0] class req=0x%02X wLen=%u\r\n",
                     request->bRequest, (unsigned)tu_le16toh(request->wLength));
    }

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
            // Clear stalls and reset internal queues/state.
            if (stage == CONTROL_STAGE_SETUP) {
                if (usbd_edpt_stalled(rhport, EP_BULK_OUT)) usbd_edpt_clear_stall(rhport, EP_BULK_OUT);
                if (usbd_edpt_stalled(rhport, EP_BULK_IN))  usbd_edpt_clear_stall(rhport, EP_BULK_IN);
                s_in_q_count = 0;
                s_in_q_idx = 0;
                s_pending_zlp = false;
                s_in_busy = false;
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

static void start_next_in(uint8_t rhport)
{
    if (s_in_busy) return;
    if (s_pending_zlp) {
        s_pending_zlp = false;
        s_in_busy = true;
        rs3_tcp_logf("[RAW] -> IN ZLP\r\n");
        (void)usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, 0);
        return;
    }
    if (s_in_q_idx >= s_in_q_count) {
        s_in_q_count = 0;
        s_in_q_idx = 0;
        return;
    }
    in_frame_t *f = &s_in_q[s_in_q_idx];
    s_in_busy = true;
    rs3_tcp_logf("[RAW] -> IN bytes=%u idx=%d/%d\r\n", (unsigned)f->len, s_in_q_idx + 1, s_in_q_count);
    log_hex8("[RAW] -> IN head: ", f->buf, f->len);
    (void)usbd_edpt_xfer(rhport, EP_BULK_IN, f->buf, (uint16_t)f->len);
}

static bool ptp_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    const bool is_in = (ep_addr & 0x80) != 0;
    const uint8_t ep_num = (uint8_t)(ep_addr & 0x7F);

    if (!is_in && ep_num == (EP_BULK_OUT & 0x7F)) {
        const size_t n = (size_t)xferred_bytes;
        rs3_tcp_logf("[RAW] <- OUT bytes=%" PRIu32 " res=%d\r\n", xferred_bytes, (int)result);
        log_hex8("[RAW] <- OUT head: ", s_rx_buf, n);

        if (rs3_ptp_proxy_is_connected()) {
            (void)rs3_ptp_proxy_send_frame(RS3_PTP_RAW_PROXY_T_RAW_OUT, s_rx_buf, n);

            // Receive up to N raw IN frames from PC.
            // IMPORTANT: Don't rely on timeouts to decide "end of reply" (OpenSession is usually a single short response).
            // Python sends a RAW_DONE marker to terminate the reply for a given RS3 OUT command.
            s_in_q_count = 0;
            s_in_q_idx = 0;
            s_pending_zlp = false;

            for (int i = 0; i < (int)(sizeof(s_in_q) / sizeof(s_in_q[0])); i++) {
                uint8_t ftype = 0;
                size_t flen = 0;
                esp_err_t rr = rs3_ptp_proxy_recv_frame(&ftype,
                                                       s_in_q[i].buf,
                                                       sizeof(s_in_q[i].buf),
                                                       &flen,
                                                       1500);
                if (rr == ESP_ERR_TIMEOUT) {
                    break; // no more frames right now
                }
                if (rr != ESP_OK) {
                    rs3_tcp_logf("[RAW] proxy recv failed (%s)\r\n", esp_err_to_name(rr));
                    break;
                }
                if (ftype == RS3_PTP_RAW_PROXY_T_RAW_DONE) {
                    rs3_tcp_logf("[RAW] proxy: DONE\r\n");
                    break;
                } else if (ftype == RS3_PTP_RAW_PROXY_T_RAW_IN) {
                    s_in_q[i].len = flen;
                    s_in_q_count = i + 1;
                } else {
                    rs3_tcp_logf("[RAW] unexpected proxy frame type=0x%02X\r\n", ftype);
                    break;
                }
            }

            // Start sending queued frames immediately.
            if (s_in_q_count > 0) {
                s_in_busy = false;
                start_next_in(rhport);
            } else {
                rs3_tcp_logf("[RAW] proxy: no IN frames queued\r\n");
            }
        }

        // re-arm OUT
        usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
        return true;
    }

    if ((ep_addr & 0x7F) == (EP_BULK_IN & 0x7F)) {
        (void)result;
        rs3_tcp_logf("[RAW] <- IN complete bytes=%" PRIu32 "\r\n", xferred_bytes);
        s_in_busy = false;
        if (s_in_q_idx < s_in_q_count) s_in_q_idx++;
        start_next_in(rhport);
        return true;
    }

    return true;
}

static usbd_class_driver_t const s_ptp_driver = {
    .name = "ptp_raw_proxy",
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
// Public API (same entrypoint)
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

    rs3_tcp_logf("[USB] Starting USB PTP RAW PROXY VID=0x%04X PID=0x%04X\r\n",
                 (unsigned)CONFIG_RS3_USB_PTP_VID, (unsigned)CONFIG_RS3_USB_PTP_PID);
    return tinyusb_driver_install(&tusb_cfg);
}

#endif // CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_PROXY_RAW


