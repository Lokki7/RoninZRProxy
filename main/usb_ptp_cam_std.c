#include "usb_ptp_cam.h"

#include "sdkconfig.h"

#ifndef CONFIG_RS3_USB_PTP_ENABLE
#define CONFIG_RS3_USB_PTP_ENABLE 0
#endif

#ifndef CONFIG_RS3_USB_PTP_IMPL_STD
#define CONFIG_RS3_USB_PTP_IMPL_STD 0
#endif

#if CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_STD

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "tinyusb.h"
#include "tusb.h"

#include "log_tcp.h"
#include "tcp_server.h"

// Custom class driver hooks
#include "device/usbd_pvt.h"

// -----------------------------
// PTP constants (subset)
// -----------------------------

// Container types
#define PTP_CT_COMMAND  1
#define PTP_CT_DATA     2
#define PTP_CT_RESPONSE 3
#define PTP_CT_EVENT    4

// Operation codes (subset)
#define PTP_OC_GET_DEVICE_INFO 0x1001
#define PTP_OC_OPEN_SESSION    0x1002

// Response codes (subset, PIMA 15740 / PTP)
#define PTP_RC_OK                    0x2001
#define PTP_RC_GENERAL_ERROR         0x2002
#define PTP_RC_SESSION_NOT_OPEN      0x2003
#define PTP_RC_OPERATION_NOT_SUPPORTED 0x2005
#define PTP_RC_INVALID_PARAMETER     0x201D
#define PTP_RC_SESSION_ALREADY_OPEN  0x201E

// Still Image class-specific requests (PIMA 15740-2000: D.5.2)
#define PTP_REQ_CANCEL            0x64
#define PTP_REQ_GET_EXT_EVENT_DATA 0x65
#define PTP_REQ_RESET             0x66
#define PTP_REQ_GET_DEVICE_STATUS 0x67

typedef struct __attribute__((packed)) {
  uint32_t len;
  uint16_t type;
  uint16_t code;
  uint32_t trans_id;
} ptp_hdr_t;

static const uint8_t EP_BULK_IN  = 0x81;
static const uint8_t EP_BULK_OUT = 0x02;
static const uint8_t EP_EVT_IN   = 0x83; // interrupt (events)

static uint8_t s_rx_buf[64];
static uint8_t s_tx_buf[512];
static uint8_t s_ctrl_buf[32];

static uint8_t s_itf_num = 0;
static bool s_mounted = false;

static bool s_session_open = false;
static uint32_t s_session_id = 0;

static bool s_pending_ok = false;
static uint32_t s_pending_ok_tid = 0;

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
  .bcdDevice          = 0x0100,
  .iManufacturer      = STRID_MANUFACTURER,
  .iProduct           = STRID_PRODUCT,
  .iSerialNumber      = STRID_SERIAL,
  .bNumConfigurations = 0x01
};

// Still Image (PTP) interface descriptor:
// Interface class: 0x06 (Still Imaging), subclass 0x01, protocol 0x01 (PTP)
#define PTP_ITF_CLASS     0x06
#define PTP_ITF_SUBCLASS  0x01
#define PTP_ITF_PROTOCOL  0x01

#define CFG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7 + 7 + 7)

static uint8_t const s_fs_cfg_desc[] = {
  TUD_CONFIG_DESCRIPTOR(1, 1, 0, CFG_TOTAL_LEN, 0x80, 100),

  // Interface
  9, TUSB_DESC_INTERFACE,
  0x00, 0x00,
  0x03,
  PTP_ITF_CLASS, PTP_ITF_SUBCLASS, PTP_ITF_PROTOCOL,
  STRID_ITF,

  // Interrupt IN (events)
  7, TUSB_DESC_ENDPOINT,
  EP_EVT_IN, TUSB_XFER_INTERRUPT, 0x40, 0x00,
  0x01,

  // Bulk OUT (commands/data)
  7, TUSB_DESC_ENDPOINT,
  EP_BULK_OUT, TUSB_XFER_BULK, 0x40, 0x00,
  0x00,

  // Bulk IN (responses/data)
  7, TUSB_DESC_ENDPOINT,
  EP_BULK_IN, TUSB_XFER_BULK, 0x40, 0x00,
  0x00,
};

// -----------------------------
// Helpers
// -----------------------------

static inline uint16_t rd_le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t rd_le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void wr_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static inline void wr_le32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF); }

static void tcp_hex_dump_lines(const char *prefix, const uint8_t *data, size_t len)
{
  char line[512];
  for (size_t off0 = 0; off0 < len; off0 += 16) {
    size_t off = 0;
    off += (size_t)snprintf(line + off, sizeof(line) - off, "%s%04X: ", prefix, (unsigned)off0);
    size_t chunk = (len - off0 > 16) ? 16 : (len - off0);
    for (size_t i = 0; i < chunk && off + 4 < sizeof(line); i++) {
      off += (size_t)snprintf(line + off, sizeof(line) - off, "%02X%s", data[off0 + i], (i + 1 < chunk) ? " " : "");
    }
    off += (size_t)snprintf(line + off, sizeof(line) - off, "\r\n");
    (void)rs3_tcp_server_send(line, off);
  }
}

static void ptp_write_string_bytes(uint8_t **pp, const char *s)
{
  uint8_t *p = *pp;
  if (!s) { *p++ = 0; *pp = p; return; }
  size_t len = strlen(s);
  if (len > 254) len = 254;
  uint8_t n = (uint8_t)(len + 1); // incl. null terminator
  *p++ = n;
  for (uint8_t i = 0; i < n - 1; i++) { *p++ = (uint8_t)s[i]; *p++ = 0; }
  *p++ = 0; *p++ = 0;
  *pp = p;
}

static size_t build_device_info(uint8_t *out, size_t cap)
{
  // DeviceInfo dataset (PTP 1.x). All fields are little-endian.
  uint8_t *p = out;
  if (cap < 128) return 0;

  // StandardVersion = 100 (per PTP)
  wr_le16(p, 0x0064); p += 2;
  // VendorExtensionID, VendorExtensionVersion
  wr_le32(p, 0x00000000); p += 4;
  wr_le16(p, 0x0064); p += 2;
  // VendorExtensionDesc (empty string)
  *p++ = 0x00;
  // FunctionalMode = 0
  wr_le16(p, 0x0000); p += 2;

  // SupportedOperations (array of u16): count (u32) + entries
  wr_le32(p, 2); p += 4;
  wr_le16(p, PTP_OC_GET_DEVICE_INFO); p += 2;
  wr_le16(p, PTP_OC_OPEN_SESSION); p += 2;

  // SupportedEvents / DeviceProperties / CaptureFormats / ImageFormats: empty arrays
  wr_le32(p, 0); p += 4;
  wr_le32(p, 0); p += 4;
  wr_le32(p, 0); p += 4;
  wr_le32(p, 0); p += 4;

  // Manufacturer / Model / DeviceVersion / SerialNumber
  ptp_write_string_bytes(&p, CONFIG_RS3_USB_PTP_MANUFACTURER);
  ptp_write_string_bytes(&p, CONFIG_RS3_USB_PTP_PRODUCT);
  ptp_write_string_bytes(&p, "1.00");
  ptp_write_string_bytes(&p, CONFIG_RS3_USB_PTP_SERIAL);

  return (size_t)(p - out);
}

static size_t write_ptp_hdr_std(uint8_t *dst, uint32_t len, uint16_t type, uint16_t code, uint32_t tid)
{
  wr_le32(dst + 0, len);
  wr_le16(dst + 4, type);
  wr_le16(dst + 6, code);
  wr_le32(dst + 8, tid);
  return 12;
}

static void send_response(uint8_t rhport, uint16_t resp_code, uint32_t trans_id)
{
  size_t hdr = write_ptp_hdr_std(s_tx_buf, 12, PTP_CT_RESPONSE, resp_code, trans_id);
  rs3_tcp_logf("[PTP-STD] -> RESP code=0x%04X tid=%" PRIu32 "\r\n", resp_code, trans_id);
  tcp_hex_dump_lines("[PTP-STD] tx ", s_tx_buf, hdr);
  (void)usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, (uint16_t)hdr);
}

static void send_data_and_ok(uint8_t rhport, uint16_t op_code, uint32_t trans_id, const uint8_t *payload, size_t payload_len)
{
  if (payload_len + 12 > sizeof(s_tx_buf)) payload_len = sizeof(s_tx_buf) - 12;
  size_t hdr = write_ptp_hdr_std(s_tx_buf, (uint32_t)(12 + payload_len), PTP_CT_DATA, op_code, trans_id);
  memcpy(s_tx_buf + hdr, payload, payload_len);

  rs3_tcp_logf("[PTP-STD] -> DATA op=0x%04X tid=%" PRIu32 " bytes=%u\r\n",
               op_code, trans_id, (unsigned)payload_len);
  tcp_hex_dump_lines("[PTP-STD] tx ", s_tx_buf, (hdr + payload_len > 64) ? 64 : (hdr + payload_len));

  s_pending_ok = true;
  s_pending_ok_tid = trans_id;
  (void)usbd_edpt_xfer(rhport, EP_BULK_IN, s_tx_buf, (uint16_t)(hdr + payload_len));
}

// -----------------------------
// TinyUSB class driver
// -----------------------------

static void ptp_init(void) {}
static bool ptp_deinit(void) { return true; }
static void ptp_reset(uint8_t rhport)
{
  (void)rhport;
  s_mounted = false;
  s_session_open = false;
  s_session_id = 0;
  s_pending_ok = false;
  s_pending_ok_tid = 0;
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

  // Open 3 endpoints after interface descriptor
  uint16_t len = itf_desc->bLength;
  uint8_t const *p = (uint8_t const *)itf_desc + itf_desc->bLength;
  for (int i = 0; i < 3; i++) {
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
  rs3_tcp_logf("[USB] PTP-STD interface opened (itf=%u)\r\n", s_itf_num);
  return len;
}

static bool ptp_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  // Handle Still Image class requests on EP0. Return false to stall if unsupported.
  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;
  if (request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE) return false;
  if ((uint8_t)tu_le16toh(request->wIndex) != s_itf_num) return false;

  switch (request->bRequest) {
    case PTP_REQ_GET_DEVICE_STATUS: {
      // DeviceStatus: uint16 length (=4), uint16 status (PTP response code)
      if (stage == CONTROL_STAGE_SETUP) {
        wr_le16(s_ctrl_buf + 0, 4);
        wr_le16(s_ctrl_buf + 2, PTP_RC_OK);
        return tud_control_xfer(rhport, request, s_ctrl_buf, 4);
      }
      return true;
    }

    case PTP_REQ_CANCEL: {
      // Host sends a small structure (code + transaction_id). We accept it and ignore.
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
        s_session_open = false;
        s_session_id = 0;
        s_pending_ok = false;
      }
      return true;
    }

    case PTP_REQ_GET_EXT_EVENT_DATA:
    default:
      return false;
  }
}

static bool ptp_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  const bool is_in = (ep_addr & 0x80) != 0;
  const uint8_t ep_num = (uint8_t)(ep_addr & 0x7F);

  // OUT: receive command container
  if (!is_in && ep_num == (EP_BULK_OUT & 0x7F)) {
    size_t n = (size_t)xferred_bytes;
    rs3_tcp_logf("[PTP-STD] <- OUT bytes=%" PRIu32 " res=%d\r\n", xferred_bytes, (int)result);
    if (n >= 12) {
      uint32_t clen = rd_le32(s_rx_buf + 0);
      uint16_t ctype = rd_le16(s_rx_buf + 4);
      uint16_t code  = rd_le16(s_rx_buf + 6);
      uint32_t tid   = rd_le32(s_rx_buf + 8);

      rs3_tcp_logf("[PTP-STD] cmd len=%" PRIu32 " type=0x%04X op=0x%04X tid=%" PRIu32 " (rx=%u)\r\n",
                   clen, ctype, code, tid, (unsigned)n);
      tcp_hex_dump_lines("[PTP-STD]  ", s_rx_buf, (n > 64 ? 64 : n));

      if (clen < 12 || clen > n) {
        // We only handle commands that fit in a single USB OUT transfer.
        send_response(rhport, PTP_RC_GENERAL_ERROR, tid);
      } else if (ctype != PTP_CT_COMMAND) {
        send_response(rhport, PTP_RC_GENERAL_ERROR, tid);
      } else if (code == PTP_OC_GET_DEVICE_INFO) {
        uint8_t info[256];
        size_t info_len = build_device_info(info, sizeof(info));
        if (info_len > 0) {
          send_data_and_ok(rhport, code, tid, info, info_len);
        } else {
          send_response(rhport, PTP_RC_GENERAL_ERROR, tid);
        }
      } else if (code == PTP_OC_OPEN_SESSION) {
        // One parameter: SessionID (u32)
        if (s_session_open) {
          send_response(rhport, PTP_RC_SESSION_ALREADY_OPEN, tid);
        } else if (clen < 16) {
          send_response(rhport, PTP_RC_INVALID_PARAMETER, tid);
        } else {
          s_session_id = rd_le32(s_rx_buf + 12);
          s_session_open = true;
          rs3_tcp_logf("[PTP-STD] OpenSession sid=%" PRIu32 "\r\n", s_session_id);
          send_response(rhport, PTP_RC_OK, tid);
        }
      } else {
        // For now, only advertise + implement OpenSession/GetDeviceInfo.
        send_response(rhport, PTP_RC_OPERATION_NOT_SUPPORTED, tid);
      }
    }

    // re-arm OUT
    usbd_edpt_xfer(rhport, EP_BULK_OUT, s_rx_buf, sizeof(s_rx_buf));
    return true;
  }

  // IN complete: if we sent DATA, follow-up with RESPONSE OK
  if (is_in || ep_num == (EP_BULK_IN & 0x7F)) {
    if (ep_num == (EP_BULK_IN & 0x7F) && s_pending_ok) {
      s_pending_ok = false;
      send_response(rhport, PTP_RC_OK, s_pending_ok_tid);
    }
    return true;
  }

  return true;
}

static usbd_class_driver_t const s_ptp_driver = {
  .name = "ptp_std",
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

  rs3_tcp_logf("[USB] Starting USB PTP-STD device VID=0x%04X PID=0x%04X\r\n",
               (unsigned)CONFIG_RS3_USB_PTP_VID, (unsigned)CONFIG_RS3_USB_PTP_PID);
  return tinyusb_driver_install(&tusb_cfg);
}

#endif // CONFIG_RS3_USB_PTP_ENABLE && CONFIG_RS3_USB_PTP_IMPL_STD


