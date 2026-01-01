## rs3proxy (ESP-IDF, ESP32‑S3)

Project for researching and controlling a camera via **DJI RS3** over **USB PTP**:

- **PTP emulation on ESP32** (a “Sony-like” camera so RS3 accepts the device).
- **Raw proxy mode**: ESP32 as a USB device for RS3 plus a TCP bridge to a PC (for sniff/relay).
- **TCP server** for logs and commands (incl. OTA and reboot).
- **LCD + touch UI**: status and “Update FW” / “Restart MCU” buttons.

Board: **Waveshare ESP32-S3-Touch-LCD-1.83**. Handy Waveshare guide: [ESP32-S3-Touch-LCD-1.83 — Run the First ESP-IDF Demo](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.83#Run_the_First_ESP-IDF_Demo)

### Ports / connectivity

- **TCP logs + commands**: `CONFIG_RS3_TCP_SERVER_PORT` (default **1234**)  
  Connect: `nc <esp-ip> 1234`
- **PTP raw proxy** (USB bulk <-> TCP binary protocol): `CONFIG_RS3_USB_PTP_PROXY_PORT` (default **1235**)  
  Used by `scripts/rs3_ptp_raw_proxy.py` (and/or `scripts/rs3_ptp_proxy.py`).

### Configuration (menuconfig)

All key settings are in `main/Kconfig.projbuild`:

- **Wi‑Fi (STA)**: SSID/password to connect and obtain an IP
- **TCP server**: logs/commands port (default 1234)
- **OTA**: `CONFIG_RS3_OTA_URL` (default URL for the “Update FW” button and the `ota <url>` command)
- **USB PTP (camera emulation)**:
  - `CONFIG_RS3_USB_PTP_ENABLE`
  - VID/PID/bcdDevice/strings (to match a real camera)
  - **PTP implementation**:
    - **Standard** (`RS3_USB_PTP_IMPL_STD`): basic implementation for PC-host
    - **Legacy (RS3)** (`RS3_USB_PTP_IMPL_LEGACY`): current implementation for RS3
    - **Raw proxy** (`RS3_USB_PTP_IMPL_PROXY_RAW`): USB bulk <-> TCP bridge without modifications

### Build

In the project root:

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

If you're using ESP-IDF from an installed path, typically this is enough:

```bash
source ~/esp/esp-idf/export.sh
export IDF_COMPONENT_MANAGER=1
cd /path/to/rs3proxy
idf.py build
```

### Flash and monitor

Run flashing/monitoring however you prefer (COM/tty depends on your system):

```bash
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

### TCP logs and commands

Connect to logs:

```bash
nc <esp-ip> 1234
```

Commands (TCP):

- `ota <url>`: pull-OTA update (see `CONFIG_RS3_OTA_URL`)
- `pair`: start Nikon Bluetooth pairing (same as the touch button)
- `shutter`: do a Bluetooth “shutter click” (same as the touch button)
- `reboot` / `restart` / `reset`: reboot the MCU

### PTP debug logging

Firmware has a compile-time flag:

- `RS3_PTP_LOG_DEBUG=0` (default): quiet/minimal
- `RS3_PTP_LOG_DEBUG=1`: verbose logs (for USB/PTP debugging)

### Touch buttons (LCD)

There are two buttons at the bottom of the screen:

- **Update FW**: starts OTA using the URL from `CONFIG_RS3_OTA_URL`
- **Restart MCU**: calls `esp_restart()`

### Scripts (macOS / USB PTP)

See `scripts/README.md`:

- `scripts/usb_dump_ptp_enumeration.py`: dump VID/PID/bcdDevice/strings and endpoints from a real camera
- `scripts/ptp_getdeviceinfo.py`: get raw DeviceInfo bytes
- `scripts/rs3_ptp_raw_proxy.py`: raw proxy (for `RS3_USB_PTP_IMPL_PROXY_RAW` mode)

On macOS you often need:

```bash
sudo killall PTPCamera 2>/dev/null || true
```

