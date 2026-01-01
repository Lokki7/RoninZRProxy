## rs3proxy (ESP-IDF, ESP32‑S3)

> **Disclaimer:** This project’s code was written entirely using **GPT-5.2**.

`rs3proxy` is an ESP-IDF firmware project with two main parts:

- **DJI RS3 ↔ USB PTP**: ESP32 emulates a PTP still-image camera so RS3 accepts it (plus optional proxy modes for debugging/relaying).
- **Nikon camera control over Bluetooth (NimBLE)**: on-demand connect/pair + **shutter click** remote control.

Target board: **Waveshare ESP32-S3-Touch-LCD-1.83** (LCD+touch UI is optional). Waveshare guide: [ESP32-S3-Touch-LCD-1.83 — Run the First ESP-IDF Demo](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.83#Run_the_First_ESP-IDF_Demo)

### Ports

- **TCP logs + commands**: `CONFIG_RS3_TCP_SERVER_PORT` (default **1234**)  
  Connect: `nc <esp-ip> 1234`
- **PTP proxy server** (used by raw proxy / tooling): `CONFIG_RS3_USB_PTP_PROXY_PORT` (default **1235**)  
  Used by `scripts/rs3_ptp_raw_proxy.py` and `scripts/rs3_ptp_proxy.py`.

### Quick start

Build + flash:

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Connect to logs/commands:

```bash
nc <esp-ip> 1234
```

Useful TCP commands:

- `ota <url>`: pull-OTA update (see `CONFIG_RS3_OTA_URL`)
- `pair` / `btpair`: start Nikon Bluetooth pairing flow
- `shutter` / `btshutter`: Nikon shutter click (press + release)
- `reboot` / `restart` / `reset`: reboot the MCU

### Configuration (menuconfig)

Project settings live in `main/Kconfig.projbuild`:

- **Wi‑Fi (STA)**: `RS3_WIFI_*` (SSID/password)
- **TCP server**: `RS3_TCP_SERVER_*` (default port 1234)
- **OTA**: `RS3_OTA_*` (default URL for UI button and `ota <url>`)
- **USB PTP (camera emulation)**: `RS3_USB_PTP_*`

Bluetooth/Nikon settings are **ESP-IDF NimBLE** settings (not in `Kconfig.projbuild`):

- Enable **Bluetooth** + **NimBLE**: `CONFIG_BT_ENABLED=1`, `CONFIG_BT_NIMBLE_ENABLED=1`
- Recommended for Nikon pairing/session: enable NimBLE security + bonding + NVS persist (so keys can be stored)

### Part 1: DJI RS3 over USB PTP

Enable: `USB PTP (camera emulation)` → `CONFIG_RS3_USB_PTP_ENABLE`

PTP implementation options:

- **Legacy (RS3/DJI quirks)** (`RS3_USB_PTP_IMPL_LEGACY`): current implementation targeting RS3 behavior
- **Standard PTP** (`RS3_USB_PTP_IMPL_STD`): stricter/basic PTP containers for PC hosts
- **Raw proxy** (`RS3_USB_PTP_IMPL_PROXY_RAW`): do not emulate; forward raw USB bulk bytes to a PC over TCP

If you want RS3 to accept the device as a specific camera, set VID/PID/bcdDevice and USB strings in menuconfig.

### Part 2: Nikon camera control over Bluetooth (NimBLE)

Implementation is in `main/nikon_bt.cpp` and exposes:

- `pair` / `btpair`: scans for a Nikon camera advertising Nikon “remote” service UUID, connects, runs a Nikon remote-mode handshake, and stores the last peer in NVS (`rs3_bt/nikon_peer`) for later reconnect/session init.
- `shutter` / `btshutter`: triggers Nikon shutter click (GATT write: press + release). If not connected, it attempts a fast reconnect to the last peer first.

Typical usage:

- Put the Nikon camera into a state where it advertises its Bluetooth “remote” service (pairing/remote mode).
- Run `pair` and follow any prompts on the camera (if shown).
- Run `shutter` to trigger a photo.

Notes/limitations (current behavior):

- The firmware starts NimBLE at boot. It will attempt a **fast connect** to the last peer, otherwise it scans for a Nikon camera and connects.
- Background auto-reconnect after disconnect is currently **disabled** in code (`kBtAutoReconnectEnabled = false`). If you get disconnected, use `shutter` (fast reconnect) or `pair` (scan + connect) to re-establish the link.
- After reboot/reconnect, Nikon may require a “remote session init” handshake before accepting shutter writes; the firmware performs this automatically when needed.

### RS3 REC → Nikon shutter bridge

When RS3 sends a REC button full-press event over PTP (opcode `0x9207`), the firmware generates a Nikon Bluetooth shutter click on **both** START and STOP events (RS3 alternates them).

### LCD + touch UI (optional)

If the board has a display, the UI shows status and provides:

- **Update FW**: triggers OTA using `CONFIG_RS3_OTA_URL`
- **Restart MCU**: calls `esp_restart()`

### Scripts (macOS / USB PTP)

See `scripts/README.md`:

- `scripts/usb_dump_ptp_enumeration.py`: dump VID/PID/bcdDevice/strings and endpoints from a real camera
- `scripts/ptp_getdeviceinfo.py`: get raw DeviceInfo bytes
- `scripts/rs3_ptp_proxy.py`: PTP-level proxy (ESP TCP ↔ real camera over USB)
- `scripts/rs3_ptp_raw_proxy.py`: raw USB bulk proxy (ESP TCP ↔ raw USB bulk)

On macOS you often need:

```bash
sudo killall PTPCamera 2>/dev/null || true
```

