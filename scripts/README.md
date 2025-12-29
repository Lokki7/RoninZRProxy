## PTP tools

### `ptp_getdeviceinfo.py`

Sends **PTP GetDeviceInfo (0x1001)** to a connected USB camera (Still Image / PTP interface) and prints the **raw DeviceInfo dataset** as hex.

#### Install dependencies (macOS)

```bash
brew install libusb
python3 -m pip install --user pyusb
```

#### Run

Make sure Photos/Image Capture are closed, then free the system PTP grabber:

```bash
sudo killall PTPCamera 2>/dev/null || true
```

Run (often needs `sudo` on macOS for USB access):

```bash
cd /Users/pavel/workspace/rs3proxy
sudo python3 scripts/ptp_getdeviceinfo.py
```

If you have multiple PTP devices connected:

```bash
sudo python3 scripts/ptp_getdeviceinfo.py --pick 0
sudo python3 scripts/ptp_getdeviceinfo.py --pick 1
```

To filter by VID/PID:

```bash
sudo python3 scripts/ptp_getdeviceinfo.py --vid 0x054c --pid 0x0d9f
```

To dump the full PTP containers (headers + payload):

```bash
sudo python3 scripts/ptp_getdeviceinfo.py --dump-container
```

### `rs3_ptp_proxy.py`

Logging proxy that connects to the ESP32 **PTP proxy TCP port** (default `1235`), forwards PTP commands to a **real camera over USB**, and relays camera responses back to the ESP (which relays to RS3).

#### Enable on ESP32

- Enable `USB PTP (camera emulation)` → `Legacy RS3/DJI quirks`
- Enable `Enable PTP proxy over TCP (for sniffing/relaying via PC)`
- Set `PTP proxy TCP port` (default `1235`)

#### Run on macOS

```bash
brew install libusb
python3 -m pip install --user pyusb
sudo killall PTPCamera 2>/dev/null || true

cd /Users/pavel/workspace/rs3proxy
sudo python3 scripts/rs3_ptp_proxy.py --esp-host 192.168.1.91 --esp-port 1235 --log /tmp/rs3_ptp_proxy.log
```

Optional: restrict which camera to use (VID/PID):

```bash
sudo python3 scripts/rs3_ptp_proxy.py --esp-host 192.168.1.91 --vid 0x054c --pid 0x0957
```

### `rs3_ptp_raw_proxy.py`

Raw passthrough proxy for the **Raw proxy (USB bulk <-> TCP, no modifications)** firmware mode.

It logs raw RS3 BULK OUT packets. With `--camera`, it also tries to forward raw bytes to a real camera (best-effort) and sends back the first read chunk.

```bash
brew install libusb
python3 -m pip install --user pyusb
sudo killall PTPCamera 2>/dev/null || true

cd /Users/pavel/workspace/rs3proxy
sudo python3 scripts/rs3_ptp_raw_proxy.py --esp-host 192.168.1.91 --esp-port 1235 --log /tmp/rs3_ptp_raw_proxy.log
```


