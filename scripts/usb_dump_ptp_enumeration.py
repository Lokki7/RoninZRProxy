#!/usr/bin/env python3
"""
Dump USB enumeration-relevant info from a real PTP (Still Image) camera:
- VID/PID
- bcdDevice
- Manufacturer/Product/Serial USB string descriptors
- PTP interface number and bulk endpoint addresses + wMaxPacketSize

Use this to copy values into ESP32 menuconfig (VID/PID/strings/bcdDevice) so RS3
sees something as close to the real camera as possible.

macOS tips:
- Close Photos / Image Capture
- sudo killall PTPCamera
- You may need sudo to access USB device from python.
"""

from __future__ import annotations

import argparse
from typing import Optional, Tuple

import usb.core
import usb.util


PTP_CLASS, PTP_SUBCLASS, PTP_PROTOCOL = 0x06, 0x01, 0x01


def get_str(dev: usb.core.Device, idx: int) -> Optional[str]:
    if not idx:
        return None
    try:
        return usb.util.get_string(dev, idx)
    except Exception:
        return None


def find_ptp_device(vid: Optional[int], pid: Optional[int], pick: int) -> Tuple[usb.core.Device, int, int]:
    matches = []
    for dev in usb.core.find(find_all=True):
        if vid is not None and int(dev.idVendor) != vid:
            continue
        if pid is not None and int(dev.idProduct) != pid:
            continue
        try:
            for cfg in dev:
                for intf in cfg:
                    if (intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol) == (PTP_CLASS, PTP_SUBCLASS, PTP_PROTOCOL):
                        matches.append((dev, cfg.bConfigurationValue, intf.bInterfaceNumber))
        except usb.core.USBError:
            continue

    if not matches:
        raise SystemExit("No USB Still Image (PTP) interfaces found.")
    if pick < 0 or pick >= len(matches):
        raise SystemExit(f"--pick out of range (0..{len(matches)-1})")
    return matches[pick]


def main() -> int:
    ap = argparse.ArgumentParser(description="Dump USB enumeration parameters from a PTP camera.")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=None, help="Filter VID (e.g. 0x054c)")
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=None, help="Filter PID")
    ap.add_argument("--pick", type=int, default=0, help="Pick Nth matching PTP interface")
    args = ap.parse_args()

    dev, cfg_value, ifnum = find_ptp_device(args.vid, args.pid, args.pick)

    print(f"Device: VID=0x{int(dev.idVendor):04x} PID=0x{int(dev.idProduct):04x}")
    print(f"bcdDevice: 0x{int(dev.bcdDevice):04x}")

    m = get_str(dev, dev.iManufacturer)
    p = get_str(dev, dev.iProduct)
    s = get_str(dev, dev.iSerialNumber)
    print(f"iManufacturer: {repr(m)}")
    print(f"iProduct     : {repr(p)}")
    print(f"iSerialNumber: {repr(s)}")

    # Endpoint details on the PTP interface
    dev.set_configuration(cfg_value)
    cfg = dev.get_active_configuration()
    intf = usb.util.find_descriptor(cfg, bInterfaceNumber=ifnum)
    if intf is None:
        print(f"PTP interface: {ifnum} (not found after set_configuration)")
        return 0

    bulk_in = []
    bulk_out = []
    for ep in intf:
        addr = int(ep.bEndpointAddress)
        attrs = int(ep.bmAttributes)
        xfertype = attrs & 0x3
        if xfertype != usb.util.ENDPOINT_TYPE_BULK:
            continue
        if usb.util.endpoint_direction(addr) == usb.util.ENDPOINT_IN:
            bulk_in.append((addr, int(ep.wMaxPacketSize)))
        else:
            bulk_out.append((addr, int(ep.wMaxPacketSize)))

    print(f"PTP interface number: {ifnum}")
    print("Bulk IN endpoints :", ", ".join([f"0x{a:02x} wMaxPacketSize={mps}" for a, mps in bulk_in]) or "(none)")
    print("Bulk OUT endpoints:", ", ".join([f"0x{a:02x} wMaxPacketSize={mps}" for a, mps in bulk_out]) or "(none)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


