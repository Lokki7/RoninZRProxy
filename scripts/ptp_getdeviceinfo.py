#!/usr/bin/env python3
"""
Send PTP GetDeviceInfo (0x1001) to a USB Still Image (PTP) device and print the
raw DeviceInfo dataset as hex.
Works best on macOS with:
  brew install libusb
  python3 -m pip install --user pyusb

Notes:
- Close Photos/Image Capture first.
- You may need: sudo killall PTPCamera
- You may need to run this script with sudo on macOS to get USB access.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

import usb.core
import usb.util


PTP_CLASS = 0x06
PTP_SUBCLASS = 0x01
PTP_PROTOCOL = 0x01

PTP_CT_COMMAND = 0x0001
PTP_CT_DATA = 0x0002
PTP_CT_RESPONSE = 0x0003

PTP_OC_GET_DEVICE_INFO = 0x1001


def hexdump(buf: bytes, prefix: str = "") -> None:
    for off in range(0, len(buf), 16):
        chunk = buf[off : off + 16]
        print(f"{prefix}{off:04x}: " + " ".join(f"{b:02x}" for b in chunk))


def le16(b: bytes, off: int) -> int:
    return struct.unpack_from("<H", b, off)[0]


def le32(b: bytes, off: int) -> int:
    return struct.unpack_from("<I", b, off)[0]


def read_ptp_container(ep_in, timeout_ms: int = 5000) -> bytes:
    """
    Read one PTP container from bulk IN.
    Container format: <len32><type16><code16><tid32> + payload...
    """
    first = ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    if len(first) < 12:
        raise RuntimeError(f"Short read ({len(first)} bytes), expected >= 12")
    total_len = le32(first, 0)
    if total_len < 12:
        raise RuntimeError(f"Invalid container length={total_len}")
    buf = bytearray(first)
    while len(buf) < total_len:
        buf += ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    return bytes(buf[:total_len])


@dataclass
class PtpIface:
    dev: usb.core.Device
    cfg_value: int
    intf_num: int


def find_ptp_interface(
    vid: Optional[int] = None,
    pid: Optional[int] = None,
    pick: int = 0,
) -> PtpIface:
    matches: list[PtpIface] = []
    for dev in usb.core.find(find_all=True):
        if vid is not None and int(dev.idVendor) != vid:
            continue
        if pid is not None and int(dev.idProduct) != pid:
            continue

        try:
            for cfg in dev:
                for intf in cfg:
                    if (
                        intf.bInterfaceClass == PTP_CLASS
                        and intf.bInterfaceSubClass == PTP_SUBCLASS
                        and intf.bInterfaceProtocol == PTP_PROTOCOL
                    ):
                        matches.append(PtpIface(dev=dev, cfg_value=cfg.bConfigurationValue, intf_num=intf.bInterfaceNumber))
        except usb.core.USBError:
            # Some devices error during iteration unless privileged; ignore until selected.
            continue

    if not matches:
        raise RuntimeError("No USB Still Image (PTP) interfaces found.")
    if pick < 0 or pick >= len(matches):
        raise RuntimeError(f"--pick out of range (0..{len(matches)-1})")
    return matches[pick]


def claim_interface(dev: usb.core.Device, cfg_value: int, intf_num: int) -> Tuple[object, object]:
    # Set config
    dev.set_configuration(cfg_value)
    cfg = dev.get_active_configuration()
    intf = usb.util.find_descriptor(cfg, bInterfaceNumber=intf_num)
    if intf is None:
        raise RuntimeError("Interface not found after set_configuration")

    # On some platforms, detach kernel driver if present
    try:
        if dev.is_kernel_driver_active(intf_num):  # type: ignore[attr-defined]
            dev.detach_kernel_driver(intf_num)  # type: ignore[attr-defined]
    except (NotImplementedError, usb.core.USBError):
        pass

    usb.util.claim_interface(dev, intf_num)

    # Find bulk IN/OUT endpoints (ignore interrupt IN for events)
    ep_in = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK,
    )
    ep_out = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK,
    )

    if ep_in is None or ep_out is None:
        raise RuntimeError("Bulk IN/OUT endpoints not found on PTP interface")

    return ep_in, ep_out


def ptp_get_device_info(ep_in, ep_out, tid: int = 1) -> Tuple[bytes, bytes]:
    cmd = struct.pack("<IHHI", 12, PTP_CT_COMMAND, PTP_OC_GET_DEVICE_INFO, tid)
    ep_out.write(cmd, timeout=5000)
    data = read_ptp_container(ep_in, timeout_ms=5000)
    resp = read_ptp_container(ep_in, timeout_ms=5000)
    return data, resp


def main() -> int:
    ap = argparse.ArgumentParser(description="Send PTP GetDeviceInfo over USB and print raw dataset hex.")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=None, help="USB VID (e.g. 0x054c)")
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=None, help="USB PID (e.g. 0x0d9f)")
    ap.add_argument("--pick", type=int, default=0, help="Pick Nth matching PTP interface (default: 0)")
    ap.add_argument("--dump-container", action="store_true", help="Also dump full DATA/RESP containers (incl headers)")
    args = ap.parse_args()

    ptp = find_ptp_interface(vid=args.vid, pid=args.pid, pick=args.pick)
    d = ptp.dev
    print(f"Using VID=0x{int(d.idVendor):04x} PID=0x{int(d.idProduct):04x} cfg={ptp.cfg_value} if={ptp.intf_num}")

    ep_in = ep_out = None
    try:
        ep_in, ep_out = claim_interface(d, ptp.cfg_value, ptp.intf_num)
        data, resp = ptp_get_device_info(ep_in, ep_out, tid=1)

        dlen, dtype, dop, dtid = struct.unpack_from("<IHHI", data, 0)
        rlen, rtype, rcode, rtid = struct.unpack_from("<IHHI", resp, 0)
        print(f"DATA: len={dlen} type=0x{dtype:04x} code=0x{dop:04x} tid={dtid}")
        print(f"RESP: len={rlen} type=0x{rtype:04x} code=0x{rcode:04x} tid={rtid}")

        if args.dump_container:
            print("\nDATA container (hex):")
            hexdump(data)
            print("\nRESP container (hex):")
            hexdump(resp)

        dataset = data[12:]
        print("\nDeviceInfo dataset (hex):")
        hexdump(dataset)

    finally:
        try:
            if ep_in is not None:
                usb.util.release_interface(d, ptp.intf_num)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(d)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


