#!/usr/bin/env python3
"""
Logging PTP proxy:

RS3 (USB host) -> ESP32-S3 (USB device) -> TCP (binary frames) -> this script -> real camera (USB device)

This script connects to the ESP32 "PTP proxy" TCP port (separate from logs/OTA),
receives standard PTP command containers, forwards them to a real camera over USB
(bulk OUT), reads back PTP containers from the camera (bulk IN), and returns them
to the ESP which relays to RS3.

Dependencies (macOS):
  brew install libusb
  python3 -m pip install --user pyusb

Notes:
- Close Photos / Image Capture first.
- You may need: sudo killall PTPCamera
- You may need to run the script with sudo for USB access.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import usb.core
import usb.util


# Proxy framing: uint32_be length (type+payload), uint8 type, payload...
T_CMD_STD = 0x01
T_CONT_STD = 0x02


# PTP constants
PTP_CLASS, PTP_SUBCLASS, PTP_PROTOCOL = 0x06, 0x01, 0x01
PTP_CT_COMMAND, PTP_CT_DATA, PTP_CT_RESPONSE = 1, 2, 3


def hexdump(buf: bytes, prefix: str = "") -> str:
    lines = []
    for off in range(0, len(buf), 16):
        chunk = buf[off : off + 16]
        lines.append(f"{prefix}{off:04x}: " + " ".join(f"{b:02x}" for b in chunk))
    return "\n".join(lines)


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("socket closed")
        buf += chunk
    return bytes(buf)


def recv_frame(sock: socket.socket) -> Tuple[int, bytes]:
    hdr = recv_exact(sock, 5)
    length = struct.unpack(">I", hdr[:4])[0]
    ftype = hdr[4]
    if length == 0:
        raise ValueError("invalid frame length=0")
    payload_len = length - 1
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return ftype, payload


def send_frame(sock: socket.socket, ftype: int, payload: bytes) -> None:
    length = 1 + len(payload)
    sock.sendall(struct.pack(">IB", length, ftype) + payload)


def parse_ptp_hdr(container: bytes) -> Tuple[int, int, int, int]:
    if len(container) < 12:
        raise ValueError("short container")
    length, ctype, code, tid = struct.unpack_from("<IHHI", container, 0)
    return length, ctype, code, tid


def read_container(ep_in, timeout_ms: int = 5000) -> bytes:
    first = ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    if len(first) < 12:
        raise RuntimeError(f"short read ({len(first)} bytes)")
    total_len = struct.unpack_from("<I", first, 0)[0]
    if total_len < 12:
        raise RuntimeError(f"invalid container length={total_len}")
    buf = bytearray(first)
    while len(buf) < total_len:
        buf += ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    return bytes(buf[:total_len])


@dataclass
class CameraUsb:
    dev: usb.core.Device
    cfg_value: int
    intf_num: int
    ep_in: object
    ep_out: object


def find_ptp_camera(vid: Optional[int], pid: Optional[int], pick: int) -> CameraUsb:
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
        raise RuntimeError("No USB Still Image (PTP) camera found.")
    if pick < 0 or pick >= len(matches):
        raise RuntimeError(f"--pick out of range (0..{len(matches)-1})")

    dev, cfg_value, intf_num = matches[pick]
    dev.set_configuration(cfg_value)
    cfg = dev.get_active_configuration()
    intf = usb.util.find_descriptor(cfg, bInterfaceNumber=intf_num)
    if intf is None:
        raise RuntimeError("interface not found")

    try:
        if dev.is_kernel_driver_active(intf_num):  # type: ignore[attr-defined]
            dev.detach_kernel_driver(intf_num)  # type: ignore[attr-defined]
    except Exception:
        pass

    usb.util.claim_interface(dev, intf_num)

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
        raise RuntimeError("bulk IN/OUT endpoints not found")

    return CameraUsb(dev=dev, cfg_value=cfg_value, intf_num=intf_num, ep_in=ep_in, ep_out=ep_out)


def main() -> int:
    ap = argparse.ArgumentParser(description="RS3 PTP logging proxy (ESP TCP <-> USB camera).")
    ap.add_argument("--esp-host", required=True, help="ESP32 IP (e.g. 192.168.1.91)")
    ap.add_argument("--esp-port", type=int, default=1235, help="ESP PTP proxy port (default: 1235)")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=None, help="Camera USB VID (e.g. 0x054c)")
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=None, help="Camera USB PID")
    ap.add_argument("--pick", type=int, default=0, help="Pick Nth PTP interface (default: 0)")
    ap.add_argument("--log", default=None, help="Write log to this file")
    args = ap.parse_args()

    log_f = open(args.log, "a", encoding="utf-8") if args.log else None

    def log(msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        if log_f:
            log_f.write(line + "\n")
            log_f.flush()

    log("Opening camera USB interface...")
    cam = find_ptp_camera(args.vid, args.pid, args.pick)
    log(f"Camera: VID=0x{int(cam.dev.idVendor):04x} PID=0x{int(cam.dev.idProduct):04x} if={cam.intf_num}")

    log(f"Connecting to ESP proxy {args.esp_host}:{args.esp_port} ...")
    sock = socket.create_connection((args.esp_host, args.esp_port), timeout=5)
    sock.settimeout(None)
    log("Connected.")

    try:
        while True:
            ftype, payload = recv_frame(sock)
            if ftype != T_CMD_STD:
                log(f"Unexpected frame type=0x{ftype:02x} (len={len(payload)})")
                continue

            cmd = payload
            try:
                clen, ctype, code, tid = parse_ptp_hdr(cmd)
            except Exception as e:
                log(f"Bad CMD_STD: {e}")
                continue

            log(f"RS3->CAM CMD: type=0x{ctype:04x} code=0x{code:04x} tid={tid} bytes={len(cmd)}")
            if log_f:
                log_f.write(hexdump(cmd, prefix="  ") + "\n")

            # Forward to camera
            cam.ep_out.write(cmd, timeout=5000)

            # Read containers until RESPONSE
            while True:
                cont = read_container(cam.ep_in, timeout_ms=5000)
                try:
                    alen, atype, acode, atid = parse_ptp_hdr(cont)
                except Exception as e:
                    log(f"Bad camera container: {e}")
                    break

                log(f"CAM->RS3 CONT: type=0x{atype:04x} code=0x{acode:04x} tid={atid} bytes={len(cont)}")
                if log_f:
                    log_f.write(hexdump(cont, prefix="  ") + "\n")
                    log_f.flush()

                send_frame(sock, T_CONT_STD, cont)

                if atype == PTP_CT_RESPONSE:
                    break

    except EOFError:
        log("ESP proxy disconnected.")
    finally:
        try:
            sock.close()
        except Exception:
            pass
        try:
            usb.util.release_interface(cam.dev, cam.intf_num)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(cam.dev)
        except Exception:
            pass
        if log_f:
            log_f.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


