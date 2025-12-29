#!/usr/bin/env python3
"""
Raw PTP proxy (no byte modifications on ESP side).

ESP (USB device to RS3) forwards raw BULK OUT packets to this script over TCP.
This script can:
  - Log the raw OUT packets (always),
  - Optionally forward them to a real camera over USB (best-effort),
  - Read camera BULK IN containers and send them back to ESP as raw BULK IN payloads.

Protocol (over TCP, same framing as ptp_proxy_server):
  uint32_be length (type + payload)
  uint8 type
  payload...

Types:
  0x10 RAW_OUT: ESP -> PC, payload is raw bytes from RS3 bulk OUT
  0x11 RAW_IN : PC  -> ESP, payload is raw bytes to send to RS3 bulk IN (len may be 0 => ZLP)
  0x12 RAW_DONE: PC -> ESP, end-of-reply marker for one RAW_OUT command (no payload)

Notes:
 - If you want a ZLP, explicitly send RAW_IN with empty payload.
 - Close Photos/Image Capture; you may need: sudo killall PTPCamera
 - You may need sudo for USB access on macOS.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from typing import Optional, Tuple

import usb.core
import usb.util


T_RAW_OUT = 0x10
T_RAW_IN = 0x11
T_RAW_DONE = 0x12

PTP_CLASS, PTP_SUBCLASS, PTP_PROTOCOL = 0x06, 0x01, 0x01
PTP_CT_RESPONSE = 3
PTP_CT_COMMAND = 1
PTP_CT_DATA = 2

# Response codes we care about (subset)
PTP_RC_OK = 0x2001
PTP_RC_SESSION_ALREADY_OPEN = 0x201E

# Operation codes (subset)
PTP_OC_OPEN_SESSION = 0x1002
PTP_OC_CLOSE_SESSION = 0x1003


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


def head8(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b[:8])


def send_raw_in_chunks(sock: socket.socket, payload: bytes, chunk_max: int, log) -> None:
    """
    Send RAW_IN payload to ESP split into <=chunk_max frames.
    This is required because ESP buffers one RAW_IN frame in a fixed-size slot (512 bytes by default).
    """
    if chunk_max <= 0:
        raise ValueError("chunk_max must be > 0")
    if len(payload) == 0:
        send_frame(sock, T_RAW_IN, b"")
        log("ESP<-PY RAW_IN send ZLP")
        return
    off = 0
    idx = 0
    while off < len(payload):
        part = payload[off : off + chunk_max]
        send_frame(sock, T_RAW_IN, part)
        log(f"ESP<-PY RAW_IN send chunk[{idx}] bytes={len(part)} head={head8(part)}")
        off += len(part)
        idx += 1

def _le32(b: bytes, off: int = 0) -> int:
    return struct.unpack_from("<I", b, off)[0]

def _le16(b: bytes, off: int = 0) -> int:
    return struct.unpack_from("<H", b, off)[0]

def parse_ptp_container_header(b: bytes) -> Tuple[int, int, int, int]:
    if len(b) < 12:
        raise ValueError("short PTP container")
    total_len = _le32(b, 0)
    ctype = _le16(b, 4)
    code = _le16(b, 6)
    tid = _le32(b, 8)
    return total_len, ctype, code, tid

def read_ptp_container(ep_in, timeout_ms: int = 5000) -> bytes:
    """
    Read one full PTP container from bulk IN using the <len32,...> header.
    """
    first = ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    if len(first) < 12:
        raise RuntimeError(f"short read ({len(first)} bytes)")
    total_len = _le32(first, 0)
    if total_len < 12:
        raise RuntimeError(f"invalid PTP length={total_len}")
    buf = bytearray(first)
    while len(buf) < total_len:
        buf += ep_in.read(ep_in.wMaxPacketSize, timeout=timeout_ms).tobytes()
    return bytes(buf[:total_len])

def parse_rs3_command(payload: bytes) -> Tuple[str, int, int, int, bytes]:
    """
    Parse RS3/DJI command variants we observed and return:
      (layout, type, code, tid, params_bytes)
    layouts:
      - "dji_pad24": 00 00 00 [type16le@3][code16le@5][tid32le@7][params...]
      - "dji_pad16": 00 00 [type16le@2][code16le@4][tid32le@6][params...]
      - "dji_pad8" : 00 [type16le@1][code16le@3][tid32le@5][params...]
      - "std_len"  : [len32][type16le][code16le][tid32le][params...]
      - "alt_len"  : [len32][code16le][tid32le][type16le][params...]
    """
    b = payload
    n = len(b)
    def params_aligned(off: int) -> bytes:
        # RS3/DJI sometimes appends extra padding bytes (e.g. an extra 0x01) after tid.
        # For PTP standard command params must be 32-bit aligned. Keep only full uint32 params.
        if n <= off:
            return b""
        tail = b[off:]
        return tail[: (len(tail) // 4) * 4]

    if n >= 11 and b[0] == 0x00 and b[1] == 0x00 and b[2] == 0x00:
        ctype = _le16(b, 3)
        code = _le16(b, 5)
        tid = _le32(b, 7)
        return "dji_pad24", ctype, code, tid, params_aligned(11)
    if n >= 10 and b[0] == 0x00 and b[1] == 0x00:
        ctype = _le16(b, 2)
        code = _le16(b, 4)
        tid = _le32(b, 6)
        return "dji_pad16", ctype, code, tid, params_aligned(10)
    if n >= 9 and b[0] == 0x00:
        ctype = _le16(b, 1)
        code = _le16(b, 3)
        tid = _le32(b, 5)
        return "dji_pad8", ctype, code, tid, params_aligned(9)
    if n >= 12:
        length = _le32(b, 0)
        t_std = _le16(b, 4)
        if 1 <= t_std <= 4:
            code = _le16(b, 6)
            tid = _le32(b, 8)
            # params are in remaining bytes (ignore length mismatch; we just pass through)
            return "std_len", t_std, code, tid, params_aligned(12)
        t_alt = _le16(b, 10)
        if 1 <= t_alt <= 4:
            code = _le16(b, 4)
            tid = _le32(b, 6)
            return "alt_len", t_alt, code, tid, params_aligned(12)
    raise ValueError("unknown RS3 command layout")


def parse_rs3_container(payload: bytes, *, align_tail_u32: bool) -> Tuple[str, int, int, int, bytes]:
    """
    Parse RS3/DJI container variants and return:
      (layout, type, code, tid, tail_bytes)
    If align_tail_u32=True, tail is truncated to full uint32 params (PTP COMMAND params).
    If align_tail_u32=False, tail is returned raw (PTP DATA payload).
    """
    b = payload
    n = len(b)

    def tail(off: int) -> bytes:
        if n <= off:
            return b""
        t = b[off:]
        if not align_tail_u32:
            return t
        return t[: (len(t) // 4) * 4]

    if n >= 11 and b[0] == 0x00 and b[1] == 0x00 and b[2] == 0x00:
        ctype = _le16(b, 3)
        code = _le16(b, 5)
        tid = _le32(b, 7)
        return "dji_pad24", ctype, code, tid, tail(11)
    if n >= 10 and b[0] == 0x00 and b[1] == 0x00:
        ctype = _le16(b, 2)
        code = _le16(b, 4)
        tid = _le32(b, 6)
        return "dji_pad16", ctype, code, tid, tail(10)
    if n >= 9 and b[0] == 0x00:
        ctype = _le16(b, 1)
        code = _le16(b, 3)
        tid = _le32(b, 5)
        return "dji_pad8", ctype, code, tid, tail(9)
    if n >= 12:
        length = _le32(b, 0)
        t_std = _le16(b, 4)
        if 1 <= t_std <= 4:
            code = _le16(b, 6)
            tid = _le32(b, 8)
            return "std_len", t_std, code, tid, tail(12)
        t_alt = _le16(b, 10)
        if 1 <= t_alt <= 4:
            code = _le16(b, 4)
            tid = _le32(b, 6)
            return "alt_len", t_alt, code, tid, tail(12)
    raise ValueError("unknown RS3 container layout")

def build_std_command_container(code: int, tid: int, params_bytes: bytes) -> bytes:
    # Standard PTP command: len32,type16,code16,tid32 + params (4-byte LE)
    total_len = 12 + len(params_bytes)
    return struct.pack("<IHHI", total_len, PTP_CT_COMMAND, code & 0xFFFF, tid & 0xFFFFFFFF) + params_bytes


def build_std_data_container(code: int, tid: int, data_bytes: bytes) -> bytes:
    total_len = 12 + len(data_bytes)
    return struct.pack("<IHHI", total_len, PTP_CT_DATA, code & 0xFFFF, tid & 0xFFFFFFFF) + data_bytes


def _read_until_response(ep_in, timeout_ms: int = 1200) -> Optional[bytes]:
    """
    Best-effort read: try to read one or more PTP containers until we see RESPONSE (type=3).
    Returns the RESPONSE container bytes or None.
    """
    deadline = time.time() + (timeout_ms / 1000.0)
    while time.time() < deadline:
        try:
            cont = read_ptp_container(ep_in, timeout_ms=int(max(100, (deadline - time.time()) * 1000)))
        except Exception:
            return None
        try:
            _, ctype, _, _ = parse_ptp_container_header(cont)
        except Exception:
            continue
        if ctype == PTP_CT_RESPONSE:
            return cont
    return None


def best_effort_close_camera_session(ep_out, ep_in, log) -> None:
    """
    Many Sony cameras keep a PTP session open and respond SessionAlreadyOpen (0x201E) to OpenSession.
    Try to close any stale session so you don't need to power-cycle the camera.
    """
    # 1) Try CloseSession directly.
    try:
        ep_out.write(build_std_command_container(PTP_OC_CLOSE_SESSION, tid=1, params_bytes=b""), timeout=1000)
        resp = _read_until_response(ep_in, timeout_ms=1200)
        if resp:
            _, _, code, _ = parse_ptp_container_header(resp)
            log(f"Camera preflight: CloseSession resp=0x{code:04x}")
            if code == PTP_RC_OK:
                return
    except Exception as e:
        log(f"Camera preflight: CloseSession failed: {e}")

    # 2) Try OpenSession (sid=1), then CloseSession.
    try:
        sid = struct.pack("<I", 1)
        ep_out.write(build_std_command_container(PTP_OC_OPEN_SESSION, tid=2, params_bytes=sid), timeout=1000)
        resp = _read_until_response(ep_in, timeout_ms=1200)
        if resp:
            _, _, code, _ = parse_ptp_container_header(resp)
            log(f"Camera preflight: OpenSession resp=0x{code:04x}")
    except Exception as e:
        log(f"Camera preflight: OpenSession failed: {e}")

    try:
        ep_out.write(build_std_command_container(PTP_OC_CLOSE_SESSION, tid=3, params_bytes=b""), timeout=1000)
        resp = _read_until_response(ep_in, timeout_ms=1200)
        if resp:
            _, _, code, _ = parse_ptp_container_header(resp)
            log(f"Camera preflight: CloseSession(2) resp=0x{code:04x}")
    except Exception as e:
        log(f"Camera preflight: CloseSession(2) failed: {e}")

def build_rs3_container(layout: str, ctype: int, code: int, tid: int, payload: bytes) -> bytes:
    """
    Build RS3-side container using the same layout we received commands with.
    For DATA/RESP from camera, we use (ctype, code, tid) from camera header and payload bytes.
    """
    if layout == "dji_pad24":
        hdr = b"\x00\x00\x00" + struct.pack("<HHI", ctype & 0xFFFF, code & 0xFFFF, tid & 0xFFFFFFFF)
        out = hdr + payload
        # For RESPONSE (no payload), RS3 may expect >= 12 bytes (std header size). Pad with zeros to 12.
        if ctype == PTP_CT_RESPONSE and len(payload) == 0 and len(out) < 12:
            out += b"\x00" * (12 - len(out))
        return out
    if layout == "dji_pad16":
        hdr = b"\x00\x00" + struct.pack("<HHI", ctype & 0xFFFF, code & 0xFFFF, tid & 0xFFFFFFFF)
        out = hdr + payload
        if ctype == PTP_CT_RESPONSE and len(payload) == 0 and len(out) < 12:
            out += b"\x00" * (12 - len(out))
        return out
    if layout == "dji_pad8":
        hdr = b"\x00" + struct.pack("<HHI", ctype & 0xFFFF, code & 0xFFFF, tid & 0xFFFFFFFF)
        out = hdr + payload
        if ctype == PTP_CT_RESPONSE and len(payload) == 0 and len(out) < 12:
            out += b"\x00" * (12 - len(out))
        return out
    if layout == "alt_len":
        total_len = 12 + len(payload)
        return struct.pack("<IHIH", total_len, code & 0xFFFF, tid & 0xFFFFFFFF, ctype & 0xFFFF) + payload
    # std_len default
    total_len = 12 + len(payload)
    return struct.pack("<IHHI", total_len, ctype & 0xFFFF, code & 0xFFFF, tid & 0xFFFFFFFF) + payload


def find_camera(vid: Optional[int], pid: Optional[int], pick: int):
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
        return None
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
        raise RuntimeError("bulk endpoints not found")
    return dev, intf_num, int(ep_in.bEndpointAddress), int(ep_out.bEndpointAddress), ep_in, ep_out


def main() -> int:
    ap = argparse.ArgumentParser(description="Raw RS3 PTP proxy (ESP <-> PC), optional camera forward.")
    ap.add_argument("--esp-host", required=True)
    ap.add_argument("--esp-port", type=int, default=1235)
    ap.add_argument("--log", default="/tmp/rs3_ptp_raw_proxy.log")
    ap.add_argument("--camera", action="store_true", help="Forward packets to a real camera over USB (best-effort).")
    ap.add_argument("--translate", action="store_true",
                    help="Translate RS3/DJI command layouts to standard PTP for the camera, "
                         "and translate camera containers back to RS3 layout. RAW is still logged 1:1.")
    ap.add_argument("--rs3-in-layout", choices=["auto", "dji_pad8", "dji_pad16", "dji_pad24", "std_len", "alt_len", "camera"],
                    default="auto",
                    help="(Translate mode) Force RS3-facing IN container layout.\n"
                         "  - auto: use the layout detected from the RS3 command (default)\n"
                         "  - dji_pad8/16/24: DJI padded header without length field\n"
                         "  - std_len: standard PTP container (len32,type16,code16,tid32, payload)\n"
                         "  - alt_len: alternate observed container (len32,code16,tid32,type16, payload)\n"
                         "  - camera: send the camera's standard container bytes unchanged (no repack)\n")
    ap.add_argument("--rs3-resp-pad01", action="store_true",
                    help="(Translate mode) Add a single 0x01 byte padding after RS3 response header before zero-padding to 12 bytes. "
                         "Some RS3 firmwares appear to expect this quirk.")
    ap.add_argument("--no-zlp", action="store_true",
                    help="Do not send an explicit RAW_IN ZLP frame after IN payloads whose length is a multiple of 64.")
    ap.add_argument("--rs3-in-chunk", type=int, default=512,
                    help="Max RAW_IN payload chunk size to send to ESP (ESP buffers per-frame; default 512).")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--pick", type=int, default=0)
    args = ap.parse_args()

    log_f = open(args.log, "a", encoding="utf-8")

    def log(msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        log_f.write(line + "\n")
        log_f.flush()

    cam = None  # tuple: (dev, ifnum, ep_in_addr, ep_out_addr, ep_in, ep_out)
    if args.camera:
        log("Opening camera USB interface...")
        cam = find_camera(args.vid, args.pid, args.pick)
        if cam is None:
            log("No camera found; continuing in log-only mode.")
        else:
            dev, ifnum, ep_in_addr, ep_out_addr, ep_in, ep_out = cam
            log(f"Camera: VID=0x{int(dev.idVendor):04x} PID=0x{int(dev.idProduct):04x} if={ifnum} ep_in=0x{ep_in_addr:02x} ep_out=0x{ep_out_addr:02x}")
            best_effort_close_camera_session(ep_out, ep_in, log)

    log(f"Connecting to ESP raw proxy {args.esp_host}:{args.esp_port} ...")
    sock = socket.create_connection((args.esp_host, args.esp_port), timeout=5)
    sock.settimeout(None)
    log("Connected.")

    # Some PTP operations are 2-stage: COMMAND (host->device) then DATA (host->device),
    # then the camera replies with DATA/RESPONSE. Remember the last command so we can
    # wrap the RS3 DATA stage into a standard PTP DATA container for the camera.
    pending_cam_op: Optional[int] = None
    pending_cam_tid: Optional[int] = None
    pending_rs3_layout: Optional[str] = None

    try:
        while True:
            ftype, payload = recv_frame(sock)
            if ftype != T_RAW_OUT:
                log(f"Unexpected frame type=0x{ftype:02x} len={len(payload)}")
                continue

            log(f"RS3->ESP RAW_OUT bytes={len(payload)}")
            log_f.write(hexdump(payload, prefix="  ") + "\n")
            log_f.flush()

            if cam is None:
                continue

            dev, ifnum, ep_in_addr, ep_out_addr, ep_in, ep_out = cam

            if len(payload) >= 4:
                log(f"RAW_OUT head: {payload[:8].hex(' ')}")

            cam_out = payload
            rs3_layout = None
            last_op = None
            rs3_stage = None  # "cmd" | "data" | None
            if args.translate:
                try:
                    rs3_layout, ctype, code, tid, tail = parse_rs3_container(payload, align_tail_u32=True)
                except Exception as e:
                    log(f"Translate: cannot parse RS3 container: {e}")
                    send_frame(sock, T_RAW_DONE, b"")
                    log("ESP<-PY RAW_DONE")
                    continue

                if ctype == PTP_CT_COMMAND:
                    cam_out = build_std_command_container(code, tid, tail)
                    last_op = code
                    pending_cam_op = code
                    pending_cam_tid = tid
                    pending_rs3_layout = rs3_layout
                    rs3_stage = "cmd"
                    log(f"Translate: {rs3_layout} -> std_len CMD op=0x{code:04x} tid={tid} bytes={len(cam_out)}")
                elif ctype == PTP_CT_DATA:
                    # Re-parse without alignment to preserve DATA payload bytes.
                    rs3_layout, _, code2, tid2, data_tail = parse_rs3_container(payload, align_tail_u32=False)
                    op = pending_cam_op if pending_cam_op is not None else code2
                    tid_use = pending_cam_tid if pending_cam_tid is not None else tid2
                    rs3_layout = pending_rs3_layout if pending_rs3_layout is not None else rs3_layout
                    cam_out = build_std_data_container(op, tid_use, data_tail)
                    last_op = op
                    rs3_stage = "data"
                    log(f"Translate: {rs3_layout} -> std_len DATA op=0x{op:04x} tid={tid_use} bytes={len(cam_out)} payload={len(data_tail)}")
                else:
                    log(f"Translate: ignoring container type={ctype}")
                    send_frame(sock, T_RAW_DONE, b"")
                    log("ESP<-PY RAW_DONE")
                    continue

            # Forward bytes to camera bulk OUT.
            try:
                ep_out.write(cam_out, timeout=2000)
            except Exception as e:
                log(f"Camera write failed: {e}")
                try:
                    dev.clear_halt(ep_out_addr)
                    log("Cleared camera OUT halt.")
                except Exception:
                    pass
                send_frame(sock, T_RAW_DONE, b"")
                log("ESP<-PY RAW_DONE")
                continue

            # IMPORTANT:
            # For operations that require host->device DATA, the camera will not reply until it receives DATA.
            # We must NOT block for seconds after COMMAND, because that prevents ESP from re-arming the OUT endpoint
            # and RS3 cannot send the DATA stage. So after COMMAND we only probe briefly; on timeout we send RAW_DONE
            # immediately and wait for the next RS3 OUT (DATA stage).
            cam_read_timeout_ms = 4000
            if rs3_stage == "cmd":
                cam_read_timeout_ms = 200

            # Read full PTP containers and relay them back as RAW_IN.
            # Stop when we get RESPONSE (type=3). Also send ZLP to RS3 if container size is multiple of 64.
            for _ in range(8):
                try:
                    cont = read_ptp_container(ep_in, timeout_ms=cam_read_timeout_ms)
                except usb.core.USBError as e:
                    # After some commands the camera expects a host->device DATA stage.
                    # Treat timeouts as "waiting for DATA", not as an error.
                    msg = str(e).lower()
                    if "timed out" in msg or "timeout" in msg:
                        if rs3_stage == "cmd":
                            log(f"Camera awaiting RS3 DATA stage (no reply after CMD): {e}")
                            # Let ESP re-arm OUT so RS3 can send DATA.
                            send_frame(sock, T_RAW_DONE, b"")
                            log("ESP<-PY RAW_DONE")
                            break
                        log(f"Camera read timeout: {e}")
                        send_frame(sock, T_RAW_DONE, b"")
                        log("ESP<-PY RAW_DONE")
                        break
                    log(f"Camera read failed: {e}")
                    try:
                        dev.clear_halt(ep_in_addr)
                        log("Cleared camera IN halt.")
                    except Exception:
                        pass
                    send_frame(sock, T_RAW_DONE, b"")
                    log("ESP<-PY RAW_DONE")
                    break
                except Exception as e:
                    log(f"Camera read failed: {e}")
                    send_frame(sock, T_RAW_DONE, b"")
                    log("ESP<-PY RAW_DONE")
                    break

                try:
                    total_len, ctype, code, tid = parse_ptp_container_header(cont)
                except Exception:
                    total_len, ctype, code, tid = (len(cont), -1, -1, -1)

                log(f"CAM->RS3 RAW_IN PTP bytes={len(cont)} type={ctype} code=0x{code:04x} tid={tid}")
                log_f.write(hexdump(cont, prefix="  ") + "\n")
                log_f.flush()
                out_bytes = cont
                if args.translate and rs3_layout is not None:
                    # Convert standard camera container to RS3-side format.
                    payload_bytes = cont[12:]
                    send_code = code
                    # Some cameras reply SessionAlreadyOpen to OpenSession if a previous session exists.
                    # RS3 expects OpenSession OK; treat this as OK on the RS3-facing side to proceed.
                    if ctype == PTP_CT_RESPONSE and last_op == 0x1002 and code == PTP_RC_SESSION_ALREADY_OPEN:
                        send_code = PTP_RC_OK
                        # IMPORTANT: camera may include response parameters (len=16). RS3 expects OpenSession OK
                        # without response parameters, so drop them when overriding.
                        payload_bytes = b""
                        log("Translate: overriding OpenSession SessionAlreadyOpen -> OK for RS3")
                    # Decide which RS3-facing format to send.
                    rs3_reply_layout = rs3_layout
                    if args.rs3_in_layout != "auto":
                        rs3_reply_layout = args.rs3_in_layout

                    if rs3_reply_layout == "camera":
                        # Send camera bytes as-is (but still allow overriding response code, e.g. OpenSession).
                        if send_code == code:
                            out_bytes = cont
                        else:
                            # Rebuild standard container with modified response code.
                            out_bytes = build_rs3_container("std_len", ctype, send_code, tid, payload_bytes)
                    else:
                        out_bytes = build_rs3_container(rs3_reply_layout, ctype, send_code, tid, payload_bytes)
                    # Optional quirk: add 0x01 after tid for RESPONSE only (before zero padding)
                    if args.rs3_resp_pad01 and ctype == PTP_CT_RESPONSE and len(payload_bytes) == 0:
                        # Insert after header (layout-specific header size)
                        if rs3_reply_layout == "dji_pad24":
                            hsz = 11
                        elif rs3_reply_layout == "dji_pad16":
                            hsz = 10
                        elif rs3_reply_layout == "dji_pad8":
                            hsz = 9
                        else:
                            hsz = None
                        if hsz is not None and len(out_bytes) >= hsz:
                            out_bytes = out_bytes[:hsz] + b"\x01" + out_bytes[hsz:]
                            if len(out_bytes) < 12:
                                out_bytes += b"\x00" * (12 - len(out_bytes))
                    log(f"Translate: std -> {rs3_reply_layout} bytes={len(out_bytes)}")

                # Send camera->RS3 bytes via ESP. Chunk if needed (ESP buffers per RAW_IN frame).
                send_raw_in_chunks(sock, out_bytes, args.rs3_in_chunk, log)

                # ZLP decision must be based on what RS3 actually receives (out_bytes).
                if (not args.no_zlp) and (len(out_bytes) % 64) == 0:
                    send_frame(sock, T_RAW_IN, b"")
                    log("ESP<-PY RAW_IN send ZLP")

                if ctype == PTP_CT_RESPONSE:
                    pending_cam_op = None
                    pending_cam_tid = None
                    pending_rs3_layout = None
                    send_frame(sock, T_RAW_DONE, b"")
                    log("ESP<-PY RAW_DONE")
                    break

    except EOFError:
        log("ESP disconnected.")
    finally:
        try:
            sock.close()
        except Exception:
            pass
        if cam is not None:
            dev, ifnum, ep_in_addr, ep_out_addr, ep_in, ep_out = cam
            try:
                usb.util.release_interface(dev, ifnum)
            except Exception:
                pass
            try:
                usb.util.dispose_resources(dev)
            except Exception:
                pass
        log_f.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


