#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Fake ISO-TP ECU for bench-testing the Mini-VCI driver.

Speaks the CH340 "USB-CAN Analyzer" serial protocol directly (same framing as
kobolt/usb-can), so no SocketCAN is needed.

Listens on a request CAN id and answers on a response id:

    21 43  -> single-frame reply  61 43 <target> <actual>   (mimics a CAN A/C
              amplifier's air-mix servo data list)
    21 44  -> multi-frame reply, long enough to force First Frame + Flow
              Control + Consecutive Frames, which exercises the driver's
              reassembly path and makes the adapter's flow control visible.

Anything else gets a negative response 7F 21 11 (serviceNotSupported).

Usage:  ./isotp_responder.py [tty] [--verbose]
"""
import sys
import time

import serial

TTY = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
TTY_BAUD = 2000000
CAN_SPEED_500K = 0x03

REQ_ID = 0x7C4          # what the tester sends to
RSP_ID = 0x7CC          # what we answer on

verbose = False


def log(*a):
    print(*a, flush=True)


def open_adapter(path):
    ser = serial.Serial(path, TTY_BAUD, timeout=0.05)
    cmd = bytearray(20)
    cmd[0], cmd[1], cmd[2] = 0xAA, 0x55, 0x12
    cmd[3] = CAN_SPEED_500K
    cmd[4] = 0x01           # standard 11-bit frames
    # bytes 5..12 are filter and mask, left zero = accept everything
    cmd[13] = 0x00          # normal mode (not loopback/silent)
    cmd[14] = 0x01
    cmd[19] = sum(cmd[2:19]) & 0xFF
    ser.write(cmd)
    ser.flush()
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def send(ser, can_id, data):
    """One standard CAN frame, zero-padded to 8 like a real ECU."""
    payload = bytes(data) + b"\x00" * (8 - len(data))
    frame = bytes([0xAA, 0xC0 | 8, can_id & 0xFF, (can_id >> 8) & 0xFF]) + payload + b"\x55"
    ser.write(frame)
    ser.flush()
    if verbose:
        log(f"    TX {can_id:03x}  {payload.hex(' ')}")


def read_frame(ser):
    """Return (can_id, data) or None. Resyncs on 0xAA."""
    b = ser.read(1)
    if not b or b[0] != 0xAA:
        return None
    hdr = ser.read(1)
    if not hdr:
        return None
    if hdr[0] == 0x55:                      # command/status frame
        ser.read(18)
        return None
    if (hdr[0] >> 4) != 0xC:
        return None
    dlc = hdr[0] & 0x0F
    rest = ser.read(2 + dlc + 1)
    if len(rest) < 2 + dlc + 1:
        return None
    can_id = rest[0] | (rest[1] << 8)
    return can_id, rest[2:2 + dlc]


def wait_flow_control(ser, timeout=1.0):
    """The tester must answer a First Frame with 30 <bs> <stmin>."""
    end = time.time() + timeout
    while time.time() < end:
        got = read_frame(ser)
        if not got:
            continue
        can_id, data = got
        if can_id == REQ_ID and data and (data[0] & 0xF0) == 0x30:
            fs, bs, stmin = data[0] & 0x0F, data[1], data[2]
            log(f"  <- flow control: FS={fs} BS={bs} STmin={stmin}")
            return bs, stmin
    log("  !! no flow control within 1s")
    return None


def send_multiframe(ser, payload):
    """First Frame, wait for FC, then Consecutive Frames."""
    log(f"  -> multi-frame reply, {len(payload)} bytes")
    send(ser, RSP_ID, [0x10 | ((len(payload) >> 8) & 0x0F), len(payload) & 0xFF] + list(payload[:6]))

    fc = wait_flow_control(ser)
    if fc is None:
        return
    _, stmin = fc

    sn, off = 1, 6
    while off < len(payload):
        chunk = payload[off:off + 7]
        send(ser, RSP_ID, [0x20 | (sn & 0x0F)] + list(chunk))
        off += len(chunk)
        sn += 1
        time.sleep(max(stmin, 1) / 1000.0)
    log(f"  -> sent {sn - 1} consecutive frames")


def handle(ser, data):
    pci_type = data[0] & 0xF0
    if pci_type != 0x00:
        return                              # only single-frame requests here
    length = data[0] & 0x0F
    req = data[1:1 + length]
    log(f"  <- request {req.hex(' ')}")

    if len(req) >= 2 and req[0] == 0x21 and req[1] == 0x43:
        # Air-mix servo: byte0 commanded, byte1 actual. Plausible mid-travel.
        send(ser, RSP_ID, [0x04, 0x61, 0x43, 0x7B, 0x79])
        log("  -> 61 43 7b 79  (single frame)")
    elif len(req) >= 2 and req[0] == 0x21 and req[1] == 0x44:
        body = bytes([0x61, 0x44]) + bytes((i * 3) & 0xFF for i in range(38))
        send_multiframe(ser, body)
    else:
        send(ser, RSP_ID, [0x03, 0x7F, 0x21, 0x11])
        log("  -> 7f 21 11 (serviceNotSupported)")


def main():
    global verbose
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    path = args[0] if args else TTY

    ser = open_adapter(path)
    log(f"responder up on {path}: listening {REQ_ID:03x}, answering {RSP_ID:03x}")
    log("(21 43 -> single frame, 21 44 -> multi-frame, else negative response)")

    try:
        while True:
            got = read_frame(ser)
            if not got:
                continue
            can_id, data = got
            if can_id == REQ_ID and data:
                if (data[0] & 0xF0) == 0x30:
                    continue                # stray flow control, ignore
                handle(ser, data)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
