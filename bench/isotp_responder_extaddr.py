#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Fake ISO-TP ECU with EXTENDED/MIXED ADDRESSING, for bench-testing the
Mini-VCI driver's extended-addressing path (rmvci `IsoTpConfig::with_ext_addr`
on both `IsoTpPath`s).

Same CH340 "USB-CAN Analyzer" serial framing as `isotp_responder.py`, but every
ISO-TP frame carries the address byte FIRST, ahead of the PCI:

    wire CAN payload = [ADDR] [PCI ...] [data ...]

so a single frame holds ≤ 6 data bytes, a first frame 5, a consecutive frame 6.
The driver uses one address byte for both directions (N_TA extension), so this
responder both expects requests led by ADDR and answers led by ADDR.

    21 43  -> single-frame reply  ADDR 61 43 <target> <actual>
    21 44  -> multi-frame reply (FF + FC + CFs), all frames led by ADDR
    else   -> negative response  ADDR 7F 21 11

This is the layout derived from the firmware (address at msg[4], payload at
msg[5]); running it against the cable is what settles whether the firmware
leaves the address byte at the head of the reassembled reply (rmvci
`IsoTp::recv` on the firmware path strips it when RxStatus 0x80 is set).

Usage:  ./isotp_responder_extaddr.py [tty] [--addr 0xNN] [--verbose]
"""
import sys
import time

import serial

TTY = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
TTY_BAUD = 2000000
CAN_SPEED_500K = 0x03

REQ_ID = 0x7C4          # what the tester sends to
RSP_ID = 0x7CC          # what we answer on
ADDR = 0xF1             # extended-addressing byte (both directions)

verbose = False


def log(*a):
    print(*a, flush=True)


def open_adapter(path):
    ser = serial.Serial(path, TTY_BAUD, timeout=0.05)
    cmd = bytearray(20)
    cmd[0], cmd[1], cmd[2] = 0xAA, 0x55, 0x12
    cmd[3] = CAN_SPEED_500K
    cmd[4] = 0x01           # standard 11-bit frames
    cmd[13] = 0x00          # normal mode
    cmd[14] = 0x01
    cmd[19] = sum(cmd[2:19]) & 0xFF
    ser.write(cmd)
    ser.flush()
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def send(ser, can_id, isotp):
    """One CAN frame carrying [ADDR] + isotp bytes, zero-padded to 8."""
    data = bytes([ADDR]) + bytes(isotp)
    payload = data + b"\x00" * (8 - len(data))
    frame = bytes([0xAA, 0xC0 | 8, can_id & 0xFF, (can_id >> 8) & 0xFF]) + payload + b"\x55"
    ser.write(frame)
    ser.flush()
    if verbose:
        log(f"    TX {can_id:03x}  {payload.hex(' ')}")


def read_frame(ser):
    """Return (can_id, data) or None. `data` includes the address byte."""
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
    """The tester answers a First Frame with ADDR 30 <bs> <stmin>."""
    end = time.time() + timeout
    while time.time() < end:
        got = read_frame(ser)
        if not got:
            continue
        can_id, data = got
        if can_id == REQ_ID and len(data) >= 4 and data[0] == ADDR and (data[1] & 0xF0) == 0x30:
            fs, bs, stmin = data[1] & 0x0F, data[2], data[3]
            log(f"  <- flow control: FS={fs} BS={bs} STmin={stmin}")
            return bs, stmin
    log("  !! no flow control within 1s")
    return None


def send_multiframe(ser, payload):
    """First Frame (5 data bytes under ext addr), wait for FC, then CFs (6 each)."""
    log(f"  -> multi-frame reply, {len(payload)} bytes")
    send(ser, RSP_ID, [0x10 | ((len(payload) >> 8) & 0x0F), len(payload) & 0xFF] + list(payload[:5]))

    fc = wait_flow_control(ser)
    if fc is None:
        return
    _, stmin = fc

    sn, off = 1, 5
    while off < len(payload):
        chunk = payload[off:off + 6]
        send(ser, RSP_ID, [0x20 | (sn & 0x0F)] + list(chunk))
        off += len(chunk)
        sn += 1
        time.sleep(max(stmin, 1) / 1000.0)
    log(f"  -> sent {sn - 1} consecutive frames")


def handle(ser, data):
    # data[0] is the address byte; the PCI follows.
    if len(data) < 2 or data[0] != ADDR:
        return
    if (data[1] & 0xF0) != 0x00:
        return                              # only single-frame requests here
    length = data[1] & 0x0F
    req = data[2:2 + length]
    log(f"  <- request {req.hex(' ')}  (addr {ADDR:02x})")

    if len(req) >= 2 and req[0] == 0x21 and req[1] == 0x43:
        send(ser, RSP_ID, [0x04, 0x61, 0x43, 0x7B, 0x79])
        log(f"  -> {ADDR:02x} 04 61 43 7b 79  (single frame)")
    elif len(req) >= 2 and req[0] == 0x21 and req[1] == 0x44:
        body = bytes([0x61, 0x44]) + bytes((i * 3) & 0xFF for i in range(38))
        send_multiframe(ser, body)
    else:
        send(ser, RSP_ID, [0x03, 0x7F, 0x21, 0x11])
        log("  -> 7f 21 11 (serviceNotSupported)")


def main():
    global verbose, ADDR
    argv = sys.argv[1:]
    verbose = "--verbose" in argv or "-v" in argv
    if "--addr" in argv:
        ADDR = int(argv[argv.index("--addr") + 1], 0)
    pos = [a for a in argv if not a.startswith("-")]
    # drop the --addr value from positionals if it slipped in
    path = pos[0] if pos and pos[0] != hex(ADDR) else TTY

    ser = open_adapter(path)
    log(f"ext-addr responder up on {path}: listening {REQ_ID:03x}, answering {RSP_ID:03x}, addr {ADDR:02x}")
    log("(21 43 -> single frame, 21 44 -> multi-frame, else negative response)")

    try:
        while True:
            got = read_frame(ser)
            if not got:
                continue
            can_id, data = got
            if can_id == REQ_ID and len(data) >= 2 and data[0] == ADDR:
                if (data[1] & 0xF0) == 0x30:
                    continue                # stray flow control, ignore
                handle(ser, data)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
