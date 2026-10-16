#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Characterise an ISO-TP *sender's* flow-control behaviour.

The companion to isotp_responder.py. That one answers requests, so it
exercises the driver's receive path. This one is the mirror: it makes the
device under test send us a multi-frame message, answers with a Flow Control
frame advertising a specific block size and STmin, and then measures whether
the sender actually obeyed them.

This tests the firmware-analysis claim that the Mini-VCI firmware's ISO-TP
transmit path ignores both BS and STmin. Run it against the cable's own ISO15765 (protocol 6) and
against a host-side implementation to compare.

What it reports, per run:

    BS      how many consecutive frames arrived before the sender paused for
            another Flow Control. Advertised BS=2 but 8 arrived -> ignored.
    STmin   the observed inter-frame gaps against what we asked for.
            Asked 50 ms, saw 1.1 ms -> ignored.

Usage:  ./isotp_fc_probe.py [tty] [--bs N] [--stmin MS] [--label TEXT]
"""
import sys
import time

import serial

TTY = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
TTY_BAUD = 2000000
CAN_SPEED_500K = 0x03

REQ_ID = 0x7C4          # what the device under test sends to
RSP_ID = 0x7CC          # what we answer on

# How long to keep collecting consecutive frames after the last one before
# deciding the sender has paused (or finished). Must be well above the
# advertised STmin to avoid calling a legitimate pause a stall.
QUIET_S = 0.40


def log(*a):
    print(*a, flush=True)


def open_adapter(path):
    ser = serial.Serial(path, TTY_BAUD, timeout=0.002)
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


def send(ser, can_id, data):
    payload = bytes(data) + b"\x00" * (8 - len(data))
    frame = bytes([0xAA, 0xC0 | 8, can_id & 0xFF, (can_id >> 8) & 0xFF]) + payload + b"\x55"
    ser.write(frame)
    ser.flush()


def read_frame(ser):
    """Return (timestamp, can_id, data) or None. Resyncs on 0xAA."""
    b = ser.read(1)
    if not b or b[0] != 0xAA:
        return None
    t = time.monotonic()
    hdr = ser.read(1)
    if not hdr:
        return None
    if hdr[0] == 0x55:
        ser.read(18)
        return None
    if (hdr[0] >> 4) != 0xC:
        return None
    dlc = hdr[0] & 0x0F
    rest = ser.read(2 + dlc + 1)
    if len(rest) < 2 + dlc + 1:
        return None
    return t, rest[0] | (rest[1] << 8), rest[2:2 + dlc]


def collect_block(ser, deadline):
    """Consecutive frames until the sender goes quiet for QUIET_S."""
    frames = []
    last = time.monotonic()
    while time.monotonic() < deadline:
        got = read_frame(ser)
        if got is None:
            if frames and (time.monotonic() - last) > QUIET_S:
                break
            continue
        t, can_id, data = got
        if can_id != REQ_ID or not data:
            continue
        if (data[0] & 0xF0) == 0x20:
            frames.append((t, data))
            last = t
        elif (data[0] & 0xF0) == 0x10:
            # A new First Frame: the previous transfer is over.
            return frames, data
    return frames, None


def characterise(ser, bs, stmin_ms, label):
    log(f"\n=== waiting for a multi-frame message from the sender ({label}) ===")
    log(f"    advertising BS={bs} STmin={stmin_ms} ms")

    # --- wait for the First Frame ---
    ff = None
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        got = read_frame(ser)
        if got is None:
            continue
        _, can_id, data = got
        if can_id == REQ_ID and data and (data[0] & 0xF0) == 0x10:
            ff = data
            break
    if ff is None:
        log("!! no First Frame within 60 s")
        return None

    ff_dl = ((ff[0] & 0x0F) << 8) | ff[1]
    log(f"    First Frame: FF_DL={ff_dl}")

    # --- answer with the flow control we want tested ---
    send(ser, RSP_ID, [0x30, bs, stmin_ms])
    fc_at = time.monotonic()

    # --- measure the first block ---
    frames, _ = collect_block(ser, fc_at + 30.0)
    if not frames:
        log("!! no consecutive frames arrived")
        return None

    gaps_ms = [
        (frames[i][0] - frames[i - 1][0]) * 1000.0 for i in range(1, len(frames))
    ]
    first_gap_ms = (frames[0][0] - fc_at) * 1000.0

    log(f"    frames before the sender paused : {len(frames)}")
    if gaps_ms:
        log(
            f"    inter-frame gap ms             : "
            f"min {min(gaps_ms):.1f}  median {sorted(gaps_ms)[len(gaps_ms) // 2]:.1f}  "
            f"max {max(gaps_ms):.1f}"
        )
    log(f"    FC -> first CF ms              : {first_gap_ms:.1f}")

    # --- verdicts ---
    expected_block = ff_dl_blocks(ff_dl, bs)
    if bs == 0:
        bs_verdict = "n/a (BS=0 means unlimited)"
    elif len(frames) <= bs:
        bs_verdict = f"HONORED (paused after {len(frames)}, asked {bs})"
    else:
        bs_verdict = f"IGNORED (sent {len(frames)} without waiting, asked {bs})"

    if stmin_ms == 0:
        stmin_verdict = "n/a (asked for 0)"
    elif not gaps_ms:
        stmin_verdict = "no gaps to measure (single frame block)"
    elif min(gaps_ms) >= stmin_ms * 0.8:
        stmin_verdict = f"HONORED (min gap {min(gaps_ms):.1f} ms >= asked {stmin_ms} ms)"
    else:
        stmin_verdict = f"IGNORED (min gap {min(gaps_ms):.1f} ms << asked {stmin_ms} ms)"

    log(f"    BS    : {bs_verdict}")
    log(f"    STmin : {stmin_verdict}")

    # Keep granting blocks so the sender can finish and not sit in an error.
    for _ in range(expected_block + 4):
        send(ser, RSP_ID, [0x30, bs, stmin_ms])
        more, _ = collect_block(ser, time.monotonic() + 2.0)
        if not more:
            break
    return len(frames), gaps_ms


def ff_dl_blocks(ff_dl, bs):
    cfs = max(0, (ff_dl - 6 + 6) // 7)
    return cfs if bs == 0 else (cfs // max(bs, 1)) + 1


def main():
    argv = sys.argv[1:]
    opts = {}
    positional = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a.startswith("--"):
            # Every option here takes a value; consume it so it is not
            # mistaken for the tty path.
            opts[a] = argv[i + 1] if i + 1 < len(argv) else ""
            i += 2
        else:
            positional.append(a)
            i += 1

    path = positional[0] if positional else TTY
    bs = int(opts.get("--bs", 2))
    stmin = int(opts.get("--stmin", 50))
    label = opts.get("--label", "device under test")

    ser = open_adapter(path)
    log(f"fc probe up on {path}: listening {REQ_ID:03x}, answering {RSP_ID:03x}")
    try:
        while True:
            characterise(ser, bs, stmin, label)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
