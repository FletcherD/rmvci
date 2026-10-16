# Changelog

All notable changes to the published crates. This project follows
[Semantic Versioning](https://semver.org/); while at `0.x`, a minor bump may
break the API.

## rmvci-core 0.1.0 — 2026-10-16

First public release.

- `Device`: opens the cable (serial, direct USB via `nusb`, or a
  caller-supplied `Transport`), runs the DES handshake, and owns the port on
  one actor thread. `close()` waits for the port to be released.
- Typed `Channel<P>` for ISO 9141, ISO 14230, CAN and ISO 15765, with the
  filter identifier width fixed by the protocol type; `RawChannel` for
  untyped access.
- `IsoTp`: ISO 15765-2 through the firmware or host-side over raw CAN
  (`IsoTpPath`), including extended/mixed addressing.
- `KLineEcu`: ISO 14230 framing, the vendor `SET_CONFIG` bring-up,
  FAST_INIT, responsePending draining, and re-init of a wedged ECU.
- `UdsTransport`: the request/response seam both `IsoTp` and `KLineEcu`
  implement.
- Periodic messages, `GET_CONFIG`/`SET_CONFIG`, `READ_VBATT`, programming
  voltage.
- Minimum supported Rust version: 1.88.
