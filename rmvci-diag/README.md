# rmvci-diag

Transport-agnostic **KWP2000 (ISO 14230-3)** and **UDS (ISO 14229-1)**
diagnostic clients with the Toyota extensions recovered from Techstream, built
on the [`rmvci-core`](../rmvci-core) `UdsTransport` seam.

The same clients drive both Toyota diagnostic buses:

| Bus | Transport | Typical ECUs |
|-----|-----------|--------------|
| **K-line / ISO 14230** | `rmvci_core::KLineEcu` | Gen-2 body/HVAC (NHW20 Prius A/C amp, addr `0x98`) |
| **ISO-TP / CAN** | `rmvci_core::FirmwareIsoTp` / `IsoTp` | powertrain, HV, newer CAN A/C amps |

Pick the client by ECU generation:

- **`Kwp2000`** — KWP2000 services (`21`/`30`/`3B`/`18`/`14`/`12` …), the older
  Toyota P3/P4 ECUs (K-line and early CAN).
- **`Uds`** — UDS services (`22`/`2E`/`2F`/`31`/`19` …), the newer P5 ECUs.

## Design

- **Standard definitions are borrowed, not re-typed.** Service ids and negative
  response codes come from [`automotive_diag`](https://crates.io/crates/automotive_diag)
  (MIT/Apache). This crate adds request building, response validation, and the
  non-standard Toyota services.
- **Response-pending is handled in the transport.** `7F .. 78`
  (requestCorrectlyReceived-ResponsePending) frames are swallowed by `rmvci-core`
  before the client sees them, so the client is a thin synchronous
  request/response layer.
- **Every method returns the bytes after the echoed service id / identifier**,
  or maps `7F <sid> <nrc>` to `Error::Negative { service, nrc }`. The error type
  is shared between the two protocols (they share the ISO NRC table).

## Use — KWP2000 over K-line

```rust
use rmvci_core::{Device, KLineEcu};
use rmvci_diag::Kwp2000;

let dev = Device::open("/dev/ttyUSB0")?;
let mut ecu = KLineEcu::connect(&dev, 0x98)?; // vendor bring-up + filter + timing
ecu.fast_init()?;                              // StartCommunication, key bytes
let mut kwp = Kwp2000::new(ecu);

let ambient = kwp.read_data_by_local_id(0x02)?;      // 21 02
let servo   = kwp.read_data_by_local_id(0x0d)?;      // 21 0D air-mix damper (D)
let dtcs    = kwp.read_dtc_by_status(0x00, 0xff00)?; // 18 00 FF00
```

## Use — UDS over CAN

```rust
use rmvci_core::{CanId, Device, FirmwareIsoTp};
use rmvci_diag::Uds;

let dev = Device::open("/dev/ttyUSB0")?;
let isotp = FirmwareIsoTp::new(&dev, CanId::Std(0x7e0), CanId::Std(0x7e8))?;
let mut uds = Uds::new(isotp);

uds.diagnostic_session_control(0x03)?;         // extended session
let vin  = uds.read_data_by_id(0xf190)?;       // 22 F1 90
let dtcs = uds.read_dtc_by_status_mask(0xff)?; // 19 02 FF
```

## Service coverage

**KWP2000** (`Kwp2000`) — `start_diagnostic_session` (10), `ecu_reset` (11),
`tester_present` (3E), `security_access` (27), `read_data_by_local_id` (21),
`read_data_by_id` (22), `read_dtc_by_status` (18), `read_status_of_dtc` (17),
`clear_diagnostic_information` (14), `read_freeze_frame` (12), `io_control_local`
(30), `start_routine_local` (31), `write_data_by_local_id` (3B); Toyota:
`active_test`, `customize_write_toyota` (`A5`), `obd_current_data`/`obd_stored_dtcs`.

**UDS** (`Uds`) — `diagnostic_session_control` (10), `ecu_reset` (11),
`tester_present` (3E), `security_access` (27), `communication_control` (28),
`control_dtc_setting` (85), `read_data_by_id` (22), `write_data_by_id` (2E),
`clear_diagnostic_information` (14), `read_dtc_information` / `read_dtc_by_status_mask`
(19), `io_control_by_id` (2F), `routine_control` (31).

Both expose `send_raw(sid, data)` / `send(Command, data)` for anything not wrapped.

> **Active tests and writes command real hardware.** Only issue them with the
> vehicle in the state the service manual requires.

## Provenance

Toyota-specific service framing (`0x30`/`0x2F` active test, `0xA5`/`0x3B`/`0x2E`
customize write, `0x12` freeze-frame, single-byte `0x21` LID reads) is grounded
in the Techstream database reverse engineering under `re/techstream/`
(`ghidra/notes_acttest.md`, `notes_dtc.md`, `notes_customize.md`,
`notes_freezeframe.md`, and `datalist_by_gen/`).

## License

GPL-3.0-or-later (matching the rmvci workspace).
