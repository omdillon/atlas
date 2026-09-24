# BLE protocol reference

**This is the single documented source of truth for the wire format.** The
firmware (`firmware/src/main.cpp`, `build_telemetry_frame()`) and the GUI
(`software/telemetry.py`, `FRAME_FORMAT`/`struct.unpack`) each implement this
independently in their own language. There is no shared schema file, so any
change to the frame layout or command vocabulary must be made in three
places by hand: firmware, GUI, and this document. Check all three before
trusting any one of them after a protocol change.

Everything below happens over one custom BLE GATT service, after the initial
connection; no serial/USB traffic is used once connected.

## Identifiers

| | UUID |
|---|---|
| Service | `5237876f-2b43-4a69-9cf3-f01a8f6f2404` |
| Telemetry characteristic (NOTIFY) | `12df09d3-200c-4abe-b423-cbfe105cb2b6` |
| Command characteristic (WRITE) | `c6975e18-d186-4acb-bab3-6138e560b4de` |

Device advertises as **`ProHand-EXP13`** (kept unchanged from the source
firmware this was exported from; renaming it is a deliberately deferred
firmware-behavior change).

## Device to GUI: telemetry frame

Pushed as a BLE NOTIFY on the telemetry characteristic roughly every 60ms
(~17Hz) while a client is connected and subscribed. **55 bytes, little-endian:**

| Bytes | Field | Type |
|---|---|---|
| 0-1 | `seq` | uint16 |
| 2 | `transition_active_flag` | uint8 |
| 3 | `current_node_id` | uint8 |
| 4 | `target_node_id` | uint8 |
| 5-14 | motor 0 (thumb) record | see below |
| 15-24 | motor 1 (index) record | see below |
| 25-34 | motor 2 (middle) record | see below |
| 35-44 | motor 3 (ring) record | see below |
| 45-54 | motor 4 (pinky) record | see below |

Each 10-byte per-motor record:

| Offset within record | Field | Type |
|---|---|---|
| 0 | `state` | uint8 (0=IDLE, 1=MOVING, 2=HOLDING, 3=STALLED) |
| 1-4 | `voltage` | float32 |
| 5-8 | `current` | float32 |
| 9 | `fault_byte` | uint8 (bitfield, see below) |

Python struct format: `"<HBBB" + "BffB" * 5` (`struct.calcsize` = 55).

Position (ripple-derived) and duty cycle are deliberately **not** included;
neither fed anything beyond a now-removed GUI display column.

### Node ID encoding

`current_node_id` / `target_node_id` encode `(gesture index) + 1`:

| id | gesture | id | gesture |
|---|---|---|---|
| 0 | PACK | 7 | *(WAVE, never appears; not a graph node)* |
| 1 | OPEN | 8 | BIRD |
| 2 | POWER | 9 | TWO |
| 3 | TRIPOD | 10 | JAMBO |
| 4 | PINCH | 11 | THREE |
| 5 | POINTER | 12 | THUMB |
| 6 | ROCK | | |

`0xFF` on `current_node_id` **only** means "off-node/mid-transition" (no
finger-mask combination currently matches any known gesture). `target_node_id`
has no such sentinel: it always holds the last-requested node (defaulting to
OPEN before any command is sent), so it simply equals `current_node_id`
whenever the hand is settled. This lets a client reconstruct current/target/
in-progress state purely from telemetry, with no need to remember what it
last clicked, correct even on a fresh reconnect mid-transition.

### Fault bits (`fault_byte`, mirrors the DRV8214's own FAULT register)

| Bit | Name | Meaning |
|---|---|---|
| 0x20 | STALL | Firmware's software stall watchdog detected a stall (ripple-stagnation while driven) |
| 0x10 | OCP | Overcurrent protection tripped |
| 0x08 | OVP | Overvoltage protection tripped |
| 0x04 | TSD | Thermal shutdown: driver IC overheated |
| 0x02 | NPOR | Power-on-reset occurred (e.g. brownout) |
| 0x01 | CNT_DONE | Ripple-counting threshold exceeded (status flag, not a fault, reusing the same register) |

Bits latch until an explicit `clear_faults` command.

## GUI to device: commands

ASCII text, case-insensitive, written to the command characteristic. The
firmware executes every move itself; clients never stream individual motor
steps.

**Gesture commands** (routed through the firmware's transition engine, except
`wave`): `open`, `power`, `tripod`, `pinch`, `pointer`, `rock`, `pack`, `bird`,
`two`, `jambo`, `three`, `thumb`, `wave`.

`wave` is a standalone, non-blocking scripted animation; it bypasses the
transition engine entirely and never appears as a `current_node_id`/
`target_node_id` value.

**Per-finger commands:** `<finger>_close` / `<finger>_open`, where `<finger>`
is one of `thumb`, `index`, `middle`, `ring`, `pinky`.

**Safe override** (top priority, abandons whatever the hand is doing):
`override_open`, `override_pack`.

**Fault reset:** `clear_faults`.

Any unrecognised command is logged and ignored by the firmware.
