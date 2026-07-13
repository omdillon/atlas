# PRD v3 — Prosthetic Hand Smart Interface
**Project:** EPSRC Prosthetic Hand Development (ENG:04)
**Author:** Om Kalyan
**Supervisor:** Dr. Matthew Dyson
**Version:** 3.0
**Date:** 2026-07-07
**Status:** Active — supersedes v2.0

### Changelog from v2.0
| Issue | Change |
|---|---|
| GATT table listed 20-byte telemetry max (incompatible with 28-byte frame) | Corrected to 40 bytes; table now references post-MTU-negotiation size |
| Ripple counts typed as `int16` (overflows at ~32k, within one finger closure) | Retyped as `uint32`; packet size and schema updated accordingly |
| OTA used unacknowledged WRITE WITHOUT RESPONSE with no sequencing | Replaced with chunked WRITE + software ACK/retry protocol; throughput target revised |
| Failsafe watchdog placed on Core 1 with direct I²C calls (thread-unsafe) | Redesigned: Core 1 sets atomic flag; Core 0 owns all I²C and issues brakes |
| No return path defined for calibration ACK despite FR-CAL-05 requiring one | Added `RESPONSE_CHAR_UUID` (NOTIFY) as a fourth GATT characteristic |
| NFR-PERF-03 required telemetry during OTA (Core 1 cannot serve both) | Telemetry is explicitly suspended during OTA; Core 0 drops frames non-blocking |
| PID float byte order unspecified | All multi-byte fields mandated little-endian; implementation note added |
| Ripple count absolute vs delta left as open question | Resolved: absolute `uint32` cumulative; web app plots directly |
| No I²C mutex defined for Core 0 concurrent access | FreeRTOS mutex (`SemaphoreHandle_t`) mandated for all I²C transactions on Core 0 |
| OTA throughput target (6 KB/s) unreachable with acknowledged writes | Revised to 2 KB/s minimum; hybrid ACK protocol described |
| No firmware re-advertising behaviour defined after connection drop | ESP32 must restart advertising within 1 s of disconnection |
| `IPROPI_RESISTOR` value missing from hardware defaults table | Added (4700 Ω) |
| MTU negotiation responsibility unassigned in Web Bluetooth | Clarified: browser calls `server.connect()` then explicit MTU exchange trigger |
| Ripples-per-rotation discrepancy (14 vs Jai's `NUM_RIPPLES 7`, `RED_RATIO 64`) | Flagged as unresolved empirical question; must be confirmed on bench before Phase 2 |

---

## 1. Overview

### 1.1 Purpose

This document defines the requirements for the **Smart Interface** phase of the prosthetic hand project. The system provides a wireless, browser-based developer tool to monitor real-time myoelectric telemetry from the prosthetic hand, dynamically calibrate motor and control parameters, and flash updated firmware to the ESP32 microcontroller — all without a USB cable or serial connection.

### 1.2 Scope

This PRD covers three tightly coupled components:

- **ESP32 firmware layer** — BLE GATT server, telemetry packaging, OTA receive handler, inter-core safety architecture
- **BLE transport layer** — GATT service/characteristic architecture, packet schemas, OTA integrity protocol
- **Web application** — Web Bluetooth client, real-time oscilloscope dashboard, calibration panel, OTA uploader

Out of scope for this PRD: embedded intent decoding (Phase 3), PID positional control (Phase 2), IMU integration, and clinical calibration protocols (Phase 5).

### 1.3 Hardware Platform

| Component | Detail |
|---|---|
| Microcontroller | ESP32 (dual-core Xtensa LX6, 240 MHz) |
| Motor drivers | DRV8214 × 5 (one per finger), I²C bus at 100 kHz |
| EMG sensors | Gravity Analog EMG × 4 channels, analogue output to ESP32 ADC |
| Firmware toolchain | PlatformIO (VS Code), Arduino framework |
| Target browser | Chrome 89+ / Edge 89+ (Web Bluetooth API) |
| BLE version | BLE 4.2+ (target MTU 512 bytes post-negotiation) |

### 1.4 Key Design Constraints

- Web Bluetooth is only available in Chromium-based browsers over HTTPS or `localhost`. GitHub Pages satisfies the HTTPS requirement.
- BLE ATT MTU defaults to 23 bytes (20-byte payload). MTU must be explicitly negotiated upward by the client after connection. The Web Bluetooth API does not expose a `requestMTU()` call — MTU exchange happens implicitly when Chrome connects, but the resulting MTU is browser and OS dependent. The implementation must test and log the negotiated MTU at connection time and adapt chunk sizes accordingly.
- The web app must be a single static `.html` file — no build step, no `node_modules`, no local server.
- All OTA `.bin` files are produced by PlatformIO at `.pio/build/<env>/firmware.bin`.
- **All multi-byte values in all packet schemas are little-endian.** This applies uniformly to the ESP32 (which is natively little-endian) and must be enforced explicitly in the JavaScript implementation using `DataView` with `littleEndian = true`.
- The I²C bus is shared between all five DRV8214 drivers and must never be accessed concurrently from multiple FreeRTOS tasks. A single `SemaphoreHandle_t i2c_mutex` must be acquired before any `Wire` transaction and released immediately after.

---

## 2. User Roles

### Developer / Engineer (primary user)

Uses the interface during active firmware development to:

- Debug motor stalls and verify ripple-count behaviour across all 5 motors
- Observe raw and MAV-filtered EMG signals from all 4 channels in real time
- Tune PID constants and DRV8214 parameters without recompiling and reflashing via USB
- Flash updated firmware builds wirelessly to avoid physical access to the hand

### Clinician / Researcher (future user — Phase 5)

Not in scope for this phase. Architecture decisions must not preclude a clinician-facing view being added as a separate tab in the same web app. The telemetry stream is sufficient for a future non-technical overlay; no firmware changes would be required.

---

## 3. Objectives and Success Metrics

| Objective | Success Metric |
|---|---|
| Real-time telemetry visualisation | EMG and ripple-count data renders at 20–30 Hz without dropped frames or UI jank on a mid-spec laptop |
| Dynamic parameter calibration | A parameter write from the browser is acknowledged by the ESP32 and reflected in the calibration panel within 200 ms |
| Wireless OTA flashing | A 1 MB firmware binary flashes successfully over BLE without USB, completing within 10 minutes |
| Connection reliability | BLE connection sustains a continuous 30-minute development session without requiring manual reconnection |
| Zero-install deployment | The web app opens directly in Chrome from a GitHub Pages URL with no npm install, no local server, and no browser extensions |

> **Note on OTA throughput:** The v2 target of 3 minutes (6 KB/s) is unachievable with the acknowledged write protocol required for integrity. The revised 10-minute target (≈1.7 KB/s) reflects realistic BLE 4.2 throughput with a software ACK every 8 chunks. If empirical testing shows better throughput, the target may be tightened.

---

## 4. System Architecture

### 4.1 Layer Overview

The system is decomposed into three layers with clean interfaces. The ESP32 firmware is never aware of the web app's UI. The web app is never aware of motor hardware. The BLE transport is the sole contract between them.

```
┌──────────────────────────────────────────────────────────────┐
│                       Web Application                         │
│  BLE Manager │ Telemetry Parser │ Dashboard │ OTA Uploader   │
└───────────────────────────┬──────────────────────────────────┘
                            │  Web Bluetooth API (Chrome/Edge)
┌───────────────────────────▼──────────────────────────────────┐
│                       BLE Transport                           │
│  TELEMETRY (NOTIFY) │ RESPONSE (NOTIFY) │ COMMAND (WRITE)   │
│  OTA_DATA (WRITE)                                            │
└───────────────────────────┬──────────────────────────────────┘
                            │  GATT over BLE 4.2+
┌───────────────────────────▼──────────────────────────────────┐
│                     ESP32 Firmware                            │
│  Core 0: ADC · MAV · I²C+mutex · Ripple · PID · Stall       │
│          · Failsafe brake (on atomic flag from Core 1)       │
│  Core 1: GATT server · Notify scheduler · Cmd parser        │
│          · OTA handler · Disconnect watchdog (sets flag)     │
└──────────────────────────────────────────────────────────────┘
```

### 4.2 ESP32 Dual-Core Architecture

#### Core 0 — Real-time control loop

Core 0 owns **all hardware access**. Nothing on Core 1 ever touches the I²C bus or the ADC directly.

Responsibilities:
- ADC sampling of all 4 EMG channels
- MAV (Mean Absolute Value) feature extraction per channel
- I²C transactions to all 5 DRV8214 drivers (always mutex-guarded)
- Ripple count polling from each DRV8214 via I²C
- PID positional control loop (Phase 2; stub for Phase 1)
- Stall detection: inrush window suppression (22 ms on startup) + ripple-count delta monitoring
- Telemetry frame assembly, pushed to inter-core queue with `xQueueSend(..., 0)` — **non-blocking, discards on full**
- **Failsafe brake execution:** reads `volatile atomic_bool ble_disconnected` flag set by Core 1; if set, acquires I²C mutex and issues brake commands to all 5 DRV8214s, then clears the flag

#### Core 1 — BLE stack

Core 1 owns the BLE stack exclusively. It never calls `Wire`, never touches ADC GPIO, and never directly actuates motors.

Responsibilities:
- GATT server lifecycle: advertising, connection handling, MTU negotiation logging
- **Re-advertising:** within 1 second of any disconnection event, Core 1 restarts BLE advertising so the web app can reconnect without power-cycling the device
- Telemetry notification scheduler: pulls frames from inter-core queue, paces notifications at 20–30 Hz
- **OTA mode:** when an OTA session is active, telemetry notifications are suspended. Core 1 sets `volatile atomic_bool ota_active = true`. Core 0 reads this flag and calls `xQueueSend` with timeout 0 (non-blocking drop) rather than blocking, ensuring the real-time loop is never stalled by a full queue
- Command characteristic write handler: parses incoming command bytes and either applies parameters (via a second FreeRTOS queue to Core 0) or executes BLE-layer actions directly
- Response characteristic notifier: sends ACK/NACK frames to the browser after each command or OTA checkpoint
- Disconnect watchdog: on `onDisconnect()` callback, sets `volatile atomic_bool ble_disconnected = true` (Core 0 reads this to execute brakes), restarts advertising, and cancels any active OTA session

#### Inter-core communication

| Mechanism | Direction | Purpose |
|---|---|---|
| `QueueHandle_t telemetry_queue` (depth 4) | Core 0 → Core 1 | Telemetry frames for BLE notification |
| `QueueHandle_t command_queue` (depth 8) | Core 1 → Core 0 | Parsed calibration parameters to apply |
| `volatile atomic_bool ble_disconnected` | Core 1 → Core 0 | Signals failsafe brake execution |
| `volatile atomic_bool ota_active` | Core 1 → Core 0 | Signals Core 0 to drop telemetry non-blocking |
| `SemaphoreHandle_t i2c_mutex` | Shared (Core 0 only acquires) | Guards all Wire transactions |

> **Critical:** `i2c_mutex` must only ever be acquired by Core 0 tasks. Core 1 must never call `Wire` directly. If Core 1 needs to issue a motor command (e.g. during initial connection handshake), it must enqueue it to `command_queue` and let Core 0 execute it.

### 4.3 GATT Service Architecture

All four characteristics live under a single custom 128-bit service UUID. UUIDs must be generated once using a UUID v4 generator and hardcoded identically in both firmware and web app.

| Characteristic | UUID constant | Operation | Max payload | Purpose |
|---|---|---|---|---|
| `TELEMETRY_CHAR_UUID` | `TELEM_UUID` | NOTIFY | 40 bytes | ESP32 → Browser: sensor frame at 20–30 Hz |
| `RESPONSE_CHAR_UUID` | `RESP_UUID` | NOTIFY | 8 bytes | ESP32 → Browser: ACK/NACK for commands and OTA checkpoints |
| `COMMAND_CHAR_UUID` | `CMD_UUID` | WRITE WITH RESPONSE | 20 bytes | Browser → ESP32: parameter updates and control commands |
| `OTA_DATA_CHAR_UUID` | `OTA_UUID` | WRITE WITHOUT RESPONSE | 512 bytes | Browser → ESP32: firmware chunk data (high-throughput) |

> **MTU dependency:** The 40-byte telemetry payload and 512-byte OTA chunk both exceed the default 20-byte ATT payload. These sizes are only valid after MTU negotiation. At connection time the firmware must log the negotiated MTU. If MTU remains at 23 bytes (20-byte payload), the OTA chunk size must fall back to 18 bytes and the telemetry frame must be split across two notifications. The web app must handle both cases.

### 4.4 Telemetry Packet Schema

Transmitted on `TELEMETRY_CHAR_UUID` as a packed binary buffer. **All fields little-endian.**

```
Offset  Field                 Type      Bytes  Notes
0       Frame sequence no.    uint16    2      Wraps at 65535; web app uses for drop detection
2       EMG raw ch.0          int16     2      Raw 12-bit ADC value, sign-extended
4       EMG raw ch.1          int16     2
6       EMG raw ch.2          int16     2
8       EMG raw ch.3          int16     2
10      MAV ch.0              uint16    2      MAV × 100 (fixed-point, 0–4095)
12      MAV ch.1              uint16    2
14      MAV ch.2              uint16    2
16      MAV ch.3              uint16    2
18      Ripple count M1       uint32    4      Motor 1 absolute cumulative count
22      Ripple count M2       uint32    4      Motor 2
26      Ripple count M3       uint32    4      Motor 3
30      Ripple count M4       uint32    4      Motor 4
34      Ripple count M5       uint32    4      Motor 5
38      Stall flags           uint8     1      Bitmask: bit N = motor N+1 stalled
39      System event          uint8     1      See event code table below

Total: 40 bytes
```

**System event codes:**

| Code | Event |
|---|---|
| `0x00` | No event |
| `0x01` | Grip triggered (grip ID in stall flags byte) |
| `0x02` | All ripple counters reset |
| `0x03` | OTA session started |
| `0x04` | OTA session complete — rebooting |
| `0x05` | OTA session aborted (error) |
| `0x06` | BLE re-advertising started |

> **Ripple count note:** `uint32` supports up to 4,294,967,295 cumulative ripples. At ~30,000 per full finger closure, this provides over 143,000 full closures before overflow — sufficient for the lifetime of this system. Web app plots the absolute value directly; no delta accumulation required.

> **Sequence number note:** The web app shall track the sequence number and log a warning in the event console when `received_seq != expected_seq`. Gaps indicate dropped BLE notifications. This is diagnostic only — the system does not retransmit telemetry frames.

### 4.5 Response Packet Schema

Transmitted on `RESPONSE_CHAR_UUID` as a packed binary buffer. **All fields little-endian.** Used for calibration ACK/NACK and OTA checkpoint acknowledgement.

```
Offset  Field                 Type      Bytes  Notes
0       Response type         uint8     1      See response type table below
1       Command ID echoed     uint8     1      Echoes the command ID being acknowledged
2       Motor/channel ID      uint8     1      Which motor or channel this ACK covers
3       Status                uint8     1      0x00 = OK, 0x01 = invalid value, 0x02 = busy
4       Echoed value (lo)     uint16    2      Echoes the accepted value (for cal confirmation)
6       OTA chunk index       uint16    2      For OTA ACK: index of last accepted chunk

Total: 8 bytes
```

**Response type codes:**

| Code | Meaning |
|---|---|
| `0x01` | Calibration parameter ACK/NACK |
| `0x02` | Override command ACK |
| `0x03` | OTA checkpoint ACK (every 8 chunks) |
| `0x04` | OTA begin ACK (includes negotiated chunk size in echoed value field) |
| `0x05` | OTA abort notification |

### 4.6 Command Packet Schema

Written to `COMMAND_CHAR_UUID` (WRITE WITH RESPONSE). **All fields little-endian.**

```
Byte 0       Command ID    uint8
Byte 1–N     Payload       varies (see table)
```

| Cmd ID | Name | Payload | Notes |
|---|---|---|---|
| `0x01` | Set KMC | `uint8 motor_id, uint8 kmc` | motor_id: 1–5 |
| `0x02` | Set KMC scale | `uint8 motor_id, uint8 scale` | scale: 1–4 |
| `0x03` | Set inrush duration | `uint8 motor_id, uint16 ms` | little-endian uint16 |
| `0x04` | Set brake threshold | `uint8 motor_id, uint16 threshold` | little-endian uint16 |
| `0x05` | Set IPROPI resistor | `uint8 motor_id, uint16 ohms` | Default 4700 Ω |
| `0x06` | Set EMG min/max | `uint8 channel, int16 min, int16 max` | channel: 0–3 |
| `0x07` | Set PID constants | `uint8 motor_id, float32 Kp, float32 Ki, float32 Kd` | 3 × IEEE 754 little-endian |
| `0x10` | Trigger grip | `uint8 grip_id` | 0=open, 1=power, 2=tripod, 3=lateral |
| `0x11` | Zero ripple counters | `uint8 motor_mask` | bitmask; 0xFF = all motors |
| `0x20` | OTA begin | `uint32 total_size` | Initiates OTA session |
| `0x21` | OTA abort | _(no payload)_ | Cancels active OTA session |

> **Float encoding (cmd `0x07`):** PID constants are IEEE 754 single-precision floats. The ESP32 receives them with `memcpy(&float_val, &buf[offset], 4)`. The web app must write them using `dataView.setFloat32(offset, value, true)` (littleEndian = true). Failure to set the endian flag will silently send garbage PID values that may cause uncontrolled motor behaviour.

### 4.7 OTA Integrity Protocol

OTA uses a hybrid approach: high-throughput `WRITE WITHOUT RESPONSE` for data chunks, with a software-level checkpoint ACK every 8 chunks to detect and recover from packet loss.

**Session flow:**

```
Browser                                    ESP32
   |                                         |
   |-- CMD 0x20 (OTA begin, total_size) ---> |
   |                                         | Update.begin(total_size)
   |<-- RESP 0x04 (ACK, chunk_size=512) ---- |
   |                                         |
   | [Repeat for each chunk group of 8]      |
   |-- OTA chunk 0 (512 bytes, no resp) ---> |
   |-- OTA chunk 1 (512 bytes, no resp) ---> |
   |-- OTA chunk 2 (512 bytes, no resp) ---> |
   |   ...                                   |
   |-- OTA chunk 7 (512 bytes, no resp) ---> |
   |                                         | Update.write() × 8
   |<-- RESP 0x03 (OTA checkpoint, idx=7) -- |
   |                                         |
   | [If checkpoint not received in 2 s]     |
   | [browser retransmits group from idx N]  |
   |                                         |
   | [Final partial group + last chunk]      |
   |-- CMD 0x21... no, final chunk signals   |
   |   end by total bytes matching           |
   |                                         | Update.end()
   |                                         | Update.isFinished() → reboot
   |  [BLE disconnection detected]           |
   |--> display "Flash complete — reconnect" |
```

**Error handling:**
- If `Update.hasError()` after any `write()` call: ESP32 sends RESP `0x05` (OTA abort), calls `Update.abort()`, and resumes telemetry
- If BLE disconnects mid-OTA: Core 1 `onDisconnect()` calls `Update.abort()` before setting the disconnect flag. Partial image is never applied
- If browser does not receive a checkpoint ACK within 2 seconds: it retransmits the last group of 8 chunks from the last acknowledged index. The ESP32 tracks the byte offset written and rejects chunks below the current write pointer (idempotent re-send is safe)
- If total received bytes exceed `total_size`: ESP32 sends RESP `0x05` and aborts

**Chunk size adaptation:**
- Default chunk size: 512 bytes (requires negotiated MTU ≥ 515 bytes with 3-byte ATT header)
- Fallback chunk size: 18 bytes (safe for default 23-byte MTU)
- At OTA begin ACK (RESP `0x04`), the ESP32 echoes the chunk size it expects based on the negotiated MTU. The browser must use this value, not assume 512 bytes

---

## 5. Functional Requirements

### 5.1 Connection Management

- **FR-CON-01:** The web app shall provide a Connect button that initiates a BLE device scan filtered to the prosthetic hand's custom service UUID
- **FR-CON-02:** The web app shall display connection state: `Disconnected` / `Connecting` / `Connected` and the RSSI value (dBm) updated every 2 seconds while connected
- **FR-CON-03:** Immediately after `server.connect()` resolves, the web app shall log the negotiated ATT MTU to the event console. If MTU cannot be determined, it shall assume 23 bytes (conservative fallback)
- **FR-CON-04:** On unexpected disconnection, the web app shall: (a) freeze all live charts, (b) display a prominent `Connection lost` banner, (c) automatically attempt one reconnect after 3 seconds, (d) if reconnect fails, display a manual reconnect button. The ESP32 will have restarted advertising within 1 second of disconnection
- **FR-CON-05:** The web app shall handle the case where the browser does not support Web Bluetooth (`navigator.bluetooth === undefined`) by displaying a clear error message with a link to a compatible browser, rather than a JavaScript exception

### 5.2 Telemetry Dashboard

- **FR-TEL-01:** The dashboard shall display a scrolling oscilloscope for each of the 4 raw EMG channels, with a configurable time window (default 5 seconds, range 1–30 seconds)
- **FR-TEL-02:** The dashboard shall overlay the MAV-filtered signal on each EMG oscilloscope in a visually distinct colour
- **FR-TEL-03:** The dashboard shall display the current ripple count for each of the 5 motors as both a numeric readout and a proportional bar indicating position between 0 and the configured full-closure count
- **FR-TEL-04:** The dashboard shall display a fixed-height scrolling event log with timestamps (relative to session start, in seconds) showing: stall detections, grip triggers, counter resets, OTA events, and dropped-frame warnings
- **FR-TEL-05:** The dashboard shall display the current telemetry notification rate (Hz), calculated as a rolling average over the last 20 frames, to confirm the 20–30 Hz target is being met
- **FR-TEL-06:** The dashboard shall detect gaps in the frame sequence number and log `[WARN] Dropped frame: seq expected N, got M` in the event log. Isolated drops are expected; sustained gaps indicate a BLE reliability problem

### 5.3 Calibration Controls

- **FR-CAL-01:** The calibration panel shall provide per-motor numeric inputs for: KMC (0–255), KMC scale (1–4), inrush duration (ms), brake threshold, and IPROPI resistor value (Ω)
- **FR-CAL-02:** The calibration panel shall provide per-channel numeric inputs for EMG minimum (noise floor) and maximum (saturation) thresholds
- **FR-CAL-03:** The calibration panel shall provide per-motor numeric inputs for PID constants Kp, Ki, and Kd
- **FR-CAL-04:** Calibration changes shall be sent on user commit (slider release or input field blur). During an active OTA session, the calibration panel shall be disabled and greyed out
- **FR-CAL-05:** Each calibration control shall show two values side by side: the pending value the user has set, and the last confirmed value echoed in a RESPONSE notification from the ESP32. If no ACK is received within 500 ms of a write, the control shall display a `⚠ No ACK` indicator
- **FR-CAL-06:** The calibration panel shall include a "Load defaults" button that resets all inputs to the hardware defaults defined in Section 7 and immediately sends all default values to the ESP32

### 5.4 Manual Overrides

- **FR-OVR-01:** The override panel shall provide buttons to trigger each named grip state: Open, Power grip, Tripod grip, Lateral grip
- **FR-OVR-02:** The override panel shall provide a per-motor and an all-motors Zero ripple counters button
- **FR-OVR-03:** Override commands shall be sent immediately on button press with no confirmation dialog
- **FR-OVR-04:** During an active OTA session, the override panel shall be disabled and greyed out

### 5.5 OTA Firmware Uploader

- **FR-OTA-01:** The OTA tab shall provide a file picker (`<input type="file" accept=".bin">`) for selecting the firmware binary. No drag-and-drop is required
- **FR-OTA-02:** After file selection and before flashing begins, the web app shall display: filename, file size in KB, and estimated flash time based on current BLE throughput (or a default estimate if no prior session data)
- **FR-OTA-03:** The web app shall display a progress bar showing bytes confirmed-delivered / total bytes. Progress only advances on receipt of a checkpoint ACK from the ESP32, not on chunk write
- **FR-OTA-04:** The web app shall display the current measured OTA throughput (KB/s) as a live readout during flashing
- **FR-OTA-05:** On successful completion (BLE disconnect detected after `0x04` system event in telemetry), the web app shall display a success state and prompt the user to reconnect
- **FR-OTA-06:** If OTA is aborted by the ESP32 (RESP `0x05`), the web app shall display the reason and offer a retry button without requiring page reload
- **FR-OTA-07:** The OTA tab shall be disabled (greyed out) unless the BLE connection is active
- **FR-OTA-08:** During active OTA flashing, the telemetry dashboard shall display a `OTA in progress — telemetry suspended` overlay. This is expected behaviour, not an error

---

## 6. Non-Functional Requirements

### 6.1 Performance

- **NFR-PERF-01:** The telemetry parser and chart rendering pipeline shall process each incoming BLE notification within 40 ms to maintain smooth 25 Hz rendering. Chart updates shall use `requestAnimationFrame` and never block the main thread
- **NFR-PERF-02:** OTA throughput shall sustain a minimum of **1.7 KB/s** over BLE with the checkpoint-ACK protocol, targeting completion of a 1 MB binary within 10 minutes. If sustained throughput exceeds 3 KB/s in practice, the target shall be revised
- **NFR-PERF-03:** Telemetry is explicitly suspended during OTA. Core 0 shall drop telemetry frames non-blocking (zero-timeout `xQueueSend`) when `ota_active` is true, ensuring the real-time control loop is never stalled by queue backpressure

### 6.2 Reliability and Safety

- **NFR-REL-01:** If BLE disconnects during OTA, Core 1's `onDisconnect()` callback must call `Update.abort()` before setting the `ble_disconnected` flag. Partial firmware images must never be applied under any disconnection scenario
- **NFR-REL-02:** On receiving the `ble_disconnected` flag, Core 0 shall acquire the I²C mutex and issue brake commands to all 5 DRV8214 motors within 500 ms. Core 0 is the sole executor of this action — Core 1 must not touch the I²C bus
- **NFR-REL-03:** The command queue (`command_queue`) shall be checked at the top of every Core 0 loop iteration. Calibration parameters received during motor actuation shall be applied at the next safe moment (between PID cycles), not mid-movement
- **NFR-REL-04:** Invalid command payloads (out-of-range values, unknown command IDs) shall be rejected by Core 1's command parser with a NACK response (RESP status `0x01`). They must never be forwarded to `command_queue`

### 6.3 Deployability

- **NFR-DEP-01:** The web app shall be a single self-contained `.html` file. All JavaScript may be inlined or loaded from `cdnjs.cloudflare.com` or `cdn.jsdelivr.net`. No other external origins
- **NFR-DEP-02:** The web app shall be deployable to GitHub Pages and function correctly over HTTPS
- **NFR-DEP-03:** The web app shall function on `localhost` (HTTP) for development without HTTPS
- **NFR-DEP-04:** The web app shall not use `localStorage` or any persistent browser storage in v3. Calibration state is held in memory for the session only. Persistence may be added in a future minor version once the calibration schema is stable

### 6.4 Browser Compatibility

- **NFR-BRW-01:** Target browsers: Chrome 89+ and Edge 89+. Firefox and Safari are out of scope (no Web Bluetooth support)
- **NFR-BRW-02:** If `navigator.bluetooth` is undefined, the web app shall display an explicit unsupported-browser message with the text "Web Bluetooth is not available in this browser. Please use Chrome or Edge on desktop." It must not throw a JavaScript exception

---

## 7. Hardware Parameters and Defaults

These values were established empirically in Jai Nayyar's thesis and confirmed on the university PCB. They are the authoritative defaults for both firmware initialisation and the web app calibration panel's "Load defaults" state.

| Parameter | Constant name | Default value | Notes |
|---|---|---|---|
| Internal motor resistance | `INTERNAL_MOTOR_R` | 2.5957 Ω | Measured via LCR bridge; use this, not the 2.6 approximation |
| KMC (back-EMF constant) | `KMC_VALUE` | 5 | |
| KMC scale | `KMC_SCALE` | 4 | Range 1–4 |
| Inrush current duration | `INRUSH_MS` | 22 ms | Stall detection suppressed for this window on motor start |
| IPROPI resistor | `IPROPI_RESISTOR` | 4700 Ω | Connected to IPROPI pin; affects current sense scaling |
| Motor supply current | `MOTOR_CURRENT` | 0.8 A | Constant-current mode |
| I²C frequency | `I2C_FREQUENCY` | 100,000 Hz | 100 kHz standard mode |
| SCL pin | `SCL_PIN` | GPIO 1 | |
| SDA pin | `SDA_PIN` | GPIO 2 | |
| nFAULT pin (motor 5) | `NFAULT5` | GPIO 36 | Low when stall detected on driver 5 |
| Max motor RPM (safe) | `MAX_RPM` | 400 RPM | Physical limit ~1500 RPM; 400 used for safety |
| Full finger closure | — | ~30,000 ripples | Empirical; thumb uses ~10,000 from open |

### ⚠ Unresolved: Ripples-per-rotation discrepancy

Section 7 of v2 stated 14 ripples per motor rotation. Jai's firmware defines `NUM_RIPPLES 7` (per output shaft revolution) and `RED_RATIO 64` (with the note "best guess is 1:16 so 16"). These values are mutually inconsistent and suggest the gearbox reduction ratio has not been precisely measured.

**This must be resolved on the bench before any Phase 2 positional control work.** An incorrect ripples-per-revolution constant will make all angle/position calculations wrong by a fixed multiplier. The recommended method is to command a motor through exactly one full output shaft rotation (verified visually), count the ripples reported by `getRippleCount()`, and record the result as the definitive `RIPPLES_PER_OUTPUT_REV` constant. Until this is done, the full-closure ripple count (~30,000) should be treated as a calibration target, not a derived value.

---

## 8. Resolved Design Decisions

The following questions were open in v2 and are now resolved:

| # | Question | Decision | Rationale |
|---|---|---|---|
| 1 | Persist calibration profiles to `localStorage`? | No — in-memory only for v3 | Schema likely to change during development; persistence adds complexity with no current benefit |
| 2 | Include sequence number in telemetry frame? | Yes — `uint16` at offset 0 | 2-byte cost; enables dropped-frame detection that is directly useful for diagnosing BLE reliability |
| 3 | Does `BLEDevice::setMTU()` need explicit call on ESP32? | Yes — call `BLEDevice::setMTU(517)` in setup(); also log actual negotiated MTU in `onMTUChange()` callback | Arduino BLE library defaults to 23 bytes unless explicitly set |
| 4 | Absolute cumulative vs delta ripple count? | Absolute `uint32` cumulative | Web app plots directly; no client-side accumulation; `uint32` eliminates overflow concern |
| 5 | OTA: INDICATE vs WRITE WITHOUT RESPONSE? | WRITE WITHOUT RESPONSE + software checkpoint ACK every 8 chunks | Best throughput; integrity maintained by ACK/retry; INDICATE would be ~3× slower |

---

## 9. Development Phases

### Phase 1A — BLE skeleton
ESP32 GATT server advertising with all four characteristics registered. Telemetry sends a synthetic counter frame (incrementing sequence number, zeroed sensor fields). Web app connects, subscribes to TELEMETRY and RESPONSE notifications, parses the frame, and displays the sequence number in the event log. OTA and command characteristics accept writes and log byte counts without acting on them. **Goal:** validate the full BLE pipeline — connection, MTU negotiation, notification delivery, and WRITE — before any real hardware is involved.

### Phase 1B — Live telemetry
Wire Core 0 ADC reads and DRV8214 I²C ripple count polling into the telemetry frame. Add the I²C mutex. EMG oscilloscopes and ripple-count bars go live in the web app. Stall flags and system events visible in the event log. Validate that the 20–30 Hz notification rate is sustained.

### Phase 1C — Calibration writes and ACK
Implement Core 1 command parser and Core 0 command queue consumer. Implement Core 1 RESPONSE notifier. Web app calibration panel functional end-to-end: write a parameter, receive ACK, see confirmed value update. Test NACK path with an out-of-range value.

### Phase 1D — Manual overrides
Implement grip trigger commands and ripple counter reset. Verify each grip state physically actuates the correct motors. Verify that override commands during active telemetry do not disrupt the notification stream.

### Phase 1E — OTA
Implement OTA handler on Core 1 with `Update.h`. Implement checkpoint ACK protocol. Implement `ota_active` flag and Core 0 non-blocking queue drop. Test full flash of a 1 MB binary. Test abort on disconnection mid-flash. Verify post-flash reconnect. Verify that the pre-flash firmware is preserved on incomplete flash.

### Phase 1F — Hardening and reliability testing
30-minute continuous telemetry session with active motor commands. Test cases: BLE drop during OTA, invalid command payloads, motor stall during calibration write, browser tab backgrounded during flash, rapid connect/disconnect cycles. Confirm re-advertising restarts reliably after each disconnection.

---

## 10. Known Limitations (v3)

| Limitation | Consequence | Planned resolution |
|---|---|---|
| Telemetry suspended during OTA | Developer cannot observe EMG or motor state while flashing | Acceptable for developer tool; not a clinical constraint |
| No calibration profile persistence | Settings lost on page reload | Phase 1 scope only; add in a later minor version once schema is stable |
| Ripples-per-rotation constant unverified | Phase 2 positional control will be inaccurate until resolved | Bench measurement required before Phase 2 begins |
| OTA throughput ≈1.7–3 KB/s | ~5–10 min for a 1 MB binary | Acceptable for a dev tool; not a latency-critical path |
| Single BLE connection | Cannot connect two clients simultaneously | Not needed for Phase 1; flag for Phase 5 multi-device |

---

## 11. Future Scope (Phase 2+)

- **PID positional control (Phase 2)** — Ripple count as PV in a closed-loop PID controller per motor. Requires the `uint32` ripple counts and command `0x07` (PID constants) already defined in this PRD. No BLE schema changes needed.
- **Abstract decoder visualiser (Phase 3)** — 2D MCI cursor space in the web app, replicating the Dyson 2018 centre-out task. Requires a new NOTIFY characteristic streaming normalised muscle activation vectors (2× `float32`). Firmware schema additive change only.
- **Session recording and CSV export** — In-memory buffer of telemetry frames with download-as-CSV. Web app only; no firmware changes required.
- **Clinician view** — Simplified tab showing EMG amplitude as % of MVC, current grip state, and session fatigue indicator. Same telemetry stream; different rendering. No firmware changes.
- **IMU integration (Phase 4)** — MPU6050 over I²C on Core 0. Two additional `int16` fields (roll, pitch) appended to the telemetry frame at offsets 40–43. Frame grows to 44 bytes; backward-compatible if web app checks frame length before parsing IMU fields.
- **localStorage calibration profiles** — Once the command schema is stable (post-Phase 1C), persist per-device calibration profiles keyed by BLE device ID.
