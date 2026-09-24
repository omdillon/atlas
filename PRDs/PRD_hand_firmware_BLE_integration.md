# PRD — Hand Firmware BLE Integration Requirements
**Project:** EPSRC Prosthetic Hand Development (ENG:04)
**Author:** Om Kalyan
**Version:** 1.0
**Date:** 2026-07-21
**Status:** Draft — derived requirements, feeds into the PRD v3 implementation plan
**Related documents:** `PRD_v3_prosthetic_smart_interface.md` (parent/authoritative system PRD), `ESP32S3_DevKitC1_Experiments.md`

---

## 1. Overview

### 1.1 Purpose

This document specifies the concrete firmware requirements for adding BLE control to the real hand firmware (`robolimb_esp32`, referred to below by its working copy `hand.cpp`), derived by working backward from a feature-complete BLE control GUI (`tools/exp09_refinedtools/telemetry.py`) and its paired sandbox firmware (`src/exp09_refined/main.cpp`). The sandbox firmware simulates motor behavior in software to validate the BLE protocol and GUI end-to-end; this PRD captures exactly what must be true of the *real* firmware for that same GUI to operate against real hardware, unmodified.

### 1.2 Scope

Covers the BLE transport layer and its firmware-side data sources for one hand: connection lifecycle, motor telemetry reporting, and grasp/reset/fault command handling. Cross-references real methods available on the existing `DRV8214` driver library (`lib/drv8214_multiplatform`) so each requirement is traceable to either an existing capability or a net-new piece of firmware logic.

**Out of scope** (see §5): LED indicator control, camera/computer-vision input, OTA firmware update, EMG channels, PID positional control. None of these are exercised by the current GUI.

### 1.3 Background / Source material

- GUI: `tools/exp09_refinedtools/telemetry.py` — the reference client this PRD's requirements must satisfy.
- Sandbox firmware (protocol reference, simulated data): `src/exp09_refined/main.cpp`.
- Real firmware (target of this PRD, motor control already implemented, BLE not yet added): `src/exp09_refined/hand.cpp` (working copy of `robolimb_esp32/robolimb_esp32/src/main.cpp`).
- Motor driver capabilities: `robolimb_esp32/robolimb_esp32/lib/drv8214_multiplatform/include/DRV8214.h`.

---

## 2. Functional Requirements

### 2.1 Connection lifecycle

| ID | Requirement |
|---|---|
| FR-BLE-01 | Device advertises as `"ProHand-Test"` via NimBLE on boot. |
| FR-BLE-02 | On client connect, advertising stops. On client disconnect, advertising restarts automatically (no reboot required to reconnect). |
| FR-BLE-03 | On connect, request maximum ATT MTU (517) and log the negotiated value. Telemetry (FR-BLE-05) requires a minimum 80-byte payload (77-byte frame + 3-byte ATT header); the GUI surfaces a warning if MTU negotiation falls short, so this must be logged, not silently accepted. |

**Reference implementation:** `ServerCallbacks` (`onConnect`/`onDisconnect`/`onMTUChange`) and the `init_ble()` setup sequence in `src/exp09_refined/main.cpp` — directly portable, no hardware-specific logic involved.

### 2.2 Motor telemetry (device → GUI)

| ID | Requirement |
|---|---|
| FR-BLE-04 | A NOTIFY|READ characteristic (`CHAR_TELEMETRY_UUID`) streams a 77-byte frame at 5 Hz (every 200 ms) while a client is connected and subscribed: `uint16 seq` + 5 × `{state:u8, ripple_count:u32, voltage:f32, current:f32, duty_cycle:u8, fault_byte:u8}`, little-endian. |
| FR-BLE-05 | Per-motor `ripple_count`, `voltage`, `current`, `duty_cycle`, and `fault_byte` must be genuine hardware readings, not simulated/fabricated values. |
| FR-BLE-06 | Per-motor `state` (`IDLE`=0 / `MOVING`=1 / `HOLDING`=2 / `STALLED`=3) must be tracked and reported. |
| FR-BLE-07 | Telemetry array index *i* (0-4) must correspond to physical motor *i+1*, i.e. index 0 = motor 1 = thumb, indices 1-4 = motors 2-5 = index/middle/ring/pinky, matching the GUI's confirmed motor-to-finger mapping. |

**Field-by-field data source (traceability):**

| Frame field | Source | Status |
|---|---|---|
| `ripple_count` | `DRV8214::getRippleCount()` | Available now. Returns `uint16_t`; frame field is `uint32_t` — zero-extend, no precision loss (full-closure values are ≤30,000, well within `uint16_t` range). |
| `voltage` | `DRV8214::getMotorVoltage()` | Available now. |
| `current` | `DRV8214::getMotorCurrent()` | Available now. |
| `duty_cycle` | `DRV8214::getDutyCycle()` | Available now. |
| `fault_byte` | `DRV8214::getFaultStatus()` | Available now. Reads the real DRV8214 FAULT register — same bit layout (`FAULT_STALL`/`FAULT_OCP`/`FAULT_OVP`/`FAULT_TSD`/`FAULT_NPOR`/`FAULT_CNT_DONE`, values `0x20`/`0x10`/`0x08`/`0x04`/`0x02`/`0x01`) the GUI already decodes — no remapping needed. |
| `state` | *(no driver equivalent)* | **Net-new.** No `getState()`-style method exists. Requires a `motor_state[5]` array maintained alongside `execMotorCommmand()`: set `MOVING` on FORWARD/REVERSE, `HOLDING`/`IDLE` on BRAKE (by whether ripple count is nonzero), and OR in `STALLED` by checking `getFaultStatus() & FAULT_STALL` at telemetry-build time. |

### 2.3 Commands (GUI → device)

| ID | Requirement |
|---|---|
| FR-BLE-08 | A WRITE characteristic (`CHAR_MOTOR_CMD_UUID`) accepts ASCII text commands, case-insensitive, trimmed of trailing whitespace. |
| FR-BLE-09 | Values `"open"`, `"power"`, `"tripod"`, `"pinch"`, `"pointer"`, `"rock"`, `"wave"`, `"pack"` each trigger the corresponding existing `queueGrasp(graspName)` call. No change to grasp sequencing/timing logic itself — only the trigger path is new. |
| FR-BLE-10 | Value `"reset"` calls `resetRippleCounter()` on all 5 drivers and clears any tracked `motor_state`/target bookkeeping. |
| FR-BLE-11 | Value `"clear_faults"` calls `resetFaultFlags()` on all 5 drivers. |
| FR-BLE-12 | Unrecognised text is logged and ignored (no error response required — matches sandbox behavior). |

**Reference implementation:** `CommandCallbacks` in `src/exp09_refined/main.cpp` — the trim/lowercase/dispatch skeleton ports directly; only the grasp-name → action mapping changes (string → `graspName` enum → existing `queueGrasp()`, instead of string → `GraspStep` table lookup).

### 2.4 Not required by the GUI, but present in the reference protocol

| Item | Notes |
|---|---|
| `CHAR_MOTOR_DIRECT_UUID` (2-byte `[motor_index, direction]` diagnostic write) | No GUI control ever writes to it. Optional to port — useful as a standalone hardware bring-up/scripted-testing hook, not a GUI dependency. |

---

## 3. Non-Functional Requirements

| ID | Requirement |
|---|---|
| NFR-DEP-01 | Add `lib_deps = h2zero/NimBLE-Arduino @ ^1.4.2` to the `platformio.ini` environment building this firmware — currently absent. |
| NFR-COMPAT-01 | Wire format (UUIDs, frame byte layout, command strings) must byte-for-byte match `src/exp09_refined/main.cpp`'s protocol so `telemetry.py` requires zero changes to operate against the real firmware. |
| NFR-SYNC-01 | There is no shared schema file between firmware (C++) and GUI (Python) — any future frame-layout change must be applied to both `build_telemetry_frame()`/`FULL_CLOSURE_RIPPLES` (firmware) and `FRAME_FORMAT`/`FULL_CLOSURE_RIPPLES` (GUI) together. |

---

## 4. Explicitly Out of Scope

- **LED indicator control** — removed from the GUI (EXP-09 dropped the EXP-08 "LED Demo" feature); the real hand has no LED hardware to serve it.
- **Camera / computer-vision input** — removed from the GUI in the same pass ("Hand Tracking" feature); not applicable to a firmware PRD regardless.
- **OTA firmware update, EMG channels, PID positional control** — out of scope per parent PRD v3 (§1.2); not exercised by the current GUI at all.

---

## 5. Risks / Open Questions

- **`motor_state` design (FR-BLE-06) is unimplemented anywhere yet** — needs a concrete design pass (where exactly it's written/read, interaction with the existing `motorQueue`/10ms drain timer) before implementation, not just wiring up an existing getter like the other telemetry fields.
- **`src/exp09_refined/` currently contains both `main.cpp` (sandbox) and `hand.cpp` (real-firmware copy)**, each defining `setup()`/`loop()` — these cannot co-compile in the same PlatformIO environment as-is. Needs resolving (e.g. `hand.cpp` moved to its own environment/directory) before this firmware can build with BLE added.
- **Ripples-per-rotation calibration** (`NUM_RIPPLES`, `RED_RATIO` in `hand.cpp`) is flagged as an open empirical question in PRD v3's changelog — unrelated to BLE integration itself, but affects whether `FULL_CLOSURE_RIPPLES`-style position-percentage math (used by the GUI's Pos% column and Virtual Hand twin) is meaningful on real hardware.

---

## 6. Traceability Summary

Every currently-implemented GUI feature maps to a requirement in this document:

| GUI feature | Requirement(s) |
|---|---|
| Connect / Disconnect / status label | FR-BLE-01, FR-BLE-02 |
| MTU-too-small warning | FR-BLE-03 |
| Motor Telemetry table (State/Ripple/Pos%/Volts/Current/Duty%) | FR-BLE-04 – FR-BLE-07 |
| Fault Detection table | FR-BLE-04, FR-BLE-05 |
| Clear Faults button | FR-BLE-11 |
| Reset button | FR-BLE-10 |
| Gesture Controls (8 grasp buttons) | FR-BLE-08, FR-BLE-09 |
| Virtual Hand digital twin | *(derived client-side from FR-BLE-04/07, no additional firmware requirement)* |
| Help dialog | *(static content, no firmware requirement)* |
