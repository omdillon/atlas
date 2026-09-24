# EXP-11 Plan — Bug Fixes + Safe-State Override

Status: **IMPLEMENTED.** Parts A (A1-A4, plus a later A7 PACK-safety fix not in
the original text below), B (safe-state override, firmware) and C (Safe Override
GUI buttons) all shipped in `src/exp11_dynamic/main.cpp` /
`tools/exp11_dynamictools/telemetry.py` prior to 2026-07-30. That date's session
found and fixed a further, separate homing bug this doc did not cover (a
ripple-count-ceiling false-failure at the CLOSED hard-stop, see
`updateHoming()`'s `rippleCeilingHit` handling) and added a `homing_active_flag`
telemetry byte + GUI button-gating tier so Gesture Controls/finger buttons now
visibly grey out during a homing sweep instead of the firmware silently
discarding their clicks. The rest of this document is kept as the historical
design record for Parts A-D below.

## Context

EXP-11 layered posture gestures (L3) and a transition graph (L4) on top of a
dead-reckoned position estimate (L1) that was only ever *cosmetic* in EXP-09 —
see `docs/exp09_development.md`. Making position control-critical without making
it *true* is the root of the current bugs. This plan (a) fixes those bugs at the
correct layer, and (b) adds a **safe-state override**: a top-priority command
that abandons whatever the hand is doing and drives it to a known safe posture
(**OPEN** or **PACK**), acting as a physical reset.

---

## Part A — Bug fixes

| # | Bug | Root cause (layer) | Fix |
|---|-----|--------------------|-----|
| A1 | Object grip corrupts position to 100% | L1: snap conflates object-stall with endstop-stall | Snap only when position is actually AT the endpoint |
| A2 | Gesture queue can wedge forever | L3: no phase-completion watchdog | Per-phase timeout |
| A3 | Unvalidated direct edge active | L4: `DIRECT_EDGES` ships a TODO edge | Empty the table (pure via-OPEN baseline) |
| A4 | Hand may not physically open at boot | setup(): OPEN is idempotent at assumed pos 0 | Seed assumed-closed, then drive OPEN |
| A5 | Graph mislocalisation | L1 error propagates into `currentGraphNode` | Mostly resolved by A1; optional confidence tighten |
| A6 | Homing open-leg unreliable | Physical: open end has no firm stop | Tuning + validation task (not a code bug) |

### A1 — Object-grip position corruption (headline)
`updateFingerMoves()` `main.cpp:517`. Today, on `reached || STALLED` it brakes,
then snaps if `finger_move_target >= fullClosure`. A finger that **stalls on an
object mid-travel** during a full-close therefore snaps to `fullClosure` while
physically at ~60%.

**Fix:** only re-anchor when the finger is genuinely *at* the endpoint — gate the
snap on measured position, not just the target:
```cpp
static const uint32_t REANCHOR_BAND = FINGER_TARGET_EPSILON;  // ripples near the end
bool at_closed = (finger_move_target[i] >= fullClosure(i)) &&
                 (pos + REANCHOR_BAND >= fullClosure(i));
bool at_open   = (finger_move_target[i] == 0) && (pos <= REANCHOR_BAND);
if      (at_closed) motor_position[i] = (int32_t)fullClosure(i);
else if (at_open)   motor_position[i] = 0;
// else: mid-travel stall (object grip) -> leave position as measured, DO NOT snap
```
Net: drift correction still happens at true endstops; an object grip keeps its
honest ~60% position. This is the single most important fix — A5 largely
dissolves once position stops lying.

### A2 — Per-phase watchdog
`updateGestureQueue()` `main.cpp:1216`. It waits on `finger_move_active` with no
time bound. Add a phase clock: record `phaseStartMs` when a phase starts; if
`millis() - phaseStartMs > PHASE_TIMEOUT_MS` (~4000 ms), force-brake the phase's
fingers, log it, and advance (or abort the remaining queue). Restores the
time-bounded guarantee the EXP-09 FIFO had for free, and prevents a wedged
gesture.

### A3 — Disable the unvalidated direct edge
`DIRECT_EDGES[]` `main.cpp:1271`. Set it empty (`static const DirectEdge
DIRECT_EDGES[] = {};` with `DIRECT_EDGE_COUNT` deriving to 0) so the graph is a
**pure star through OPEN** until an edge is bench-validated. Keeps the mechanism,
removes the latent collision path.

### A4 — Physical open at boot
`setup()`. Before `applyGesture(OPEN)`, seed `motor_position[i] = fullClosure(i)`
(assume worst-case closed) so the OPEN posture is non-idempotent and actually
drives every finger open; each finger then stalls at its open extent and (via A1)
re-anchors to 0. Alternatively call the new `driveOverride(OPEN)` (Part B) at
startup. Either guarantees a defined physical boot posture.

### A6 — Homing reliability (validation, not a patch)
The open leg has no firm hard-stop, so `MEASURE_OPEN` depends on ripple
stagnation. Bench-tune `HOMING_GRACE`/`STALL_*` and confirm the `HOMING_MIN_CLOSURE`
reject rate is low; document expected closure ranges per finger. Track separately.

---

## Part B — Safe-state override (the critical new feature)

A **top-priority** command that overrides any gesture, finger move, or homing in
progress and drives the hand to a **safe posture**, then holds. Two targets, both
required: **OPEN** (hand flat/relaxed) and **PACK** (thumb across palm, fingers
open — the stow pose). It is the hand's "reset to a known physical state."

### Why it bypasses the graph
The override must be *immediate and predictable*. OPEN is the graph's universal
safe waypoint, so driving straight to it is inherently collision-safe. PACK is
made safe by **phasing** (open the fingers first, then sweep the thumb), so it
too needs no graph routing. Bypassing L4 is therefore correct here — the targets
are the safe states.

### Commands (BLE text, case-insensitive, same characteristic)
- `override_open` → drive to OPEN
- `override_pack` → drive to PACK

### Firmware design
1. **Deferred flags** (next to the other pending flags):
   ```cpp
   static volatile bool     pendingOverrideRequest = false;
   static volatile graspName pendingOverrideTarget = OPEN;   // OPEN or PACK
   ```
2. **Parse** in `CommandCallbacks::onWrite` (place ABOVE the grasp/finger checks
   so the words are never captured as anything else):
   ```cpp
   if (value == "override_open") { pendingOverrideTarget = OPEN; pendingOverrideRequest = true; }
   else if (value == "override_pack") { pendingOverrideTarget = PACK; pendingOverrideRequest = true; }
   ```
3. **Dispatch FIRST in `loop()`** — before the homing lockout and before
   grasp/finger/home handling, so nothing outranks it:
   ```cpp
   if (pendingOverrideRequest) { pendingOverrideRequest = false; driveOverride(pendingOverrideTarget); }
   ```
4. **`driveOverride(graspName safe)`** (runs on core 1):
   ```cpp
   static void driveOverride(graspName safe) {
       abortHoming();               // ultimate abort -- works even mid-homing
       cancelAllFingerMoves();
       clearGestureQueue();
       pendingGraspRequest = false; // nothing competes this tick
       pendingHomeRequest  = false;
       for (uint8_t i = 0; i < MOTOR_COUNT; i++) pendingFingerReq[i] = false;

       if (safe == OPEN) enqueueGesturePhases(OPEN);   // single phase, all -> 0
       else              enqueueSafePack();            // phased: fingers open, THEN thumb
       Serial.printf("[OVR] Override -> %s\n", safe == OPEN ? "OPEN" : "PACK");
   }
   ```
   Because `homing_active` is now false, the normal executor
   (`updateGestureQueue` + `updateFingerMoves`) drives the safe posture on the
   following ticks. With A1 in place, reaching OPEN re-anchors every finger to 0
   and PACK re-anchors the thumb to `fullClosure` — so the override doubles as a
   *physical* re-zero, superseding the counter-only `reset`.
5. **Collision-safe PACK** — a dedicated 2-phase so the fingers clear before the
   thumb sweeps:
   ```cpp
   static const MotionPhase GEST_PACK_SAFE[] = {
       {0x1E, {0, 0, 0, 0, 0}},          // phase 0: open index..pinky
       {0x01, {10000, 0, 0, 0, 0}},      // phase 1: then close thumb across palm
   };
   static void enqueueSafePack() {
       for (auto& p : GEST_PACK_SAFE) phaseQueue.push(p);
   }
   ```

### Priority summary
`override_* > reset/clear_faults > homing lockout > grasp/finger`. The override is
processed at the very top of the tick, clears every competing request, aborts
homing, and cannot itself be pre-empted except by another override or a reset.

### Files
- `src/exp11_dynamic/main.cpp` — flags, `onWrite` parse, `loop()` top-of-tick
  dispatch, `driveOverride()`, `GEST_PACK_SAFE` + `enqueueSafePack()`.

---

## Part C — GUI changes (`tools/exp11_dynamictools/telemetry.py`)

- Add a prominent **Safe Override** group (top of the control column, visually
  distinct — amber/red) with two buttons: **OVERRIDE → OPEN** and
  **OVERRIDE → PACK**, wired to `self._send_command("override_open" / "override_pack")`.
- Enable them whenever connected: add to `_set_command_buttons_enabled`.
- Keep them enabled during a gesture/homing (they are the escape hatch) — they do
  not depend on any other UI state.
- Update the Help dialog: document the override as the priority reset-to-safe-state
  and note it aborts homing.

---

## Part D — Verification

1. **Build:** `pio run -e exp11_dynamic`; flash. `py_compile` the GUI.
2. **A1 (grip):** close a finger onto an object so it stalls ~mid-travel; confirm
   its Pos% holds at the real value and does NOT jump to 100%; telemetry Ripple
   matches the physical position.
3. **A2 (watchdog):** command a gesture whose target exceeds real travel (or block
   a finger indefinitely); confirm the phase times out, logs, and the queue does
   not wedge.
4. **A4 (boot):** power-cycle from a closed pose; confirm the hand drives open.
5. **Override during gesture:** start POWER, then `override_open` mid-move →
   hand immediately abandons POWER and opens; `override_pack` → fingers open then
   thumb closes.
6. **Override during homing:** start `home`, then `override_pack` → homing aborts
   cleanly and the hand goes to PACK in a defined state.
7. **Re-anchor:** after `override_open`, confirm all positions read 0 (open
   re-anchored); after `override_pack`, thumb reads ~100%, fingers ~0%.

## Out of scope (tracked, not in this plan)
- L1 absolute sensing (encoder) — the real long-term fix for drift.
- Current/force-terminated grip — removes object-stall ambiguity at the source.
- Continuous-invariant collision model to replace enumerated graph edges.
