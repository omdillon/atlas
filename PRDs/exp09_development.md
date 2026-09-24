# EXP-09 as Fundamentals — Architecture Decomposition & Development Map

Purpose: strip the RoboLimb firmware down to its fundamental layers so future
development targets the right seam. Compares the clean EXP-09 baseline
(`src/exp09_refined/hand.cpp`) against the EXP-11 iteration
(`src/exp11_dynamic/main.cpp`) and shows **where** the leverage for real
improvement actually lives.

---

## 1. The layer model

Strip the names away and EXP-09 is a **5-layer stack over one actuation
primitive**:

| Layer | Responsibility (EXP-09) | Key symbol | State |
|-------|-------------------------|------------|-------|
| **L0 — Actuation** | Drive / brake ONE motor. The single choke point every motor action passes through; owns per-motor bookkeeping. | `execMotorCommmand()` `hand.cpp:941` | **Solid** |
| **L1 — Position sensing** | Absolute finger position by *dead reckoning*: bank each ripple leg (signed by direction) at every direction change. | `flushRippleTravel()`, `liveMotorPosition()` `hand.cpp:263` | **Fundamental weakness** |
| **L2 — Per-motor closed loop** | Drive a single finger toward a target; brake on reach or stall. | `startFingerMove` / `updateFingerMoves` | Sound |
| **L3 — Coordination** | A gesture = a **timed FIFO** of L0 commands, assumed to start from OPEN. | `queueGrasp` + `motorQueue` | Open-loop, start-state-dependent |
| **L4 — Planning / safety** | *(none)* — the human "returns to open" between gestures. | — | Absent |
| **X — Stall detection** | Ripple-stagnation → `MOTOR_STALLED`. | `updateStallDetection` | Cross-cutting |
| **X — Telemetry / BLE** | 77-byte frame; raw ripple position. | `build_telemetry_frame` | Cross-cutting |

### The two load-bearing facts

- **L0 is solid.** One function, correct bookkeeping (`flushRippleTravel` banks
  the ending leg, resets the counter, records direction + state + re-arms stall).
  Everything hangs off it cleanly — it is the right primitive.
- **L1 is the fundamental weakness.** Position is *open-loop dead reckoning*: the
  DRV8214 ripple counter is magnitude-only and reset at every reversal, so error
  accumulates and is worst near stalls (the regime the config itself calls "NOT
  RELIABLE ENOUGH"). **In EXP-09 this barely matters** — position is *cosmetic*,
  consumed only by the telemetry Pos% column. Nothing *controls* on it.

---

## 2. What EXP-11 changed, mapped onto the layers

| Layer | EXP-11 change | Verdict |
|-------|---------------|---------|
| **L1** | `fullClosure()` accessor; homing state machine; NVS calibration; **stall-anchored re-homing** (snap to endpoint); `g_drive_current` for gentle homing | Right layer — but the snap introduced a **new bug** (§4) |
| **L2** | Generalised `startFingerMove` → `startMotorMove(idx, target)` (arbitrary target) `main.cpp:454` | **Sound** — clean generalisation |
| **L3** | Timed FIFO → **posture phases** (`MotionPhase` = mask + per-finger pct; `phaseQueue`; `updateGestureQueue`) | Sound direction; missing a watchdog |
| **L4** | **New** transition graph (`GRAPH_NODES`, `edgeAllowed`, BFS `graphFindPath`, `currentGraphNode`, `planAndRunTransition`) | Heaviest, least-validated, most bug-prone |
| **X-tel** | Normalised 0–10000 position **plus** raw ripple → 97-byte frame | Sound |

### The structural shift that matters

EXP-11 made **position control-critical.** L3 postures, L4 localisation, and the
L2 idempotence checks now all *consume `liveMotorPosition` as ground truth* —
while L1 stayed dead-reckoned. That single mismatch — trusting a value that was
only ever a cosmetic estimate — is where EXP-11's bugs cluster.

---

## 3. The fundamental fault line

```
EXP-09:  position is COSMETIC      → L1 error shows up only as a wrong Pos% readout
EXP-11:  position is CONTROL-GRADE → L1 error shows up as wrong motion, wrong route,
                                      corrupted state, wedged queues
```

EXP-11 built **up** the stack (L3 → L4) on an **unhardened L1**. That is
backwards: leverage runs **down** the stack, not up.

---

## 4. Concrete bugs — and why they are all the same bug

- **Object grip corrupts position** — `main.cpp:517`. A full-close posture
  (target = `fullClosure`) that *stalls on an object at 60%* still hits the snap
  `finger_move_target >= fullClosure` → `motor_position = fullClosure`. The finger
  now reports 100% closed while physically at 60%. (EXP-09 never had this — the
  snap is a NEW EXP-11 addition meant to fix drift that instead conflates
  "stalled at the endstop" with "stalled on an object".)
- **Graph mislocalisation** — `main.cpp:1300`. `currentGraphNode` matches live
  position to a node within 15%; feed it the corrupted/drifted position and it
  localises to the wrong node → plans an unsafe or nonsensical route.
- **No phase-completion watchdog** — `main.cpp:1216`. `updateGestureQueue` waits
  on `finger_move_active` with no timeout. EXP-09's timed FIFO was time-bounded
  and could not hang; EXP-11 can wedge the whole gesture queue if a finger
  neither reaches target nor stalls.
- **Active unvalidated edge** — `main.cpp:1271`. `DIRECT_EDGES = { TRIPOD↔PINCH }`
  ships enabled with a "bench-validate before trusting" TODO — a latent collision
  path.
- **No physical open at boot** — `setup()`'s `applyGesture(OPEN)` is idempotent at
  the assumed boot position 0, so unlike EXP-09's forced timed-open the hand may
  not actually open.

Every one of these traces to L1: **position is trusted but not true.**

---

## 5. Where fundamental changes should be made (leverage, ordered)

Developing *from* EXP-09 is the right instinct — it is the clean foundation.
Leverage runs **down** the stack:

1. **L1 — make position trustworthy (highest leverage).** Everything else is only
   as good as this. Cheap path: strengthen re-anchoring but fix the conflation —
   only snap when the stall is *at* the expected endpoint, never for a mid-travel
   object stall; re-anchor at *both* ends during normal use. Real path: an
   absolute encoder. Until L1 is honest, L3/L4 keep producing "bugs" that are
   really propagated position error.
2. **L0 / L2 — actuation quality.** Two untapped hardware primitives already on
   the board: **current/force-terminated closing** (grip to a force, not a
   position — sidesteps the object-stall problem entirely) and the DRV8214's
   **hardware ripple-threshold auto-stop** for crisp position stops. L2 is
   bang-bang (drive full, brake) → overshoot; a soft-stop/proportional finish
   belongs here.
3. **L3 — coordination.** Posture phases are the right model; add a **per-phase
   watchdog** so a stuck finger cannot wedge the queue (restores the one
   robustness property the timed FIFO had for free).
4. **L4 — safety.** Heaviest layer; depends entirely on L1 being honest
   (localisation). Build it *last*. A continuous-invariant guard covers off-node
   states better than enumerated edges.

**Bottom line:** EXP-09's fundamentals are `execMotorCommmand` (L0, solid) and
dead-reckoned `motor_position` (L1, the weak link). The single highest-value
development is turning L1 from a *cosmetic estimate* into *control-grade truth* —
do that and most of EXP-11's bugs dissolve rather than needing individual patches.
