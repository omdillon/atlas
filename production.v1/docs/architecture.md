# Architecture

End-to-end flow, camera to motors:

```
Webcam
  |
  v
hand_tracker.py       MediaPipe HandLandmarker (pretrained, off-the-shelf
                       model, not project-trained) extracts 21 hand
                       landmarks per frame, converts them to 5 continuous
                       per-finger curl percentages (PIP-angle for
                       index/middle/ring/pinky, distance-ratio for the
                       thumb), then thresholds each against a per-user
                       calibration (see docs/setup_software.md) into a
                       5-bit "finger closed" mask (bit0=thumb..bit4=pinky).
  |
  v
gesture_classifier.py Exact-match lookup: the 5-bit mask is compared
                       against a fixed table of 12 known gesture masks
                       (mirroring firmware/src/gesture_graph.h's
                       GRAPH_NODES table). No fuzzy/nearest-neighbour
                       matching: an in-between or mid-motion hand pose
                       classifies as "no gesture" rather than snapping to
                       the nearest real one. Noise rejection is the next
                       stage's job, not this one's.
  |
  v
cv_dispatch_gate.py    Temporal stability gate: a classified gesture must
                       hold steady for a configurable window (default
                       1.0s) and differ from the last gesture actually
                       sent, before it's allowed through. This is what
                       actually rejects noise/jitter; the classifier
                       stays deliberately "dumb".
  |
  v
telemetry.py           The GUI's `_send_command()`, the same code path
  (PC, BLE central)     used by the manual grasp buttons. CV-armed control
                       and manual buttons are just two callers of one
                       function; a manual click always takes effect
                       immediately.
  |
  v  BLE write, CHAR_MOTOR_CMD_UUID (ASCII text command)
  |
firmware main.cpp      Parses the command string (grasp name / per-finger
  (ESP32-S3)           open-close / safe override / wave / clear_faults)
                       and dispatches into the transition engine (or, for
                       `wave`, a standalone animation runner that bypasses
                       it).
  |
  v
transition_engine.*    Binary per-finger state machine (FINGER_OPEN /
                       FINGER_CLOSED / FINGER_OPENING / FINGER_CLOSING,
                       not a continuous position). Diffs the current
                       finger-closed bitmask against the target node's
                       mask, plans an "open everything that needs opening
                       first, then close in the target's declared phase
                       order" sequence, and drives it. A binary model has
                       no partial value to keep honest: a finger is
                       either settled (safe to trust) or actively
                       moving/interrupted (never trusted until it settles
                       again).
  |
  v
execMotorCommand()     Drives the DRV8214 motor drivers (5x, over I2C).
  + DRV8214 lib        Closing legs are stall-or-timeout terminated
                       (whichever comes first); opening legs are
                       timeout-only, since the hardware's stall/ripple
                       sensing is not reliable enough under an opening
                       (often spring-assisted) load, so an opening leg
                       always runs to its full timeout ceiling.
  |
  v
5x motors -> hand moves
  |
  v  BLE notify, CHAR_TELEMETRY_UUID, every ~60ms
  |
telemetry.py           Decodes the frame (see docs/protocol.md) and
  (GUI)                updates the Motor Telemetry table, Gesture Graph
                       panel (current/target node, live from the device,
                       not tracked client-side from "what did I last
                       click"), and Fault Detection table.
```

## Why the classifier is deliberately "dumb"

`gesture_classifier.py` does an exact match against a fixed table of known
masks, not a fuzzy or nearest-neighbour match. The firmware's own boot-time
check (`validateGraphNodes()`) guarantees these masks are mutually unique, so
an exact match is unambiguous by construction. A fuzzy matcher would risk
silently snapping an incomplete or in-transit hand pose to the nearest real
gesture, exactly the false-positive risk `cv_dispatch_gate.py`'s stability
window exists to prevent instead. Classification and noise-rejection are
kept as two separate, simple responsibilities rather than one smarter but
harder-to-reason-about one.

## Why "hand tracking training" is calibration, not machine learning

There is no trained model or labeled dataset anywhere in this system. The
MediaPipe hand-landmark detector is a generic pretrained third-party model,
unrelated to any specific gesture set. The only per-user "training" step is
`cv_calibrate.py`: it walks through all 12 known gestures, and because the
correct open/closed label for every finger in every gesture is already known
from the gesture mask table (no manual labeling needed), it samples curl%
values and computes a per-finger open/closed threshold as the midpoint
between the two. This threshold data (`cv_calibration.json`) is specific to
one person's hand, camera, and lighting, and is not shipped with this
repository; see `docs/setup_software.md`.

## Command source arbitration

Manual GUI buttons and camera-driven control both funnel through the same
`_send_command()` path on the PC side, and the same firmware command parser
on the device side; there is no separate "mode" the firmware has to track.
A manual button press always takes effect immediately; if the camera later
disagrees, it can only re-assert control once its own stability window
elapses again. Losing sight of the hand in-frame simply stops new
camera-driven commands; the physical hand holds its last commanded gesture
rather than doing anything on its own.
