"""End-to-end CV pipeline dry run: HandTracker -> finger_states/
states_to_mask -> classify_mask -> DispatchGate, all the way to a
would-dispatch decision, but with no BLE connection at all. Nothing can
move a physical hand from this script; it exists purely to characterize
classification accuracy and dispatch timing at a desk with a webcam before
any hardware is involved.

Run: python cv_dry_run.py [--stability SECONDS] [--camera INDEX]
Press 'q' or Esc to quit.

Loads cv_calibration.json if present (see cv_calibrate.py) for per-finger
thresholds, else falls back to the flat default baseline. Expect some
misclassification on the flat baseline; that's the point of running this
before calibrating, not a bug.
"""

import argparse
import time

import cv2

from cv_dispatch_gate import DispatchGate
from gesture_classifier import classify_mask
from hand_tracker import (
    FINGER_NAMES,
    HandTracker,
    finger_states,
    load_thresholds,
    states_to_mask,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stability", type=float, default=1.0,
                         help="Stability window in seconds before a would-dispatch fires (default 1.0)")
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()

    thresholds = load_thresholds()
    tracker = HandTracker(camera_index=args.camera)
    gate = DispatchGate(stability_seconds=args.stability)

    tracker.open()
    print(f"Stability window: {args.stability}s. Press 'q' or Esc to quit.")
    try:
        while True:
            now = time.monotonic()
            frame, curl_pcts, detected = tracker.step()

            states = finger_states(curl_pcts, thresholds)
            mask = states_to_mask(states)
            candidate = classify_mask(mask)

            dispatched = gate.update(candidate, now)
            if dispatched is not None:
                print(f"[WOULD SEND] {dispatched.upper()}")

            stable_candidate, remaining, last_dispatched = gate.status(now)

            curl_str = " ".join(
                f"{name[0].upper()}:{pct:3.0f}" for name, pct in zip(FINGER_NAMES, curl_pcts)
            )
            status_line = (
                f"{curl_str}  mask:0x{mask:02X}  "
                f"{'HAND' if detected else 'no hand'}  "
                f"candidate:{stable_candidate or '-'}  "
                f"hold:{remaining:.1f}s  "
                f"last_sent:{last_dispatched or '-'}"
            )
            cv2.putText(frame, status_line, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            cv2.imshow("cv_dry_run.py (no BLE, nothing can move)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        tracker.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
