"""Per-finger closed-threshold calibration.

Firmware only cares about binary open/closed per finger, not depth, so this
doesn't need to reproduce true curl depth, only find, for each finger, a
curl% cutoff that lands on the correct side of "closed enough" for every
target gesture simultaneously (including gestures that hold a finger
shallow, e.g. TRIPOD, vs deep, e.g. POINTER, while other fingers of the
same gesture sit fully open or fully closed).

Walks through the gestures in TARGET_GESTURE_MASKS order, one at a time:
prompts you to form it and hold steady, samples curl% for a few seconds,
and buckets each finger's samples into "should be open" or "should be
closed" for that gesture (the label is already known from
TARGET_GESTURE_MASKS, no manual labeling needed). Once every gesture is
captured, each finger's threshold is the midpoint between its highest
"should be open" sample and its lowest "should be closed" sample.

If a finger's open-bucket and closed-bucket samples overlap, no valid flat
threshold separates them for that finger given this gesture set. This fails
loudly rather than silently writing a bad threshold.

Run: python cv_calibrate.py [--camera INDEX] [--sample-seconds N]
Writes: cv_calibration.json
"""

import argparse
import json
import time

import cv2

from gesture_classifier import TARGET_GESTURE_MASKS
from hand_tracker import CALIBRATION_PATH, FINGER_NAMES, HandTracker


def collect_samples(tracker, seconds, label):
    print(f"\nForm {label.upper()}, hold steady. Sampling for {seconds:.0f}s starting in 2s...")
    deadline_start = time.monotonic() + 2.0
    while time.monotonic() < deadline_start:
        frame, _curl_pcts, _detected = tracker.step()
        cv2.putText(frame, f"Get ready: {label.upper()}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        cv2.imshow("cv_calibrate.py", frame)
        cv2.waitKey(1)

    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame, curl_pcts, detected = tracker.step()
        if detected:
            samples.append(curl_pcts)
        remaining = deadline - time.monotonic()
        cv2.putText(frame, f"SAMPLING {label.upper()}  {remaining:.1f}s left  ({len(samples)} frames)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("cv_calibrate.py", frame)
        cv2.waitKey(1)

    if not samples:
        raise RuntimeError(
            f"No hand detected during {label} sampling window, redo with the hand clearly in frame."
        )
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--sample-seconds", type=float, default=2.5)
    args = parser.parse_args()

    tracker = HandTracker(camera_index=args.camera)
    tracker.open()

    # For each finger, collect the curl% samples where the target mask says
    # it should be open, and separately where it should be closed, across
    # every gesture.
    open_samples = [[] for _ in range(5)]
    closed_samples = [[] for _ in range(5)]

    try:
        for gesture, mask in TARGET_GESTURE_MASKS.items():
            samples = collect_samples(tracker, args.sample_seconds, gesture)
            for finger_i in range(5):
                is_closed_target = bool(mask & (1 << finger_i))
                bucket = closed_samples[finger_i] if is_closed_target else open_samples[finger_i]
                bucket.extend(s[finger_i] for s in samples)
    finally:
        tracker.close()
        cv2.destroyAllWindows()

    thresholds = []
    problems = []
    for i, name in enumerate(FINGER_NAMES):
        if not open_samples[i] or not closed_samples[i]:
            problems.append(
                f"{name}: never appears in both an open-target and closed-target gesture "
                f"in TARGET_GESTURE_MASKS, cannot calibrate a threshold from this gesture set alone."
            )
            thresholds.append(None)
            continue

        open_max = max(open_samples[i])
        closed_min = min(closed_samples[i])

        if open_max >= closed_min:
            problems.append(
                f"{name}: open-bucket max ({open_max:.1f}%) >= closed-bucket min ({closed_min:.1f}%), "
                f"buckets overlap, no valid flat threshold separates them. "
                f"Re-run calibration with clearer, more distinct hand poses for this finger."
            )
            thresholds.append(None)
            continue

        thresholds.append((open_max + closed_min) / 2.0)

    if problems:
        print("\nCALIBRATION FAILED, not writing cv_calibration.json:")
        for p in problems:
            print(f"  - {p}")
        raise SystemExit(1)

    print("\nCalibrated thresholds (curl% at/above which a finger counts as CLOSED):")
    for name, t in zip(FINGER_NAMES, thresholds):
        print(f"  {name}: {t:.1f}%")

    with open(CALIBRATION_PATH, "w") as f:
        json.dump(
            {
                "thresholds": thresholds,
                "finger_order": FINGER_NAMES,
                "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "notes": "Recalibrate if lighting, camera position, or camera hardware changes materially.",
            },
            f,
            indent=2,
        )
    print(f"\nWrote {CALIBRATION_PATH}")
    print("Re-run cv_dry_run.py to verify all gestures now classify correctly.")


if __name__ == "__main__":
    main()
