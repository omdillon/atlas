"""Webcam hand tracking: MediaPipe landmarks -> per-finger curl % -> a
binary open/closed mask matching firmware's bit-i-CLOSED convention.

finger_states() packs bit=1 for CLOSED, matching firmware's gesture_graph.h
convention ("bit i = finger i CLOSED") exactly, so states_to_mask() needs no
further inversion.

HandTracker's only job is turning camera frames into curl percentages; it
does no debouncing or thresholding of its own. Temporal stability is
cv_dispatch_gate.DispatchGate's job, and thresholds are calibrated data
(cv_calibration.json, see cv_calibrate.py), not something a generic
camera-reading class should bake in.

Not thread-safe; drive from a single thread/QTimer, never concurrently.
"""

import json
import math
import os
import sys
import time
import urllib.request

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    HandLandmarker,
    HandLandmarkerOptions,
    HandLandmarksConnections,
    RunningMode,
)

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)

# Finger index convention: 0=thumb, 1=index, 2=middle, 3=ring, 4=pinky,
# matches firmware/src/gesture_graph.h's FINGER_NAMES exactly.
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
HAND_CONNECTIONS = [(c.start, c.end) for c in HandLandmarksConnections.HAND_CONNECTIONS]

# Continuous per-digit curl (0=fully extended, 100=fully curled).
# Fingers (index/middle/ring/pinky): PIP-joint angle, MCP->PIP->TIP, skipping
# DIP. Scale-invariant. (MCP, PIP, TIP) landmark index tuples, MediaPipe's
# 21-point topology.
FINGER_ANGLE_JOINTS = [(5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)]  # index, middle, ring, pinky
FULL_EXTEND_ANGLE_DEG = 160.0  # PIP angle at a straight finger
FULL_CURL_ANGLE_DEG = 60.0     # PIP angle at a tightly curled finger; both hand-tuned, not calibrated

# Thumb: distance-ratio metric instead of an angle (thumb opposition doesn't
# project cleanly onto a 2D hinge angle the way the other fingers' PIP does).
THUMB_TIP, THUMB_MCP = 4, 2
PALM_REF = 17  # pinky MCP knuckle, a stable "far side of palm" reference
THUMB_RATIO_EXTENDED = 1.4  # tip much farther than MCP from PALM_REF
THUMB_RATIO_CURLED = 0.8    # tip about as close as, or closer than, MCP; both hand-tuned, not calibrated

# Default flat threshold, overridden by calibrated per-finger values once
# cv_calibration.json exists.
DEFAULT_CLOSED_THRESHOLD_PCT = 50.0

CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv_calibration.json")


def load_thresholds(quiet=False):
    """Loads per-finger closed-thresholds from cv_calibration.json (see
    cv_calibrate.py) if present, else falls back to a flat
    DEFAULT_CLOSED_THRESHOLD_PCT for all 5 fingers. Shared by cv_dry_run.py
    and telemetry.py so calibration-loading logic exists in exactly one
    place."""
    if not os.path.exists(CALIBRATION_PATH):
        if not quiet:
            print(f"No calibration file at {CALIBRATION_PATH}, using flat "
                  f"{DEFAULT_CLOSED_THRESHOLD_PCT}% baseline for all 5 fingers.")
        return (DEFAULT_CLOSED_THRESHOLD_PCT,) * 5
    with open(CALIBRATION_PATH, "r") as f:
        data = json.load(f)
    thresholds = tuple(data["thresholds"])
    if not quiet:
        print(f"Loaded calibrated thresholds from {CALIBRATION_PATH}: {thresholds}")
    return thresholds


def _dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _angle_deg(a, b, c):
    """Angle at vertex b (degrees), between rays b->a and b->c."""
    v1x, v1y = a.x - b.x, a.y - b.y
    v2x, v2y = c.x - b.x, c.y - b.y
    mag = math.hypot(v1x, v1y) * math.hypot(v2x, v2y)
    if mag < 1e-9:
        return FULL_EXTEND_ANGLE_DEG
    cos_angle = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / mag))
    return math.degrees(math.acos(cos_angle))


def finger_curl_percentages(landmarks):
    """Returns 5 continuous curl values (0=extended, 100=fully curled) in
    [thumb, index, middle, ring, pinky] order.
    """
    ratio = _dist(landmarks[THUMB_TIP], landmarks[PALM_REF]) / max(
        _dist(landmarks[THUMB_MCP], landmarks[PALM_REF]), 1e-6
    )
    thumb_frac = (THUMB_RATIO_EXTENDED - ratio) / (THUMB_RATIO_EXTENDED - THUMB_RATIO_CURLED)
    percentages = [max(0.0, min(100.0, thumb_frac * 100.0))]

    for mcp_id, pip_id, tip_id in FINGER_ANGLE_JOINTS:
        angle = _angle_deg(landmarks[mcp_id], landmarks[pip_id], landmarks[tip_id])
        frac = (FULL_EXTEND_ANGLE_DEG - angle) / (FULL_EXTEND_ANGLE_DEG - FULL_CURL_ANGLE_DEG)
        percentages.append(max(0.0, min(100.0, frac * 100.0)))

    return percentages


def finger_states(curl_percentages, thresholds=None):
    """Boolean CLOSED/open per finger: bit=True means CLOSED (`pct >=
    threshold`), matching firmware's gesture_graph.h convention ("bit i =
    finger i CLOSED"), so states_to_mask() below produces a mask directly
    comparable to firmware's fingerMask values with no further inversion.

    thresholds: optional per-finger threshold vector (5 floats), same
    [thumb, index, middle, ring, pinky] order. Defaults to a flat
    DEFAULT_CLOSED_THRESHOLD_PCT for all 5 fingers if not given; pass
    cv_calibrate.py's calibrated thresholds here once available.
    """
    if thresholds is None:
        thresholds = (DEFAULT_CLOSED_THRESHOLD_PCT,) * 5
    return [pct >= t for pct, t in zip(curl_percentages, thresholds)]


def states_to_mask(states):
    """Packs 5 booleans (True=CLOSED) into a mask, bit0=thumb..bit4=pinky,
    directly comparable to gesture_graph.h's GRAPH_NODES[].fingerMask."""
    mask = 0
    for i, closed in enumerate(states):
        if closed:
            mask |= 1 << i
    return mask


def ensure_model():
    if os.path.exists(MODEL_PATH):
        return
    print(f"Downloading HandLandmarker model to {MODEL_PATH} ...")
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Model downloaded.")
    except OSError as exc:
        print(f"Failed to download model: {exc}")
        print(f"Manually download it from {MODEL_URL} and save it as {MODEL_PATH}")
        sys.exit(1)


def draw_landmarks(frame, landmarks):
    h, w = frame.shape[:2]
    points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, points[start], points[end], (0, 200, 0), 2)
    for x, y in points:
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)


class HandTracker:
    """Owns a webcam + MediaPipe HandLandmarker lifecycle.

    No transport assumption; callers decide what to do with the curl values
    returned by step(). Not thread-safe; drive from a single thread/QTimer,
    never concurrently.

    Lifecycle: open() once, step() every tick, close() once when done.
    """

    def __init__(self, camera_index=0, min_hand_detection_confidence=0.7,
                 min_tracking_confidence=0.5):
        self.camera_index = camera_index
        self._min_det_conf = min_hand_detection_confidence
        self._min_track_conf = min_tracking_confidence
        self._cap = None
        self._landmarker = None
        self._start_time = None
        self._last_curl_pcts = [0.0] * 5

    def open(self):
        """Open the camera and MediaPipe model.

        Raises RuntimeError on camera-open failure (caller decides how to
        surface that, e.g. a GUI status label).
        """
        ensure_model()
        self._cap = cv2.VideoCapture(self.camera_index)
        if not self._cap.isOpened():
            self._cap = None
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=self._min_det_conf,
            min_tracking_confidence=self._min_track_conf,
        )
        self._landmarker = HandLandmarker.create_from_options(options)
        self._start_time = time.monotonic()
        self._last_curl_pcts = [0.0] * 5

    def step(self):
        """Read and process exactly one frame.

        Returns (frame_bgr, curl_percentages, hand_detected):
          - frame_bgr: the BGR frame (flipped, landmarks drawn on it if a
            hand was detected this frame)
          - curl_percentages: 5 floats 0-100 (0=extended, 100=fully curled),
            [thumb, index, middle, ring, pinky]. On a frame with no hand
            detected, holds the last known values rather than snapping to 0,
            so a momentary tracking drop-out doesn't look like the hand
            snapped open, and a classifier built on this naturally keeps
            reporting the same (already-dispatched) gesture through a brief
            drop-out instead of spuriously flipping to "no match".
          - hand_detected: bool, whether MediaPipe found a hand THIS frame.
            Callers that want to distinguish "actively tracking" from
            "coasting on a held reading" for display purposes use this;
            it does not by itself affect curl_percentages or need to affect
            dispatch logic (see cv_dispatch_gate.py).

        Raises RuntimeError if the camera read fails (caller should stop).
        """
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("Failed to read from camera")

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.monotonic() - self._start_time) * 1000)
        result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

        hand_detected = bool(result.hand_landmarks)
        if hand_detected:
            landmarks = result.hand_landmarks[0]
            self._last_curl_pcts = finger_curl_percentages(landmarks)
            draw_landmarks(frame, landmarks)

        return frame, list(self._last_curl_pcts), hand_detected

    def close(self):
        """Idempotent: safe to call more than once."""
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None


if __name__ == "__main__":
    # Smoke test: live curl%/mask printout + preview window. No
    # classification, no BLE, just proves landmarks -> curl% -> mask
    # extraction works and the CLOSED convention is correct. With an open
    # hand in frame, mask should read 0x00; with a closed fist, 0x1F.
    tracker = HandTracker()
    tracker.open()
    print("Press 'q' or Esc to quit.")
    try:
        while True:
            frame, curl_pcts, detected = tracker.step()
            states = finger_states(curl_pcts)
            mask = states_to_mask(states)
            curl_str = " ".join(
                f"{name[0].upper()}:{pct:3.0f}" for name, pct in zip(FINGER_NAMES, curl_pcts)
            )
            label = f"{curl_str}  mask:0x{mask:02X}  {'HAND' if detected else 'no hand'}"
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("hand_tracker.py smoke test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        tracker.close()
        cv2.destroyAllWindows()
