# Software setup

## Environment

From `software/`:

```
python -m venv .venv
.venv\Scripts\activate       # Windows; use `source .venv/bin/activate` on macOS/Linux


pip install -r requirements.txt
```

This installs `bleak` (BLE), `PyQt6` + `qasync` (GUI), and `opencv-python` +
`mediapipe` + `numpy` (the hand-tracking pipeline).

## First-time setup order

Run these in order the first time you set up a new hand/camera rig:

1. **`python cv_camera_check.py`**: confirms
   OpenCV can open your camera before anything else is layered on top.

2. **`python cv_calibrate.py`**: one-time per-user, per-camera, per-lighting
   calibration. Walks you through forming each of the 12 known gestures in
   turn, samples your hand's curl percentages, and computes per-finger
   open/closed thresholds. Writes `cv_calibration.json`. **This file is
   specific to you and your setup; it is not shipped in this repository and
   should be regenerated any time your camera position, lighting, or hand
   changes noticeably.** The script fails loudly (raises, does not write a
   file) if it can't find a clean separation between "open" and "closed"
   samples for some finger; that's expected behaviour, not a bug, and means
   you should retry with a clearer/steadier hold.

3. **`python cv_dry_run.py`**: runs the full tracking -> classification ->
   dispatch-gate pipeline against your calibration, with a live preview
   window, printing what it *would* send; no BLE, no hardware required. Use
   this to confirm calibration is good before ever arming CV control against
   a real hand.

4. **`python telemetry.py`**: the full GUI. Click **Connect** to pair with a
   flashed board (see [`setup_firmware.md`](setup_firmware.md)). Manual grasp
   buttons work immediately; click **ARM CV CONTROL** to enable the webcam as
   an alternate command source (unarmed by default, as a safety backstop for
   a feature that can move the hand on its own).

See [`architecture.md`](architecture.md) for how these pieces fit together
and [`protocol.md`](protocol.md) for the BLE wire format `telemetry.py` speaks.
