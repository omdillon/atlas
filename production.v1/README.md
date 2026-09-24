# Hand Teleoperation System

A webcam-controlled prosthetic hand: a camera reads your hand pose,
classifies it into one of a fixed set of gestures, and drives a
BLE-connected ESP32-S3. A desktop GUI provides manual control, live
telemetry, and fault monitoring alongside the camera-driven control path.

## Quick start

1. Flash the firmware
2. Set up the Python environment and run the one-time hand/camera calibration
3. Run `software/telemetry.py`, click **Connect**, and either use the grasp
   buttons or arm **Hand Tracking (CV)** to control the hand with your webcam.

## How it works

See [`docs/architecture.md`](docs/architecture.md) for the full pipeline and
[`docs/protocol.md`](docs/protocol.md) for the exact BLE wire format.

