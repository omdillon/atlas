"""Stage 1 sanity check: confirm a webcam is accessible before any hand-
tracking logic exists. Opens camera index 0 and shows a raw preview window.

Run: python cv_camera_check.py
Press 'q' or Esc in the preview window to quit.
"""

import cv2

CAMERA_INDEX = 0


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")

    print("Camera opened. Press 'q' or Esc in the preview window to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read from camera")
            cv2.imshow("Stage 1 camera check", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
