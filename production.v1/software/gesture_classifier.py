"""Maps a 5-bit finger-closed mask (bit i = finger i CLOSED, matching
firmware's gesture_graph.h convention) onto one of the known graph-node
gesture names, or None if the mask doesn't exactly match any of them.

TARGET_GESTURE_MASKS must be kept in manual sync with
firmware/src/gesture_graph.h's GRAPH_NODES table; there is no shared source
of truth between the C++ header and this Python module.

Deliberately an exact-match lookup, not a nearest-neighbour/fuzzy match:
firmware's validateGraphNodes() guarantees these masks are mutually unique,
so exact match is unambiguous by construction. A fuzzy matcher would risk
silently snapping an incomplete/in-transit hand pose to the nearest real
gesture, which is exactly the false-positive risk cv_dispatch_gate.py's
stability window exists to prevent.

WAVE is deliberately absent: it has no GRAPH_NODES entry (a scripted
animation, not a static pose) and can't be detected from one sampled hand
shape.
"""

from typing import Optional

# gesture -> fingerMask, verbatim from firmware/src/gesture_graph.h's
# GRAPH_NODES table (bit0=thumb, bit1=index, bit2=middle, bit3=ring,
# bit4=pinky; bit=1 means closed).
TARGET_GESTURE_MASKS = {
    "open": 0x00,
    "power": 0x1F,
    "tripod": 0x07,
    "pinch": 0x03,
    "pointer": 0x1D,
    "rock": 0x0D,
    "pack": 0x01,
    # CV-teleoperation expansion set; see gesture_graph.h's BIRD..THUMB rows.
    "bird": 0x1B,
    "two": 0x19,
    "jambo": 0x0E,
    "three": 0x11,
    "thumb": 0x1E,
}

MASK_TO_GESTURE = {mask: name for name, mask in TARGET_GESTURE_MASKS.items()}

# Mirrors firmware's validateGraphNodes() uniqueness check at import time; if
# this ever fires, TARGET_GESTURE_MASKS has drifted out of sync with
# gesture_graph.h and classification would be ambiguous.
assert len(MASK_TO_GESTURE) == len(TARGET_GESTURE_MASKS), (
    "TARGET_GESTURE_MASKS has a duplicate mask value; check against "
    "firmware/src/gesture_graph.h's GRAPH_NODES"
)


def classify_mask(mask: int) -> Optional[str]:
    """Returns the lowercase gesture name (matching firmware's accepted BLE
    command strings, e.g. "tripod") for an exact fingerMask match, or None if
    `mask` doesn't correspond to any known gesture.

    None is the expected, normal result while a hand is mid-motion between
    poses, not an error condition.
    """
    return MASK_TO_GESTURE.get(mask)


if __name__ == "__main__":
    # Self-test: no camera, no BLE, just proves the lookup table is correct
    # and unambiguous.
    failures = []

    for name, mask in TARGET_GESTURE_MASKS.items():
        got = classify_mask(mask)
        if got != name:
            failures.append(f"mask 0x{mask:02X}: expected {name!r}, got {got!r}")

    invalid_masks_to_check = [0x02, 0x04, 0x08, 0x10, 0x1E, 0x1B, 0x15, 0x09, 0x11]
    for mask in invalid_masks_to_check:
        if mask in TARGET_GESTURE_MASKS.values():
            continue  # skip if it accidentally collides with a real gesture
        got = classify_mask(mask)
        if got is not None:
            failures.append(f"mask 0x{mask:02X}: expected None, got {got!r}")

    if failures:
        print("FAILED:")
        for f in failures:
            print(f"  {f}")
        raise SystemExit(1)

    print(f"OK: all {len(TARGET_GESTURE_MASKS)} valid gestures classify correctly, "
          f"all {len(invalid_masks_to_check)} sampled invalid masks return None.")
