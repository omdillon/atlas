"""Temporal gate between a per-frame classified gesture (or None) and an
actual BLE dispatch decision. One state machine, two responsibilities:

1. Stability: a classified gesture must be the same candidate for a full
   `stability_seconds` window before it's eligible to dispatch. Any change,
   including a drop to None (tracking loss or an ambiguous pose), resets
   the window. This also implements "hold last commanded gesture on lost
   tracking" for free: a lost hand just stops producing a new stable
   candidate, so the physical hand sits at whatever it was last told to do.

2. Dedupe: even once stable, a candidate only dispatches if it differs from
   the last gesture actually sent, by any source (CV or a manual click).

There's no separate "on tracking lost" handler: a lost hand is just another
frame with candidate=None, already covered by rule 1. Recognizing
tracking-loss for HUD/display purposes is left to the caller (see
hand_tracker.HandTracker.step()'s `hand_detected` return value).
"""

from typing import Optional


class DispatchGate:
    def __init__(self, stability_seconds: float):
        self.stability_seconds = stability_seconds
        self._stable_candidate: Optional[str] = None
        self._stable_since: Optional[float] = None
        self._last_dispatched: Optional[str] = None

    def update(self, candidate: Optional[str], now: float) -> Optional[str]:
        """Call once per frame. Returns the gesture name to dispatch this
        call, or None if nothing should be sent right now."""
        if candidate != self._stable_candidate:
            self._stable_candidate = candidate
            self._stable_since = now
            return None

        if candidate is None:
            return None

        if now - self._stable_since < self.stability_seconds:
            return None

        if candidate == self._last_dispatched:
            return None

        self._last_dispatched = candidate
        return candidate

    def status(self, now: float):
        """Read-only snapshot for display purposes (HUD/GUI status labels).
        Returns (stable_candidate, seconds_remaining_in_window, last_dispatched).
        seconds_remaining is 0.0 once the window has already elapsed."""
        remaining = 0.0
        if self._stable_candidate is not None and self._stable_since is not None:
            remaining = max(0.0, self.stability_seconds - (now - self._stable_since))
        return self._stable_candidate, remaining, self._last_dispatched

    def sync_external_command(self, gesture: Optional[str]) -> None:
        """Call whenever a command reaches the hand via a path other than
        this gate's own `update()` return value: a manual GUI click, an
        override, WAVE. Updates dedupe memory only, leaving
        `_stable_candidate`/`_stable_since` untouched, so CV's own live read
        of the camera keeps running and can still reassert itself once its
        own stability window elapses (the "momentary nudge" behavior)."""
        self._last_dispatched = gesture


if __name__ == "__main__":
    # Self-test: synthetic (candidate, now) sequences, no camera/BLE/hardware.
    failures = []

    def check(label, got, expected):
        if got != expected:
            failures.append(f"{label}: expected {expected!r}, got {got!r}")

    X = 1.0

    # 1. Stable past X, exactly one dispatch.
    g = DispatchGate(X)
    r0 = g.update("power", 0.0)
    r1 = g.update("power", 0.5)
    r2 = g.update("power", 1.0)
    r3 = g.update("power", 1.5)
    check("1a: t=0.0 first sight", r0, None)
    check("1b: t=0.5 still within window", r1, None)
    check("1c: t=1.0 window just elapsed", r2, "power")
    check("1d: t=1.5 already dispatched, no re-send", r3, None)

    # 2. Never-stable (flickering) -> zero dispatches.
    g = DispatchGate(X)
    results = [
        g.update("open", 0.0),
        g.update("power", 0.3),
        g.update("open", 0.6),
        g.update("power", 0.9),
        g.update("open", 1.2),
    ]
    check("2: flickering candidate never dispatches", any(r is not None for r in results), False)

    # 3. Stable at A, dispatched, remains at A -> no further dispatches.
    g = DispatchGate(X)
    g.update("tripod", 0.0)
    r = g.update("tripod", 1.0)
    check("3a: first stable dispatch", r, "tripod")
    r2 = g.update("tripod", 2.0)
    r3 = g.update("tripod", 3.0)
    check("3b: still tripod, no re-dispatch", (r2, r3), (None, None))

    # 4. A -> None -> B, each held past X -> two correctly-spaced dispatches.
    g = DispatchGate(X)
    g.update("pack", 0.0)
    rA = g.update("pack", 1.0)
    check("4a: A dispatches", rA, "pack")
    g.update(None, 1.1)   # hand leaves frame / ambiguous pose
    g.update("rock", 1.2)  # new candidate B appears
    r_mid = g.update("rock", 1.8)  # not yet stable for a full window
    rB = g.update("rock", 2.2)     # 2.2 - 1.2 = 1.0s stable
    check("4b: mid-window no dispatch", r_mid, None)
    check("4c: B dispatches once stable", rB, "rock")

    # 5. Mid-window drop to None resets the timer without corrupting dedupe.
    g = DispatchGate(X)
    g.update("open", 0.0)
    r = g.update("open", 1.0)
    check("5a: open dispatches", r, "open")
    g.update(None, 1.3)          # brief tracking loss
    g.update("open", 1.4)        # candidate returns to open (still == last_dispatched)
    r2 = g.update("open", 2.4)   # stable for a full window again
    check("5b: re-stabilizing at the SAME already-dispatched gesture does not re-send", r2, None)

    # 6. sync_external_command: a manual click updates dedupe memory without
    # disturbing an in-progress CV stability window (the "momentary nudge").
    g = DispatchGate(X)
    g.update("pinch", 0.0)  # CV starts tracking toward pinch
    g.sync_external_command("power")  # user manually clicks POWER at t=0.3
    r_mid = g.update("pinch", 0.5)  # CV's own window is untouched by the manual click
    check("6a: manual click doesn't reset CV's own stability timer", r_mid, None)
    r_final = g.update("pinch", 1.0)  # CV's window (measured from t=0.0) elapses
    check("6b: CV reasserts pinch once its window elapses, differs from manual power", r_final, "pinch")

    if failures:
        print("FAILED:")
        for f in failures:
            print(f"  {f}")
        raise SystemExit(1)

    print("OK: all 6 dispatch-gate scenarios behave as designed.")
