#pragma once
#include <Arduino.h>
#include "gesture_graph.h"

// Binary, stall-terminated transition engine.
//
// Each finger's tracked state is binary (FingerState below), not a
// continuous commanded-time depth. Closing ends when either the stall
// watchdog confirms the finger has stopped (notifyStall()) or a per-leg
// timeout elapses (gesture_graph.h's closeDurationMs), whichever comes
// first. Opening is timeout-only: notifyStall() is a no-op for an opening
// leg, so only FINGER_FULL_OPEN_MS ends it. Both routes funnel through the
// same completion function (completeLeg(), in transition_engine.cpp) so
// they can never diverge.
//
// This replaces an earlier design that tracked each finger's position as a
// continuous commanded-drive-time integral, which required every possible
// interruption to correctly bank partial credit for elapsed-but-incomplete
// drive time; every bug found in that design traced back to a path that
// failed to keep that credit honest. A binary model has no partial value to
// keep honest: a finger is either settled (OPEN/CLOSED, safe to trust) or
// actively moving or interrupted (OPENING/CLOSING, never trusted until it
// settles again).
//
// Actuation is delegated to two callbacks so this module has no dependency
// on motor hardware or BLE.

// OPEN/CLOSED: settled, safe to treat as ground truth.
// OPENING/CLOSING: being driven, or was driven and got interrupted before
// completion; not safe to treat as arrived either way. Only completeLeg()
// (transition_engine.cpp) promotes a finger to a settled value.
enum FingerState : uint8_t { FINGER_OPEN, FINGER_CLOSED, FINGER_OPENING, FINGER_CLOSING };

typedef void (*DriveFingerFn)(uint8_t fingerIdx, bool closing);   // start driving; engine owns completion
typedef void (*BrakeFingerFn)(uint8_t fingerIdx);                 // stop + hold now

// A transition plans as at most 1 (open) + MAX_PHASES (close) phases:
//   phase 0    - every finger that needs to open, run together (opening
//                never creates a new collision, so it's always safe first).
//   phase 1..N - the target node's declared phaseMask order, filtered to
//                fingers that still need to close.
static const uint8_t ENGINE_MAX_PHASES = 1 + MAX_PHASES;

// Sentinel _closedVia[] value meaning "this finger's closed position was not
// reached by driving toward a graph node" (i.e. requestFingerMoves() or
// WAVE's reportFingerState()). Outside graspName's real range so it can
// never collide with an actual gesture identity.
static const int8_t NO_GRAPH_TARGET = -2;

class TransitionEngine {
public:
    TransitionEngine(DriveFingerFn drive, BrakeFingerFn brake);

    // Call once the hand is physically known to be at OPEN (e.g. after the
    // boot-time forced-open sequence). Seeds every finger to FINGER_OPEN.
    void begin();

    // Diffs the current finger state against `target`'s fingerMask, plans
    // phases, and starts executing. Pre-empts any transition in progress by
    // braking fingers it still owns via cancelActiveLegs().
    void requestTransition(graspName target);

    // Standalone finger command(s): drives every finger with its bit set in
    // fingerMask to close[i] (true) or open (false), as one phase. Callers
    // moving more than one finger in the same dispatch pass must batch them
    // into a single call, since each call clears any not-yet-started phase.
    // timeoutMs[i] is the per-finger timeout ceiling (main.cpp picks it from
    // FINGER_FULL_CLOSE_MS/FINGER_FULL_OPEN_MS to match close[i]).
    void requestFingerMoves(uint8_t fingerMask, const bool close[MOTOR_COUNT], const uint16_t timeoutMs[MOTOR_COUNT]);

    // For actuation that takes over motor control from outside this
    // engine's request*() entry points (currently only WAVE). Cancels
    // whatever's active and clears the plan so the engine won't resume
    // driving these fingers while the external owner has them. Call before
    // taking over; call requestTransition()/requestFingerMoves() again to
    // hand control back.
    void yieldControl();

    // Advance active legs: brake+settle any finger whose timeout has
    // elapsed. Call every loop() tick with the current millis().
    void tick(uint32_t nowMs);

    // Safety hook: call when the stall watchdog confirms a motor this
    // engine is driving has physically stopped. Ends a closing leg
    // immediately via completeLeg(); a no-op for an opening leg (ripple
    // sensing is unreliable under opening's lighter, often spring-assisted
    // load, so opening always runs to FINGER_FULL_OPEN_MS instead). Also a
    // no-op if fingerIdx isn't currently owned by the engine.
    void notifyStall(uint8_t fingerIdx);

    // For actuation that bypasses this engine's phase machinery entirely
    // (currently only WAVE) but still moves a tracked finger. Callers
    // report the result, never claiming a settled arrival for a leg they
    // didn't run to completion.
    void reportFingerState(uint8_t fingerIdx, FingerState state);

    bool isTransitioning() const { return _phaseIndex < _phaseCount; }
    graspName transitionTarget() const { return _target; }

    // True (and *out set) iff every finger's state exactly matches some
    // GRAPH_NODES[] entry's fingerMask and nothing is transitioning. False
    // mid-transition, and false after a standalone requestFingerMoves()
    // unless the result happens to match a full node's pattern.
    bool currentNode(graspName* out) const;

private:
    struct Phase {
        uint8_t  fingerMask = 0;
        uint32_t timeoutMs[MOTOR_COUNT] = {0, 0, 0, 0, 0};   // ceiling, compared with >=, never subtracted
        bool     closing[MOTOR_COUNT]   = {false, false, false, false, false};
    };

    DriveFingerFn _drive;
    BrakeFingerFn _brake;

    FingerState _fingerState[MOTOR_COUNT] = {FINGER_OPEN, FINGER_OPEN, FINGER_OPEN, FINGER_OPEN, FINGER_OPEN};

    // Which gesture's requestTransition() call last closed this finger;
    // NO_GRAPH_TARGET if closed via requestFingerMoves()/WAVE instead.
    // Meaningless for any finger not currently FINGER_CLOSED.
    int8_t _closedVia[MOTOR_COUNT] = {NO_GRAPH_TARGET, NO_GRAPH_TARGET, NO_GRAPH_TARGET, NO_GRAPH_TARGET, NO_GRAPH_TARGET};

    Phase     _phases[ENGINE_MAX_PHASES];
    uint8_t   _phaseCount  = 0;
    uint8_t   _phaseIndex  = 0;
    bool      _phaseActive = false;
    uint8_t   _activeFingerMask = 0;
    uint32_t  _legStartMs[MOTOR_COUNT] = {0, 0, 0, 0, 0};
    graspName _target = OPEN;

    // True iff the active plan represents a real graph-node target
    // (requestTransition()), as opposed to a standalone finger move or a
    // hand-off via yieldControl().
    bool _activePlanIsGraphNode = false;

    void cancelActiveLegs();          // brake + leave state transient, nothing credited
    void startPhase(uint32_t nowMs);
    void completeLeg(uint8_t fingerIdx);   // the single settle point; called from notifyStall() and tick() only

    // The one place _fingerState[]/_closedVia[] are written together
    // (besides begin()'s boot reset), so the tag can never drift out of
    // sync with the state it describes.
    void setFingerState(uint8_t fingerIdx, FingerState newState);
};
