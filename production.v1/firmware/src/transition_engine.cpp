#include "transition_engine.h"

TransitionEngine::TransitionEngine(DriveFingerFn drive, BrakeFingerFn brake)
    : _drive(drive), _brake(brake)
{
}

void TransitionEngine::begin()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        _fingerState[i]   = FINGER_OPEN;
        _closedVia[i]     = NO_GRAPH_TARGET;
    }
    _phaseCount  = 0;
    _phaseIndex  = 0;
    _phaseActive = false;
    _activeFingerMask = 0;
    _target = OPEN;
    _activePlanIsGraphNode = false;
}

// Records which target drove this finger to FINGER_CLOSED (only meaningful
// when _activePlanIsGraphNode is true); clears the tag for any other state.
void TransitionEngine::setFingerState(uint8_t fingerIdx, FingerState newState)
{
    _fingerState[fingerIdx] = newState;
    _closedVia[fingerIdx] = (newState == FINGER_CLOSED && _activePlanIsGraphNode)
                                ? (int8_t)_target : NO_GRAPH_TARGET;
}

// The single settle point for a leg, reached only from notifyStall() (a
// confirmed stall) and tick() (a timeout). Both guard on the finger still
// being in _activeFingerMask before calling this, so a stall and a timeout
// landing close together can never double-complete the same leg.
void TransitionEngine::completeLeg(uint8_t fingerIdx)
{
    const Phase& ph = _phases[_phaseIndex];
    _brake(fingerIdx);
    setFingerState(fingerIdx, ph.closing[fingerIdx] ? FINGER_CLOSED : FINGER_OPEN);
    _activeFingerMask &= ~(1u << fingerIdx);

    if (_activeFingerMask == 0) {
        _phaseActive = false;
        _phaseIndex++;
    }
}

// Brakes every active finger but leaves _fingerState[] as-is (still
// OPENING/CLOSING from startPhase()) so the next planning pass always
// redrives it from scratch rather than trusting a mid-travel stop point.
void TransitionEngine::cancelActiveLegs()
{
    if (!_phaseActive) return;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(_activeFingerMask & (1u << i))) continue;
        _brake(i);
    }
    _phaseActive = false;
    _activeFingerMask = 0;
}

void TransitionEngine::requestTransition(graspName target)
{
    int idx = graphIndexOf(target);
    if (idx < 0) return;   // WAVE and unknown gestures aren't graph nodes; main.cpp handles those separately
    const GestureNode& node = GRAPH_NODES[idx];

    cancelActiveLegs();
    _phaseCount = 0;
    _phaseIndex = 0;
    _target = target;
    _activePlanIsGraphNode = true;

    // A finger already settled FINGER_CLOSED is only safe to leave in place
    // if it was closed while driving toward this same target last time
    // (_closedVia[i] == target) - a mismatch means either a different
    // closing order or a different stall depth, so its rest position can't
    // be trusted for this target and it must be force-reopened and
    // reclosed fresh below.
    uint8_t forceReopenMask = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(node.fingerMask & (1u << i))) continue;
        if (_fingerState[i] != FINGER_CLOSED) continue;
        if (_closedVia[i] != (int8_t)target) forceReopenMask |= (1u << i);
    }
    if (forceReopenMask) {
        Serial.printf("[ENGINE] forced re-open before reclose: mask=0x%02X, target=%d\n",
                      forceReopenMask, (int)target);
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (!(forceReopenMask & (1u << i))) continue;
            Serial.printf("[ENGINE]   finger %d (%s): was closed-via %d, now targeting %d\n",
                          (int)i, FINGER_NAMES[i], (int)_closedVia[i], (int)target);
        }
    }

    // Phase 0: every finger that needs to open, together - either because
    // `node` wants it open or because its rest position was force-reopened
    // above. Timeout uses the flat FINGER_FULL_OPEN_MS ceiling, since
    // opening never stall-terminates and this is the only thing ending it.
    Phase openPhase;
    bool  haveOpenPhase = false;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        bool wantClosed = (node.fingerMask & (1u << i)) != 0;
        bool needsOpen  = wantClosed ? (forceReopenMask & (1u << i)) != 0
                                      : (_fingerState[i] != FINGER_OPEN);
        if (!needsOpen) continue;
        openPhase.fingerMask |= (1u << i);
        openPhase.timeoutMs[i] = FINGER_FULL_OPEN_MS[i];
        openPhase.closing[i]   = false;
        haveOpenPhase = true;
    }
    if (haveOpenPhase) _phases[_phaseCount++] = openPhase;

    // Phases 1..N: the target node's declared closing order, filtered to
    // fingers that still need to close (not already settled-closed, and not
    // force-reopened above).
    for (uint8_t p = 0; p < node.numPhases; p++) {
        Phase ph;
        bool  any = false;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (!(node.phaseMask[p] & (1u << i))) continue;
            bool alreadyThere = (_fingerState[i] == FINGER_CLOSED) && !(forceReopenMask & (1u << i));
            if (alreadyThere) continue;
            ph.fingerMask |= (1u << i);
            ph.timeoutMs[i] = node.closeDurationMs[i];
            ph.closing[i]   = true;
            any = true;
        }
        if (any) _phases[_phaseCount++] = ph;
    }
}

void TransitionEngine::requestFingerMoves(uint8_t fingerMask, const bool close[MOTOR_COUNT], const uint16_t timeoutMs[MOTOR_COUNT])
{
    cancelActiveLegs();
    _phaseCount = 0;
    _phaseIndex = 0;
    _activePlanIsGraphNode = false;
    // _target is left as whatever it last was; a standalone finger move
    // isn't a node, so currentNode() will report off-node until a full
    // gesture lands.

    Phase ph;
    bool  any = false;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(fingerMask & (1u << i))) continue;
        FingerState wantSettled = close[i] ? FINGER_CLOSED : FINGER_OPEN;
        if (_fingerState[i] == wantSettled) continue;   // already there, settled
        ph.fingerMask |= (1u << i);
        ph.timeoutMs[i] = timeoutMs[i];
        ph.closing[i]   = close[i];
        any = true;
    }
    if (any) _phases[_phaseCount++] = ph;
}

void TransitionEngine::yieldControl()
{
    cancelActiveLegs();
    _phaseCount = 0;
    _phaseIndex = 0;
    _activePlanIsGraphNode = false;
}

void TransitionEngine::notifyStall(uint8_t fingerIdx)
{
    if (!_phaseActive) return;
    if (!(_activeFingerMask & (1u << fingerIdx))) return;   // not ours to stop

    // Opening legs are timeout-only; motor_state[] still reflects the stall
    // for telemetry (main.cpp), this only gates whether the engine acts on it.
    if (!_phases[_phaseIndex].closing[fingerIdx]) return;

    completeLeg(fingerIdx);
}

void TransitionEngine::reportFingerState(uint8_t fingerIdx, FingerState state)
{
    setFingerState(fingerIdx, state);
}

void TransitionEngine::startPhase(uint32_t nowMs)
{
    const Phase& ph = _phases[_phaseIndex];
    _activeFingerMask = ph.fingerMask;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(ph.fingerMask & (1u << i))) continue;
        _legStartMs[i] = nowMs;
        setFingerState(i, ph.closing[i] ? FINGER_CLOSING : FINGER_OPENING);
        _drive(i, ph.closing[i]);
    }
    _phaseActive = true;
}

void TransitionEngine::tick(uint32_t nowMs)
{
    if (!_phaseActive) {
        if (_phaseIndex < _phaseCount) startPhase(nowMs);
        return;
    }

    const Phase& ph = _phases[_phaseIndex];
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(_activeFingerMask & (1u << i))) continue;
        if ((int32_t)(nowMs - _legStartMs[i]) >= (int32_t)ph.timeoutMs[i]) {
            completeLeg(i);
        }
    }
}

bool TransitionEngine::currentNode(graspName* out) const
{
    if (isTransitioning()) return false;   // never claim a node mid-leg

    // Also refuse if any finger is transient for a reason outside this
    // engine's own phases (currently only a WAVE-interrupted finger via
    // reportFingerState()) - otherwise it could read identically to a
    // settled finger and falsely match a node.
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (_fingerState[i] == FINGER_OPENING || _fingerState[i] == FINGER_CLOSING) return false;
    }

    for (uint8_t k = 0; k < GRAPH_NODE_COUNT; k++) {
        bool match = true;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            bool wantClosed = (GRAPH_NODES[k].fingerMask & (1u << i)) != 0;
            bool isClosed   = (_fingerState[i] == FINGER_CLOSED);
            if (wantClosed != isClosed) { match = false; break; }
        }
        if (match) { *out = GRAPH_NODES[k].gesture; return true; }
    }
    return false;
}
