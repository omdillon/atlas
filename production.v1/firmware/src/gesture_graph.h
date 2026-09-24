#pragma once
#include <Arduino.h>

// Gesture graph node data structure.
//
// fingerMask is the target open/closed state per finger; the hand's current
// gesture is identified by matching the live per-finger state against a
// node's fingerMask (see transition_engine.h's FingerState), not by
// numeric position.
//
// closeDurationMs is a timeout ceiling, not a target depth: closing is
// normally stall-terminated (ends when the stall watchdog confirms the
// finger has physically stopped), and this is the backstop if that watchdog
// doesn't fire. Values are ported from the original firmware's hand-tuned
// timings.
//
// Opening never stall-terminates (see notifyStall() in transition_engine.h
// for why) and always runs to FINGER_FULL_OPEN_MS instead.
//
// numPhases/phaseMask is arrival sequencing: which fingers a node closes
// together, and in what order (e.g. POWER closes the four fingers, then
// sweeps the thumb to lock the grip). Property of the target node, not of
// a specific transition.
//
// INVARIANT: every fingerMask below must be unique in this table.
// currentNode() (transition_engine.cpp) matches purely by mask, so a
// collision would make it ambiguous. validateGraphNodes() checks this at
// boot; run it after adding a node.
static const uint8_t MOTOR_COUNT = 5;
static const uint8_t MAX_PHASES  = 2;   // no gesture needs more than 2 phases (fingers, then thumb)

enum graspName {
    PACK = -1,
    OPEN = 0,
    POWER = 1,
    TRIPOD = 2,
    PINCH = 3,
    POINTER = 4,
    ROCK = 5,
    WAVE = 6,       // scripted animation, not a graph node; see main.cpp

    // CV-teleoperation expansion set, appended after WAVE so no existing
    // enum value (and its telemetry current_node_id/target_node_id
    // encoding, which is graspName+1) shifts.
    BIRD = 7,
    TWO = 8,
    JAMBO = 9,
    THREE = 10,
    THUMB = 11,
};

struct GestureNode {
    graspName gesture;
    uint8_t   fingerMask;                     // bit i = finger i CLOSED (target state) in this node (0=thumb..4=pinky)
    uint16_t  closeDurationMs[MOTOR_COUNT];   // timeout ceiling for a closing (or, reversed, opening) leg; only meaningful where fingerMask bit i is set
    uint8_t   numPhases;
    uint8_t   phaseMask[MAX_PHASES];          // arrival sequencing, in order; each entry is a finger-bit subset of fingerMask
};

// gesture     fingerMask  closeDurationMs[thumb,index,middle,ring,pinky]   numPhases  phaseMask
static const GestureNode GRAPH_NODES[] = {
    { OPEN,    0x00, {0,    0,   0,    0,    0   }, 0, {0x00, 0x00} },
    { POWER,   0x1F, {1000, 1500,1500, 1500, 1500}, 2, {0x1E, 0x01} },  // fingers, then thumb
    { TRIPOD,  0x07, {810,  810, 810,  0,    0   }, 1, {0x07, 0x00} },  // thumb+index+middle together
    { PINCH,   0x03, {810,  810, 0,    0,    0   }, 1, {0x03, 0x00} },  // thumb+index together
    { POINTER, 0x1D, {2000, 0,   2000, 2000, 2000}, 1, {0x1D, 0x00} },  // everything but index
    { ROCK,    0x0D, {2000, 0,   1500, 1500, 0   }, 2, {0x0C, 0x01} },  // middle+ring, then thumb
    { PACK,    0x01, {2000, 0,   0,    0,    0   }, 1, {0x01, 0x00} },  // thumb only

    // CV-teleoperation expansion set: new poses without a precedent in the
    // original firmware. Phase sequencing for BIRD/TWO/THREE (which close
    // the thumb alongside other fingers, an ambiguous case not settled by
    // the 7 nodes above) defaults to the conservative fingers-first-then-
    // thumb order. Not yet bench-validated the way the 7 gestures above are.
    { BIRD,    0x1B, {2000, 1500, 0,    2000, 2000}, 2, {0x1A, 0x01} },  // index+ring+pinky, then thumb
    { TWO,     0x19, {2000, 0,    0,    2000, 2000}, 2, {0x18, 0x01} },  // ring+pinky, then thumb
    { JAMBO,   0x0E, {0,    1500, 2000, 2000, 0   }, 1, {0x0E, 0x00} },  // index+middle+ring together
    { THREE,   0x11, {2000, 0,    0,    0,    2000}, 2, {0x10, 0x01} },  // pinky, then thumb
    { THUMB,   0x1E, {0,    1500, 2000, 2000, 2000}, 1, {0x1E, 0x00} },  // index+middle+ring+pinky together
};
static const uint8_t GRAPH_NODE_COUNT = sizeof(GRAPH_NODES) / sizeof(GRAPH_NODES[0]);

static inline int graphIndexOf(graspName g)
{
    for (uint8_t i = 0; i < GRAPH_NODE_COUNT; i++)
        if (GRAPH_NODES[i].gesture == g) return i;
    return -1;
}

// Checks the fingerMask-uniqueness invariant. Call once from setup(); warns
// (does not halt) on a collision.
static inline void validateGraphNodes()
{
    for (uint8_t a = 0; a < GRAPH_NODE_COUNT; a++) {
        for (uint8_t b = a + 1; b < GRAPH_NODE_COUNT; b++) {
            if (GRAPH_NODES[a].fingerMask == GRAPH_NODES[b].fingerMask) {
                Serial.printf("[WARN] GRAPH_NODES fingerMask collision: gesture %d and %d both use mask 0x%02X, currentNode() will be ambiguous between them\n",
                              (int)GRAPH_NODES[a].gesture, (int)GRAPH_NODES[b].gesture, GRAPH_NODES[a].fingerMask);
            }
        }
    }
}

// Order must match the finger index convention used throughout (0=thumb..4=pinky).
static const char* const FINGER_NAMES[MOTOR_COUNT] = { "thumb", "index", "middle", "ring", "pinky" };

// Fully-closed duration for a standalone single-finger command, not owned by
// any gesture node. Defaults to the max closeDurationMs seen for that finger
// across GRAPH_NODES; bench-tunable independently if needed.
static const uint16_t FINGER_FULL_CLOSE_MS[MOTOR_COUNT] = { 2000, 1500, 2000, 2000, 2000 };

// Timeout ceiling for every opening leg (graph transitions and standalone
// "<finger>_open" commands alike). Opening is never stall-terminated, so
// this is the only thing that ends it.
static const uint16_t FINGER_FULL_OPEN_MS[MOTOR_COUNT] = { 2000, 2000, 2000, 2000, 2000 };
