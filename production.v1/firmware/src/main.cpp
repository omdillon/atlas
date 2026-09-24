// Motor driver / BLE / telemetry plumbing below (DRV8214 setup,
// execMotorCommand(), per-motor state, stall detection, BLE connection
// lifecycle) is ported from this project's earlier proven firmware, largely
// unchanged. Gesture dispatch is new: it goes through TransitionEngine
// (transition_engine.*), which drives fingers by commanded time rather than
// reading motor position back.
//
// Software stall detection is a safety watchdog only: for a closing leg it
// can force TransitionEngine::notifyStall() to end a jammed finger's phase
// early, but never decides "reached target" on its own. An opening leg
// ignores stall entirely (ripple sensing is unreliable under opening's
// lighter, often spring-assisted load) and always runs to gesture_graph.h's
// FINGER_FULL_OPEN_MS ceiling instead.
//
// WAVE is a scripted animation, not a graph node: it runs via a small
// standalone step runner (startWave/tickWave below) that drives fingers
// directly, bypassing the engine's phase machinery.

#include <Arduino.h>
#include "DRV8214.h"
#include "gesture_graph.h"
#include "transition_engine.h"

#include <NimBLEDevice.h>
#include <algorithm>

#define I2C_FREQUENCY 100000
#define SCL_PIN 1
#define SDA_PIN 2

#define IPROPI_RESISTOR 4700
#define NUM_RIPPLES 7
#define RED_RATIO 64
#define MAX_RPM 800
#define INTERNAL_MOTOR_R 2.5957
#define NFAULT5 36
#define MOTOR_CURRENT 1.5F
#define MOTOR_VOLTAGE 0.0F
#define DRIVER_ID_1 1
#define DRIVER_ID_2 2
#define DRIVER_ID_3 3
#define DRIVER_ID_4 4
#define DRIVER_ID_5 5

uint16_t MOTOR_SPEED = 0;

hw_timer_t *timer10ms = nullptr;
volatile bool tick10ms = false;
void IRAM_ATTR onTimer10ms() { tick10ms = true; }

// Vocabulary execMotorCommand() speaks; callers build one of these directly
// per call, no queue.
enum motorDir { FORWARD, REVERSE, BRAKE, STATIC };
struct motorCommand { uint motor; motorDir direction; int delay; };

void execMotorCommand(motorCommand &cmd);   // defined near the bottom

DRV8214_Config cfg;
void setConfig()
{
    cfg.control_mode = PWM;
    cfg.I2CControlled = true;
    cfg.regulation_mode = CURRENT_FIXED;
    cfg.voltage_range = false;
    cfg.ovp_enabled = false;
    cfg.current_reg_mode = 3;
    cfg.stall_enabled = true;
    cfg.stall_behavior = true;
    cfg.bridge_behavior_thr_reached = false;
    cfg.soft_start_stop_enabled = false;
    cfg.inrush_duration = 22;
    cfg.kmc = 5;
    cfg.kmc_scale = 4;
    cfg.verbose = false;
}

DRV8214 motorDriver5(DRV8214_I2C_ADDR_ZZ, DRIVER_ID_5, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver4(DRV8214_I2C_ADDR_Z0, DRIVER_ID_4, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver3(DRV8214_I2C_ADDR_01, DRIVER_ID_3, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver2(DRV8214_I2C_ADDR_0Z, DRIVER_ID_2, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);
DRV8214 motorDriver1(DRV8214_I2C_ADDR_00, DRIVER_ID_1, IPROPI_RESISTOR, NUM_RIPPLES, INTERNAL_MOTOR_R, RED_RATIO, MAX_RPM);

static DRV8214* const motorDrivers[MOTOR_COUNT] = {
    &motorDriver1, &motorDriver2, &motorDriver3, &motorDriver4, &motorDriver5,
};

// Per-motor state, reported over telemetry.
enum MotorState : uint8_t { MOTOR_IDLE = 0, MOTOR_MOVING = 1, MOTOR_HOLDING = 2, MOTOR_STALLED = 3 };
static MotorState motor_state[MOTOR_COUNT] = { MOTOR_IDLE, MOTOR_IDLE, MOTOR_IDLE, MOTOR_IDLE, MOTOR_IDLE };

// Ripple/position bookkeeping. Not used by TransitionEngine or gesture
// dispatch, but execMotorCommand()'s BRAKE case reads motor_position[] back
// to decide MOTOR_HOLDING vs MOTOR_IDLE. No longer packed into the
// telemetry frame (the GUI's Pos% column was dropped as redundant).
static const uint32_t FULL_CLOSURE_RIPPLES[MOTOR_COUNT] = {10000, 30000, 30000, 30000, 30000};
static int32_t  motor_position[MOTOR_COUNT]   = {0, 0, 0, 0, 0};
static motorDir motor_active_dir[MOTOR_COUNT] = {BRAKE, BRAKE, BRAKE, BRAKE, BRAKE};

static void flushRippleTravel(uint8_t i)
{
    uint32_t travelled = motorDrivers[i]->getRippleCount();
    if      (motor_active_dir[i] == FORWARD) motor_position[i] += (int32_t)travelled;
    else if (motor_active_dir[i] == REVERSE) motor_position[i] -= (int32_t)travelled;

    if (motor_position[i] < 0) motor_position[i] = 0;
    if (motor_position[i] > (int32_t)FULL_CLOSURE_RIPPLES[i]) motor_position[i] = (int32_t)FULL_CLOSURE_RIPPLES[i];

    motorDrivers[i]->resetRippleCounter();
}

// Software stall detection: a safety watchdog only (see header comment).
static uint32_t stall_last_ripple[MOTOR_COUNT]    = {0, 0, 0, 0, 0};
static uint16_t stall_stagnant_ticks[MOTOR_COUNT] = {0, 0, 0, 0, 0};
static uint32_t stall_leg_start_ms[MOTOR_COUNT]   = {0, 0, 0, 0, 0};

static const uint32_t STALL_SAMPLE_INTERVAL_MS = 50;
static const uint32_t STALL_GRACE_MS           = 120;
static const uint32_t STALL_RIPPLE_DELTA       = 2;
static const uint16_t STALL_CONFIRM_TICKS      = 3;

static void resetStallTracking(uint8_t i)
{
    stall_last_ripple[i]    = 0;
    stall_stagnant_ticks[i] = 0;
    stall_leg_start_ms[i]   = millis();
}

// Actuation callbacks wired into the engine: every finger move it decides on
// ultimately calls execMotorCommand().
static void driveFingerReal(uint8_t idx, bool closing)
{
    motorCommand cmd{ (uint)(idx + 1), closing ? FORWARD : REVERSE, 0 };
    execMotorCommand(cmd);
}

static void brakeFingerReal(uint8_t idx)
{
    motorCommand cmd{ (uint)(idx + 1), BRAKE, 0 };
    execMotorCommand(cmd);
}

static TransitionEngine engine(driveFingerReal, brakeFingerReal);

static void updateStallDetection()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (motor_state[i] != MOTOR_MOVING && motor_state[i] != MOTOR_STALLED) continue;
        if (millis() - stall_leg_start_ms[i] < STALL_GRACE_MS) continue;

        uint32_t r = motorDrivers[i]->getRippleCount();
        if (r - stall_last_ripple[i] >= STALL_RIPPLE_DELTA) {
            stall_last_ripple[i]    = r;
            stall_stagnant_ticks[i] = 0;
            if (motor_state[i] == MOTOR_STALLED) motor_state[i] = MOTOR_MOVING;
        } else if (++stall_stagnant_ticks[i] >= STALL_CONFIRM_TICKS) {
            bool alreadyStalled = (motor_state[i] == MOTOR_STALLED);
            motor_state[i] = MOTOR_STALLED;
            if (!alreadyStalled) engine.notifyStall(i);
        }
    }
}

// WAVE drives fingers directly, bypassing the engine's phase machinery, but
// reports the result of every leg back via reportFingerState(): a clean,
// uninterrupted step reports the settled state, an interrupted or stalled
// step reports the transient state (same rule as the engine's own
// completeLeg()). Without this, the engine's belief about WAVE's fingers
// goes stale the moment WAVE is interrupted by another command.
struct WaveStep { uint8_t mask; bool closing; uint16_t durationMs; };
static const WaveStep WAVE_STEPS[] = {
    {0x1E, true,  500}, {0x1E, false, 500},
    {0x1E, true,  500}, {0x1E, false, 500},
    {0x1E, true,  500}, {0x1E, false, 500},
};
static const uint8_t WAVE_STEP_COUNT = sizeof(WAVE_STEPS) / sizeof(WAVE_STEPS[0]);
static bool     waveActive     = false;
static uint8_t  waveStepIdx    = 0;
static uint32_t waveStepStartMs = 0;

static void stopWaveIfActive()
{
    if (!waveActive) return;
    const WaveStep& s = WAVE_STEPS[waveStepIdx];
    // Always interrupted mid-step here, so report the transient state, never settled.
    FingerState resultState = s.closing ? FINGER_CLOSING : FINGER_OPENING;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(s.mask & (1u << i))) continue;
        brakeFingerReal(i);
        engine.reportFingerState(i, resultState);
    }
    waveActive = false;
}

static void startWave()
{
    waveActive      = true;
    waveStepIdx     = 0;
    waveStepStartMs = millis();
    const WaveStep& s = WAVE_STEPS[0];
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) if (s.mask & (1u << i)) driveFingerReal(i, s.closing);
}

static void tickWave(uint32_t nowMs)
{
    if (!waveActive) return;
    const WaveStep& s = WAVE_STEPS[waveStepIdx];

    // Brake immediately if the stall watchdog already flagged one of this
    // step's fingers as jammed, rather than waiting out the rest of the
    // nominal duration. Closing steps only; an opening step always runs its
    // full nominal duration, same as the graph engine's own opening legs.
    bool stalled = false;
    if (s.closing) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if ((s.mask & (1u << i)) && motor_state[i] == MOTOR_STALLED) { stalled = true; break; }
        }
    }
    if (!stalled && (nowMs - waveStepStartMs < s.durationMs)) return;

    // Only a full, uninterrupted step reports a settled arrival; a
    // stall-triggered early exit reports the transient state instead.
    FingerState resultState = stalled
        ? (s.closing ? FINGER_CLOSING : FINGER_OPENING)
        : (s.closing ? FINGER_CLOSED  : FINGER_OPEN);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!(s.mask & (1u << i))) continue;
        brakeFingerReal(i);
        engine.reportFingerState(i, resultState);
    }
    waveStepIdx++;
    if (waveStepIdx >= WAVE_STEP_COUNT) { waveActive = false; return; }

    waveStepStartMs = nowMs;
    const WaveStep& next = WAVE_STEPS[waveStepIdx];
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) if (next.mask & (1u << i)) driveFingerReal(i, next.closing);
}

// BLE connection lifecycle + command dispatch, ported from the project's
// earlier firmware. Device name and UUIDs are unique to this firmware.
#define BLE_DEVICE_NAME     "ProHand-EXP13"
#define SERVICE_UUID        "5237876f-2b43-4a69-9cf3-f01a8f6f2404"
#define CHAR_TELEMETRY_UUID "12df09d3-200c-4abe-b423-cbfe105cb2b6"
#define CHAR_MOTOR_CMD_UUID "c6975e18-d186-4acb-bab3-6138e560b4de"

static volatile bool device_connected = false;
static NimBLECharacteristic* ble_telemetry = nullptr;
static uint16_t telemetry_seq = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    // Logs the connection interval actually negotiated (a peripheral can
    // only request a change via updateConnParams() below, not set it
    // directly). This interval is the real ceiling on notification rate:
    // shortening TELEMETRY_INTERVAL_MS below it buys nothing.
    void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
        device_connected = true;
        Serial.println("[BLE] Connection established");
        Serial.printf("[BLE] Negotiated connection interval: %u (%.2f ms)\n",
                       desc->conn_itvl, desc->conn_itvl * 1.25f);

        // Request a fixed 60ms interval (48 x 1.25ms) to match
        // TELEMETRY_INTERVAL_MS below. This is a request, not a guarantee -
        // the central has final say. Check the GUI's "Rate: N Hz" readout
        // to see what actually took effect.
        server->updateConnParams(desc->conn_handle, 48, 48, 0, 400);

        NimBLEDevice::stopAdvertising();
    }
    void onDisconnect(NimBLEServer* server) override {
        device_connected = false;
        Serial.println("[BLE] Client disconnected, restarting advertising");
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, ble_gap_conn_desc* desc) override {
        Serial.printf("[BLE] MTU negotiated: %u\n", mtu);
    }
};

static bool lookupGraspByName(const std::string& name, graspName* out)
{
    struct Entry { const char* text; graspName grasp; };
    static const Entry table[] = {
        {"open", OPEN}, {"power", POWER}, {"tripod", TRIPOD}, {"pinch", PINCH},
        {"pointer", POINTER}, {"rock", ROCK}, {"pack", PACK}, {"wave", WAVE},
        {"bird", BIRD}, {"two", TWO}, {"jambo", JAMBO}, {"three", THREE}, {"thumb", THUMB},
    };
    for (const Entry& e : table) if (name == e.text) { *out = e.grasp; return true; }
    return false;
}

static bool lookupFingerCommand(const std::string& cmd, uint8_t* out_motor, bool* out_close)
{
    size_t us = cmd.rfind('_');
    if (us == std::string::npos) return false;
    std::string finger = cmd.substr(0, us);
    std::string action  = cmd.substr(us + 1);

    bool close;
    if      (action == "close") close = true;
    else if (action == "open")  close = false;
    else return false;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (finger == FINGER_NAMES[i]) { *out_motor = i; *out_close = close; return true; }
    }
    return false;
}

// Deferred BLE requests: onWrite() runs on NimBLE's host task (core 0);
// loop() runs on core 1 and owns all I2C/motor access. onWrite() only
// raises a flag; loop() acts on its next tick.
static volatile bool      pendingClearFaultsRequest = false;
static volatile bool      pendingGraspRequest        = false;
static graspName          pendingGrasp = OPEN;
static volatile bool      pendingFingerReq[MOTOR_COUNT]   = {false, false, false, false, false};
static volatile bool      pendingFingerClose[MOTOR_COUNT] = {false, false, false, false, false};
static volatile bool      pendingOverrideRequest = false;
static graspName          pendingOverrideTarget  = OPEN;

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic) override {
        std::string value = characteristic->getValue();
        while (!value.empty() && std::isspace((unsigned char)value.back())) value.pop_back();
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        Serial.printf("[BLE] Command received: \"%s\"\n", value.c_str());

        graspName grasp;
        uint8_t   fmotor;
        bool      fclose;
        if (value == "clear_faults") {
            pendingClearFaultsRequest = true;
        } else if (value == "override_open") {
            pendingOverrideTarget = OPEN;
            pendingOverrideRequest = true;
        } else if (value == "override_pack") {
            pendingOverrideTarget = PACK;
            pendingOverrideRequest = true;
        } else if (lookupGraspByName(value, &grasp)) {
            pendingGrasp = grasp;
            pendingGraspRequest = true;
        } else if (lookupFingerCommand(value, &fmotor, &fclose)) {
            pendingFingerClose[fmotor] = fclose;
            pendingFingerReq[fmotor]   = true;
        } else {
            Serial.println("[BLE] Unrecognised command, ignored");
        }
    }
};

static void clearAllFaults()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) motorDrivers[i]->resetFaultFlags();
    Serial.println("[BLE] Clear faults: all sticky fault bits cleared");
}

// Telemetry frame: 55 bytes, wire format documented in docs/protocol.md.
// current_node_id/target_node_id encode (graspName + 1); 0xFF on
// current_node_id means off-node/mid-transition. target_node_id has no
// sentinel: it is always the last-requested node, so the GUI can
// reconstruct current/target/in-progress purely from telemetry, without
// remembering what it last clicked.
static const size_t TELEMETRY_FRAME_SIZE = 55;

static void build_telemetry_frame(uint8_t* buf)
{
    size_t offset = 0;
    memcpy(buf + offset, &telemetry_seq, sizeof(telemetry_seq)); offset += sizeof(telemetry_seq);

    uint8_t transitioning = (engine.isTransitioning() || waveActive) ? 1 : 0;
    memcpy(buf + offset, &transitioning, sizeof(transitioning)); offset += sizeof(transitioning);

    graspName node;
    uint8_t nodeId = 0xFF;
    if (!transitioning && engine.currentNode(&node)) nodeId = (uint8_t)((int8_t)node + 1);
    memcpy(buf + offset, &nodeId, sizeof(nodeId)); offset += sizeof(nodeId);

    uint8_t targetId = (uint8_t)((int8_t)engine.transitionTarget() + 1);
    memcpy(buf + offset, &targetId, sizeof(targetId)); offset += sizeof(targetId);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        DRV8214* driver = motorDrivers[i];
        // 4 I2C transactions instead of 6 (each field would otherwise be its
        // own transaction); not a single burst read because this chip has a
        // register auto-increment boundary that prevents it (bench-confirmed).
        DRV8214_StatusBurst status = driver->getStatusBurst();

        // The DRV8214's own hardware STALL bit is masked out (sticky and
        // unreliable); the software stall watchdog's motor_state[] is the
        // trusted signal reported here instead.
        uint8_t  fault_byte      = status.fault & ~FAULT_STALL;
        if (motor_state[i] == MOTOR_STALLED) fault_byte |= FAULT_STALL;
        uint8_t  reported_state  = (uint8_t)motor_state[i];
        float    voltage         = status.voltage;
        float    current         = status.current;

        memcpy(buf + offset, &reported_state, sizeof(reported_state)); offset += sizeof(reported_state);
        memcpy(buf + offset, &voltage,        sizeof(voltage));        offset += sizeof(voltage);
        memcpy(buf + offset, &current,        sizeof(current));        offset += sizeof(current);
        memcpy(buf + offset, &fault_byte,     sizeof(fault_byte));     offset += sizeof(fault_byte);
    }
}

static void init_ble()
{
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(517);

    NimBLEServer* ble_server = NimBLEDevice::createServer();
    ble_server->setCallbacks(new ServerCallbacks());

    NimBLEService* service = ble_server->createService(SERVICE_UUID);

    ble_telemetry = service->createCharacteristic(
        CHAR_TELEMETRY_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    NimBLECharacteristic* command_char = service->createCharacteristic(
        CHAR_MOTOR_CMD_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    command_char->setCallbacks(new CommandCallbacks());

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->setName(BLE_DEVICE_NAME);
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("[BLE] Advertising as: " + String(BLE_DEVICE_NAME));
}

// execMotorCommand(): the single choke point every motor action passes
// through. Banks the ending leg's ripples, then drives/brakes the physical
// motor and updates the bookkeeping arrays.
void execMotorCommand(motorCommand &cmd)
{
    if (cmd.direction == STATIC) return;
    if (cmd.motor < 1 || cmd.motor > MOTOR_COUNT) return;
    uint8_t idx = cmd.motor - 1;

    switch (cmd.direction) {
        case FORWARD:
            flushRippleTravel(idx);
            switch (cmd.motor) {
                case 1: motorDriver1.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 2: motorDriver2.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 3: motorDriver3.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 4: motorDriver4.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 5: motorDriver5.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;   // motor 5 wired reversed, pre-existing
            }
            motor_active_dir[idx] = FORWARD;
            motor_state[idx]      = MOTOR_MOVING;
            resetStallTracking(idx);
            break;
        case REVERSE:
            flushRippleTravel(idx);
            switch (cmd.motor) {
                case 1: motorDriver1.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 2: motorDriver2.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 3: motorDriver3.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 4: motorDriver4.turnReverse(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;
                case 5: motorDriver5.turnForward(MOTOR_SPEED, MOTOR_VOLTAGE, MOTOR_CURRENT); break;   // motor 5 wired reversed, pre-existing
            }
            motor_active_dir[idx] = REVERSE;
            motor_state[idx]      = MOTOR_MOVING;
            resetStallTracking(idx);
            break;
        case BRAKE:
            flushRippleTravel(idx);
            switch (cmd.motor) {
                case 1: motorDriver1.brakeMotor(); break;
                case 2: motorDriver2.brakeMotor(); break;
                case 3: motorDriver3.brakeMotor(); break;
                case 4: motorDriver4.brakeMotor(); break;
                case 5: motorDriver5.brakeMotor(); break;
            }
            motor_active_dir[idx] = BRAKE;
            motor_state[idx] = (motor_position[idx] > 0) ? MOTOR_HOLDING : MOTOR_IDLE;
            break;
        default:
            break;
    }
}

// Boot safety: don't assume the hand is already open at power-up.
// Unconditionally reverses every finger for a safe worst-case duration
// (2000ms, the longest closeDurationMs in GRAPH_NODES), bypassing the
// engine, which only knows deltas relative to an assumed depth. Once
// physically open, engine.begin() seeds a truthful zero.
static void bootForceOpen()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motorCommand cmd{ (uint)(i + 1), REVERSE, 0 };
        execMotorCommand(cmd);
    }
    delay(2000);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motorCommand cmd{ (uint)(i + 1), BRAKE, 0 };
        execMotorCommand(cmd);
    }
    engine.begin();
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
    delay(2000);
    setConfig();

    timer10ms = timerBegin(0, 80, true);
    timerAttachInterrupt(timer10ms, &onTimer10ms, true);
    timerAlarmWrite(timer10ms, 10000, true);
    timerAlarmEnable(timer10ms);

    Serial.println("STARTING UP");
    delay(1000);

    motorDriver5.init(cfg);
    motorDriver4.init(cfg);
    motorDriver3.init(cfg);
    motorDriver2.init(cfg);
    motorDriver1.init(cfg);

    motorDriver5.setInternalVoltageReference(3.3F);
    motorDriver4.setInternalVoltageReference(3.3F);
    motorDriver3.setInternalVoltageReference(3.3F);
    motorDriver2.setInternalVoltageReference(3.3F);
    motorDriver1.setInternalVoltageReference(3.3F);
    motorDriver5.getSenseResistor();
    delay(1000);

    motorDriver5.enableStallInterrupt();
    motorDriver4.enableStallInterrupt();
    motorDriver3.enableStallInterrupt();
    motorDriver2.enableStallInterrupt();
    motorDriver1.enableStallInterrupt();
    delay(2000);

    bootForceOpen();

    validateGraphNodes();   // warns, doesn't halt, on a fingerMask collision; see gesture_graph.h

    init_ble();
}

void loop()
{
    if (tick10ms) {
        tick10ms = false;

        // Priority order: override > clear_faults > grasp/finger. Override
        // also suppresses any clear_faults/grasp/finger pending in the same
        // tick.
        if (pendingOverrideRequest) {
            pendingOverrideRequest = false;
            stopWaveIfActive();
            pendingGraspRequest = false;
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) pendingFingerReq[i] = false;
            pendingClearFaultsRequest = false;
            engine.requestTransition(pendingOverrideTarget);
            Serial.printf("[OVR] Override -> %d\n", (int)pendingOverrideTarget);
        }
        if (pendingClearFaultsRequest) { pendingClearFaultsRequest = false; clearAllFaults(); }
        if (pendingGraspRequest) {
            pendingGraspRequest = false;
            stopWaveIfActive();
            // yieldControl() first so the engine relinquishes whatever phase
            // it owns (leaving those fingers correctly transient) instead of
            // later braking a finger WAVE has already taken over.
            if (pendingGrasp == WAVE) { engine.yieldControl(); startWave(); }
            else                      engine.requestTransition(pendingGrasp);
        }
        // Batch every pending finger request into one call: requestFingerMoves()
        // replaces the current plan each time it's called, so one call per
        // finger would let a later finger silently discard an earlier one
        // staged in the same tick.
        uint8_t fingerMoveMask = 0;
        bool    fingerClose[MOTOR_COUNT];
        uint16_t fingerTimeoutMs[MOTOR_COUNT];
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (!pendingFingerReq[i]) continue;
            pendingFingerReq[i] = false;
            fingerMoveMask |= (1u << i);
            fingerClose[i] = pendingFingerClose[i];
            // Direction-specific ceiling: reusing the close-side constant for
            // an opening leg would silently under-time it (opening is never
            // stall-terminated).
            fingerTimeoutMs[i] = fingerClose[i] ? FINGER_FULL_CLOSE_MS[i] : FINGER_FULL_OPEN_MS[i];
        }
        if (fingerMoveMask) {
            stopWaveIfActive();
            engine.requestFingerMoves(fingerMoveMask, fingerClose, fingerTimeoutMs);
        }

        engine.tick(millis());
        tickWave(millis());
    }

    static uint32_t lastStallMs = 0;
    if (millis() - lastStallMs >= STALL_SAMPLE_INTERVAL_MS) {
        lastStallMs = millis();
        updateStallDetection();
    }

    static uint32_t lastTelemetryMs = 0;
    // Matches the 60ms connection-interval request in ServerCallbacks::
    // onConnect(); one frame per connection interval is the fastest this can
    // usefully go, so 60ms is both the request and the send cadence.
    const uint32_t TELEMETRY_INTERVAL_MS = 60;
    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = millis();
        if (device_connected && ble_telemetry->getSubscribedCount() > 0) {
            telemetry_seq++;
            uint8_t buf[TELEMETRY_FRAME_SIZE];
            build_telemetry_frame(buf);
            ble_telemetry->setValue(buf, sizeof(buf));
            ble_telemetry->notify();
        }
    }
}
