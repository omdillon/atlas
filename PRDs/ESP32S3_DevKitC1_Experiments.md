# ESP32-S3 DevKitC-1 Experiment Plan
**Project:** EPSRC Prosthetic Hand Development (ENG:04)
**Purpose:** Familiarisation with the ESP32-S3 DevKitC-1 and embedded C++ patterns before integrating with the prosthetic hand firmware
**Board:** ESP32-S3-DevKitC-1 (N8R8 variant — 8 MB flash, 8 MB PSRAM)
**Toolchain:** PlatformIO (VS Code), Arduino framework, NimBLE-Arduino
**Status:** Pre-development

---

## How to use this document

Two self-contained experiments, in order:

1. **EXP-01 — GPIO:** LED + button with software debounce
2. **EXP-02 — BLE:** Minimal NimBLE GATT server with a readable characteristic

Each experiment has a circuit description, annotated code, embedded C++ concepts to note, pass criteria, and common failure modes. Complete EXP-01 first — it confirms your toolchain and Serial output work before you add BLE complexity.

---

## Project scaffold (shared across all experiments)

Every experiment lives in its own PlatformIO project folder. The structure and base files below are identical for both — only `src/main.cpp` changes.

### Directory structure

```
esp32s3-experiments/
├── exp-01-gpio/
│   ├── platformio.ini
│   ├── include/
│   │   └── config.h
│   └── src/
│       └── main.cpp
├── exp-02-ble/
│   ├── platformio.ini
│   ├── include/
│   │   └── config.h
│   └── src/
│       └── main.cpp
```

### `platformio.ini` (same for both experiments)

```ini
[env:esp32-s3-devkitc-1]
platform   = espressif32
board      = esp32-s3-devkitc-1
framework  = arduino

; Native USB serial (S3 uses USB-OTG, not a UART bridge)
monitor_speed   = 115200
monitor_port    = /dev/ttyACM0   ; Windows: COM3 or similar — check Device Manager
build_flags     = -DARDUINO_USB_MODE=1
                  -DARDUINO_USB_CDC_ON_BOOT=1

; Required for NimBLE (EXP-02 only — harmless to include in EXP-01)
lib_deps = h2zero/NimBLE-Arduino @ ^1.4.2
```

> **S3 USB note:** The S3-DevKitC-1 has two USB ports — the one labelled **USB** (left port when the board faces you) is the native USB-OTG port used for flashing and Serial. The one labelled **UART** uses the onboard CH340 bridge. Use the **USB** port and the flags above. If `monitor` shows nothing, try the UART port as a fallback and remove the two `build_flags`.

### `include/config.h` (same for both experiments)

Centralise every pin and constant here. When you move to the custom hand PCB, this is the only file that changes.

```cpp
#pragma once

// ── Board identity ────────────────────────────────────────────────
#define BOARD_NAME "ESP32-S3-DevKitC-1"

// ── GPIO pins ─────────────────────────────────────────────────────
// Onboard RGB LED on the DevKitC-1 is WS2812 on GPIO 48.
// We use an external LED on a safe GPIO instead.
#define LED_PIN     4    // External LED (any 3.3V-tolerant GPIO)
#define BUTTON_PIN  0    // Boot button (already on the board, active LOW)

// ── I²C pins (not used in EXP-01/02 but defined for later) ───────
#define SDA_PIN     2
#define SCL_PIN     1
#define I2C_FREQ    100000

// ── Serial ────────────────────────────────────────────────────────
#define BAUD_RATE   115200

// ── BLE (EXP-02) ─────────────────────────────────────────────────
#define BLE_DEVICE_NAME   "ProHand-Test"

// Custom 128-bit UUIDs — generated once, reused across all firmware
// Generate your own at https://www.uuidgenerator.net/
#define SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
#define CHAR_COUNTER_UUID "12345678-1234-1234-1234-123456789abd"
```

> **C++ pattern — `#pragma once`:** This is the modern alternative to the traditional `#ifndef CONFIG_H / #define CONFIG_H / #endif` include guard. It tells the compiler to include this file only once per translation unit even if it's `#include`d from multiple places. Prefer it on all your header files.

---

## EXP-01 — GPIO: LED and button with software debounce

### Goal

Toggle an LED on each confirmed button press using software debounce. Confirms:
- PlatformIO flashes the S3 correctly
- `pinMode` / `digitalWrite` / `digitalRead` work as expected
- `Serial.println()` appears in the monitor
- You can handle the button's mechanical bounce without hardware filtering

### What you need

- ESP32-S3-DevKitC-1
- 1× LED + 220 Ω resistor (or use the BOOT button and an LED on GPIO 4)
- 1× breadboard and jumper wires

### Circuit

```
3.3V ──┬── [10kΩ pull-up] ── GPIO 0 (BUTTON_PIN)
       │                         │
      GND                   [Button] ── GND

GPIO 4 (LED_PIN) ── [220Ω] ── LED (+) ── GND
```

> The BOOT button (GPIO 0) already has a pull-up resistor on the DevKitC-1, so no external resistor is needed if you use GPIO 0. The pin reads HIGH when the button is not pressed, LOW when pressed (active LOW).

### `src/main.cpp`

```cpp
#include <Arduino.h>
#include "config.h"

// ── Debounce configuration ────────────────────────────────────────
// How long (ms) the button signal must be stable before we accept it.
// 50 ms is a safe starting value for a tactile switch.
static const uint32_t DEBOUNCE_MS = 50;

// ── State variables ───────────────────────────────────────────────
// 'static' at file scope means these are private to this translation
// unit — not visible to any other .cpp file. Good practice when a
// variable is only ever used in one file.
static bool     led_state       = false;   // current LED on/off state
static bool     last_raw        = HIGH;    // last raw button reading
static bool     stable_state    = HIGH;    // last debounced state
static uint32_t last_change_ms  = 0;       // timestamp of last raw change

void setup() {
    Serial.begin(BAUD_RATE);

    // pinMode configures a GPIO as input or output.
    // INPUT_PULLUP enables the internal ~45kΩ pull-up resistor,
    // so the pin reads HIGH at rest and LOW when pulled to GND.
    pinMode(LED_PIN,    OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(LED_PIN, LOW);   // start with LED off

    Serial.println("EXP-01 ready — press the button");
}

void loop() {
    // ── Read the raw pin value ────────────────────────────────────
    // digitalRead returns HIGH (1) or LOW (0).
    bool current_raw = digitalRead(BUTTON_PIN);

    // ── Debounce logic ────────────────────────────────────────────
    // A mechanical button bounces — the signal oscillates between
    // HIGH and LOW several times in the first ~20ms after a press.
    // We only accept a new state if the raw signal has been stable
    // for at least DEBOUNCE_MS milliseconds.
    //
    // millis() returns the number of milliseconds since boot as
    // a uint32_t. It wraps at ~49 days — not a concern here.
    // Using subtraction (millis() - last_change_ms) handles wrap
    // correctly, unlike a direct comparison.

    if (current_raw != last_raw) {
        // The signal just changed — reset the stability timer.
        last_change_ms = millis();
        last_raw = current_raw;
    }

    if ((millis() - last_change_ms) >= DEBOUNCE_MS) {
        // The signal has been stable long enough — it's real.
        if (current_raw != stable_state) {
            stable_state = current_raw;

            // We only act on the falling edge (HIGH → LOW),
            // which is the moment the button is pressed down.
            // The rising edge (LOW → HIGH) is the release — ignore it.
            if (stable_state == LOW) {
                led_state = !led_state;                    // toggle
                digitalWrite(LED_PIN, led_state ? HIGH : LOW);
                Serial.print("Button pressed. LED is now: ");
                Serial.println(led_state ? "ON" : "OFF");
            }
        }
    }
}
```

### Embedded C++ concepts to note

**`uint32_t` instead of `int` for time:** The Arduino `millis()` function returns `unsigned long`, which is 32 bits on the ESP32. Using `uint32_t` makes the width explicit and portable. Always use `uint32_t` for timestamps in embedded code — a signed `int` would give wrong results when `millis()` wraps or when doing subtraction across large gaps.

**`static` local vs global:** Variables declared `static` at file scope (outside any function) are zero-initialised and live for the entire program lifetime, like globals, but they're invisible outside this file. Prefer this over true globals — it prevents accidental modification from other files as your codebase grows.

**Edge detection:** Reading a button in a loop gives you the signal *level*, not the *event*. Storing `stable_state` and comparing it to `current_raw` lets you detect the exact moment the state transitions (the edge). Acting on the falling edge (press) rather than the level means the LED toggles once per press, not once per loop iteration.

**Why not `delay()`:** `delay(50)` would achieve debounce but freezes the entire CPU for 50 ms. On a system where you'll later be running motor control on Core 0 and BLE on Core 1, any `delay()` call is a bug waiting to happen. The timestamp pattern above is non-blocking — the CPU keeps looping freely.

### Pass criteria

- [ ] LED toggles on each button press with no double-fires
- [ ] Serial monitor shows a line for each press, not multiple lines per press
- [ ] LED state is correctly reported as ON/OFF alternating

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Serial monitor blank | Wrong USB port — try the UART port, or check `build_flags` |
| LED doesn't light | LED polarity reversed, or wrong GPIO — check with a multimeter |
| Multiple toggles per press | `DEBOUNCE_MS` too low — increase to 80–100 ms |
| Button seems to work randomly | Missing pull-up — ensure `INPUT_PULLUP` is set, or add external 10kΩ |
| Sketch won't flash | Hold BOOT button while pressing RESET to enter download mode manually |

---

## EXP-02 — BLE: Minimal NimBLE GATT server

### Goal

Advertise a BLE device named `ProHand-Test`, accept a connection, and expose a single NOTIFY characteristic that sends an incrementing counter to the connected client every second. Confirms:

- NimBLE library initialises correctly on the S3
- The device is discoverable in a BLE scanner app (use **nRF Connect** on your phone)
- A client can connect, subscribe to notifications, and receive data
- The GATT pattern (service → characteristic → notify) works end-to-end

This is the exact same pattern the PRD uses for the `TELEMETRY_CHAR_UUID` — just with a counter instead of EMG data.

### What you need

- ESP32-S3-DevKitC-1 (no extra hardware — BLE is onboard)
- A phone with **nRF Connect** installed (Android or iOS)

### Circuit

None. This experiment is software only.

### `src/main.cpp`

```cpp
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"

// ── GATT server object pointers ───────────────────────────────────
// NimBLE objects are heap-allocated and managed by the library.
// We keep raw pointers here — NimBLE owns the memory, we just
// hold references. Do not delete these.
static NimBLEServer*         ble_server   = nullptr;
static NimBLECharacteristic* ble_counter  = nullptr;

// ── State ─────────────────────────────────────────────────────────
static bool     device_connected     = false;
static bool     prev_connected       = false;
static uint32_t counter              = 0;
static uint32_t last_notify_ms       = 0;
static const uint32_t NOTIFY_INTERVAL_MS = 1000;  // send every 1 second

// ── Connection callbacks ──────────────────────────────────────────
// NimBLE calls these methods on connect/disconnect events.
// They run in the BLE task context (not loop()), so keep them short.
//
// C++ pattern — inheriting from a callback class:
// NimBLE uses the 'observer' pattern. You subclass NimBLEServerCallbacks
// and override the methods you care about. NimBLE calls your overrides
// at the right moment. This is a classic use of virtual dispatch.
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server) override {
        device_connected = true;
        Serial.println("[BLE] Client connected");

        // Stop advertising once connected — we only support one client.
        // To allow re-connection after disconnect, restart advertising
        // in onDisconnect() (see below).
        NimBLEDevice::stopAdvertising();
    }

    void onDisconnect(NimBLEServer* server) override {
        device_connected = false;
        Serial.println("[BLE] Client disconnected — restarting advertising");

        // Restart advertising so the client can reconnect without
        // power-cycling the ESP32. This is the re-advertising behaviour
        // mandated by NFR-REL from the PRD.
        NimBLEDevice::startAdvertising();
    }
};

// ── BLE initialisation ────────────────────────────────────────────
// Called once from setup(). Broken into its own function to keep
// setup() readable — a good habit as firmware grows.
static void init_ble() {
    // Initialise the NimBLE stack with the device name.
    // This must be called before any other NimBLE function.
    NimBLEDevice::init(BLE_DEVICE_NAME);

    // Optional but recommended: request a larger ATT MTU.
    // The server can only suggest — the client negotiates the final value.
    // 517 = 512 bytes payload + 3 bytes ATT header + 2 bytes L2CAP.
    NimBLEDevice::setMTU(517);

    // Create the GATT server and register our connection callbacks.
    ble_server = NimBLEDevice::createServer();
    ble_server->setCallbacks(new ServerCallbacks());

    // Create a GATT service with our custom UUID.
    NimBLEService* service = ble_server->createService(SERVICE_UUID);

    // Create the counter characteristic inside the service.
    // NOTIFY means the server pushes data to the client — the client
    // does not need to poll. This is how all telemetry works in the PRD.
    //
    // We also add READ so the client can fetch the current value
    // immediately on connection without waiting for the next notify.
    ble_counter = service->createCharacteristic(
        CHAR_COUNTER_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    // Start the service. Must be called before advertising.
    service->start();

    // Configure and start advertising.
    // setName() sets the name shown in BLE scanner apps.
    // addServiceUUID() tells scanners which services this device offers —
    // the web app uses this UUID to filter the device list.
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->setName(BLE_DEVICE_NAME);
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("[BLE] Advertising as: " + String(BLE_DEVICE_NAME));
}

void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);   // short delay — gives the serial monitor time to connect
    Serial.println("EXP-02 starting");

    init_ble();
}

void loop() {
    // ── Send counter notification ─────────────────────────────────
    // Only send if a client is connected AND has subscribed to
    // notifications. getSubscribedCount() returns the number of
    // clients that have enabled notifications on this characteristic.
    if (device_connected && ble_counter->getSubscribedCount() > 0) {
        if ((millis() - last_notify_ms) >= NOTIFY_INTERVAL_MS) {
            last_notify_ms = millis();
            counter++;

            // Pack the counter into a 4-byte little-endian buffer.
            // This is the same packing pattern used for all values
            // in the PRD telemetry frame.
            //
            // C++ pattern — reinterpret_cast with a byte buffer:
            // We cast the address of 'counter' (a uint32_t*) to a
            // uint8_t* so we can read it as raw bytes. The ESP32 is
            // little-endian, so counter = 0x00000001 is stored as
            // bytes [0x01, 0x00, 0x00, 0x00] in memory.
            // This is the correct byte order for the PRD's packet schema.
            uint8_t buf[4];
            memcpy(buf, &counter, sizeof(counter));

            // setValue() sets the characteristic's value buffer.
            // notify() pushes it to all subscribed clients immediately.
            ble_counter->setValue(buf, sizeof(buf));
            ble_counter->notify();

            Serial.print("[BLE] Notified counter: ");
            Serial.println(counter);
        }
    }

    // ── Detect reconnection ───────────────────────────────────────
    // prev_connected lets us detect the transition from connected
    // to disconnected (falling edge on device_connected).
    // The onDisconnect callback already restarts advertising,
    // but this block lets you add any app-level cleanup here.
    if (!device_connected && prev_connected) {
        Serial.println("[BLE] Cleaning up after disconnect");
        counter = 0;   // reset counter for the next session
    }
    prev_connected = device_connected;
}
```

### Testing with nRF Connect

1. Flash the sketch and open the Serial monitor — you should see `[BLE] Advertising as: ProHand-Test`
2. Open nRF Connect on your phone → **Scanner** tab → pull to refresh
3. `ProHand-Test` should appear in the list. Tap **Connect**
4. Serial monitor should show `[BLE] Client connected`
5. In nRF Connect, expand the **Unknown Service** (your `SERVICE_UUID`)
6. Find the characteristic (your `CHAR_COUNTER_UUID`) and tap the **⬇ subscribe** button (the three-downward-arrows icon)
7. The value field should update every second — tap the value to see it decoded as a `uint32` little-endian integer
8. Disconnect in nRF Connect — Serial monitor should show `[BLE] Client disconnected — restarting advertising` and the device should reappear in the scanner

### Embedded C++ concepts to note

**`nullptr` instead of `NULL`:** In modern C++ (C++11 and later), `nullptr` is the correct null pointer literal. `NULL` is a macro that expands to `0`, which is an integer — it can cause ambiguous overload resolution in some cases. The Arduino framework on ESP32 uses C++11, so always use `nullptr`.

**`override` keyword:** When overriding a virtual method from a base class, always add `override`. This tells the compiler to verify that the method signature exactly matches a virtual method in the base class. Without it, a typo in the method name (e.g. `onConect`) would silently compile as a new method rather than an override — your callbacks would never fire and you'd have no idea why.

**`static` functions:** `init_ble()` is declared `static` — it's a file-scoped function, not a method on any class, and it's not callable from outside this file. For small embedded projects, organising code this way (file-scoped static functions rather than classes) is idiomatic and perfectly appropriate.

**`memcpy` for packing values:** The pattern `memcpy(buf, &counter, sizeof(counter))` is the standard embedded way to copy a multi-byte value into a byte buffer without relying on pointer casting. It is defined behaviour in C++, unlike direct pointer casts which can violate strict aliasing rules. Use this pattern in the telemetry frame builder.

**Callback object lifetime:** `new ServerCallbacks()` allocates the callback object on the heap and passes ownership to NimBLE. This is one of the few legitimate uses of raw `new` in modern C++ — the library expects a raw pointer and manages the lifetime. Don't try to wrap it in a `unique_ptr` here.

### Pass criteria

- [ ] `ProHand-Test` appears in nRF Connect scanner within 5 seconds of boot
- [ ] Serial shows `[BLE] Client connected` immediately on phone connection
- [ ] Characteristic value increments every second in nRF Connect
- [ ] After disconnecting, device reappears in the scanner without rebooting
- [ ] Serial shows counter reset to 0 after disconnect

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Device doesn't appear in nRF Connect | BLE not initialised — check Serial for errors; ensure NimBLE lib is in `lib_deps` |
| Connects but no notifications | Client didn't subscribe — tap the subscribe button (⬇) in nRF Connect |
| Value shown as hex, not a number | nRF Connect shows raw bytes by default — tap the value and select `uint32, little-endian` |
| `NimBLEDevice::init` crashes | Stack overflow — NimBLE needs ~8 KB stack; check that no other large objects are stack-allocated in `setup()` |
| Doesn't reconnect after disconnect | Check `onDisconnect` is firing — add `Serial.println` to confirm. If not firing, the callback isn't registered |

---

## After completing both experiments

You will have confirmed:

- PlatformIO correctly targets the S3 and flashes via native USB
- GPIO input, output, and debounce work as expected
- NimBLE initialises, advertises, connects, and notifies
- The re-advertising pattern (from the PRD) works in practice
- The `memcpy` + little-endian packing pattern produces correct output in nRF Connect

**The natural next step is EXP-03: I²C + DRV8214 bring-up** (Stage 2 of the test plan). Before that, run `i2c_scanner` as a quick sanity check that the bus is live at `SDA_PIN 2 / SCL_PIN 1` — it takes 10 minutes and saves a lot of head-scratching if the DRV8214 doesn't respond on first contact.

### Bridging to the PRD codebase

When you start the Phase 1A firmware (the BLE skeleton), EXP-02 gives you the exact pattern to expand:

| EXP-02 concept | PRD equivalent |
|---|---|
| `CHAR_COUNTER_UUID` with NOTIFY | `TELEMETRY_CHAR_UUID` — same property, 40-byte frame instead of 4-byte counter |
| `ServerCallbacks::onDisconnect` restarting advertising | NFR-REL-02 — re-advertising within 1 s of disconnection |
| `memcpy(buf, &counter, sizeof(counter))` | Telemetry frame builder — same pattern for each `uint32` ripple count field |
| `NimBLEDevice::setMTU(517)` | FR-CON-03 — MTU negotiation at connection time |
| `getSubscribedCount() > 0` guard | Prevents notify calls before the client subscribes — avoids a crash in NimBLE |

The only additions for Phase 1A are: a second characteristic (COMMAND, WRITE), a third (RESPONSE, NOTIFY), a fourth (OTA_DATA, WRITE WITHOUT RESPONSE), and wiring the notify call to a FreeRTOS queue instead of a bare counter.
