# Firmware setup

## Requirements

- [VS Code](https://code.visualstudio.com/) with the **PlatformIO IDE**
  extension installed.
- Prosthetic Hand

Open the `firmware/` folder in VS Code - PlatformIO detects the project
automatically via `platformio.ini`. No `.env` file or secrets are needed;
this firmware has no WiFi/cloud credentials. Everything is configured in
`firmware/platformio.ini` under `[env:hand_teleoperation]` (board, build
flags, library deps) - there's only one environment, so no need to select
one in the PlatformIO toolbar.

If the board doesn't auto-select in the PlatformIO status bar (bottom of
VS Code), click it and pick the right serial port - look for the one
labelled "USB", not "UART" (see **Build & flash** below).

> **Board variant note:** this config builds for the ESP32-S3 

## Wiring

I2C bus to the 5 DRV8214 drivers:

| Signal | ESP32-S3 GPIO |
|---|---|
| SDA | GPIO2 |
| SCL | GPIO1 |

> **Motor 5 (pinky) is wired physically reversed** relative to the other four
> drivers. Firmware compensates for this in software (`main.cpp`'s open/close
> handlers call `turnForward`/`turnReverse` on motor 5 the opposite way round
> from motors 1-4) — this is intentional and pre-existing, not a bug. If you
> rewire or replace this driver, preserve the reversal or motor 5 will open
> and close backwards.

## Build & flash

From `firmware/`:

```
pio run                 # compile
pio run -t upload       # flash
pio run -t monitor      # serial monitor, 115200 baud
```

Use the port labelled "USB", not "UART": the S3 uses native USB-OTG, not a
UART bridge. If the serial monitor shows nothing, try the UART port instead
and drop the two `ARDUINO_USB_*` build flags in `platformio.ini`.

## After flashing

The board advertises over BLE as `ProHand-EXP13` (see
[`protocol.md`](protocol.md) for the full command/telemetry reference). It's
ready to pair with `software/telemetry.py`; see
[`setup_software.md`](setup_software.md).
