# ESP32-C3 + Arduino Stepper Test Bench

A configurable one-to-four-axis, open-loop stepper controller with a local web
dashboard. The ESP32 hosts the UI and sends high-level commands over UART. An
Arduino Uno generates the STEP/DIR signals for its attached CNC Shield V3.

## ESP32 to Arduino wiring

| ESP32-C3 | Arduino/CNC shield | Connection |
| --- | --- | --- |
| GPIO 4 (TX) | RX / D0 | Direct |
| GPIO 5 (RX) | TX / D1 | Through a 1 kΩ / 2 kΩ divider |
| GND | GND | Direct |

For the divider, connect Arduino TX through 1 kΩ to ESP GPIO 5, then connect
2 kΩ from GPIO 5 to ground. If both boards have their own USB power, do not tie
their 5 V rails together. To use the ESP's USB supply for both logic boards,
connect ESP 5V to Arduino 5V and disconnect that link before plugging in the
Uno's USB. Disconnect the ESP serial wires while uploading firmware to the Uno.

## Wiring

The Arduino firmware uses the CNC shield's fixed active-low pin map:

| Signal | GPIO |
| --- | ---: |
| Shared ENABLE | Arduino D8 |
| Axis 1 STEP / DIR | Arduino D2 / D5 |
| Axis 2 STEP / DIR | Arduino D3 / D6 |
| Axis 3 STEP / DIR | Arduino D4 / D7 |
| Axis 4 (A) STEP / DIR | Arduino D12 / D13 |

For an independent A axis, install the shield's A.STEP and A.DIR routing jumpers
in the D12/D13 position. Selecting an X, Y, or Z clone position makes the A
socket mirror that axis instead of responding to the A-axis controls.

Use A4988-compatible STEP/DIR inputs, a common ground, and an external pull-up
on ENABLE so the drivers remain disabled while the ESP32 resets. Microstep mode
is selected on the driver; enter the resulting effective pulses/revolution in
the axis profile.

## Configure and upload

1. Set `WIFI_SSID` and `WIFI_PASSWORD` as quoted PlatformIO build flags in the
   ignored `platformio.local.ini` file.
2. Review the other build flags in `platformio.ini`; they define the initial
   axis count, pin map, motion limits, and profile defaults.
3. Upload the Arduino bridge with
   `pio run -e arduino-uno-bridge -t upload`.
4. Upload the ESP firmware with
   `pio run -e esp32-c3-devkitc-02 -t upload`.
5. Open `http://stepper-test.local/` or the IP printed on the serial monitor at
   115200 baud.

The device only joins the configured network. It does not create a fallback
access point.

## Dashboard controls

Each axis has persistent reverse/forward continuous-run controls. While an axis
is running, changing its speed or acceleration slider updates the target in
place instead of restarting the move. The displayed RPM is calculated from the
axis profile's effective pulses per revolution, including microstepping.

The default 12 V starting profile is 400 pulses/s with 800 pulses/s²
acceleration and a 1200 pulses/s profile ceiling. The firmware-wide ceiling is
5000 pulses/s. Motor inductance, driver current, microstepping, and mechanical
load determine the usable limit, so the per-axis ceiling remains editable.

Use **Focus** on an axis card to expand it and collapse the other axes into a
live side rail. Relative moves remain available as a numeric field without a
position slider.

## Safety and behavior

- This is open-loop control. Position reflects emitted pulses, not measured
  shaft position. Zero establishes a relative software reference; Disable All
  invalidates it.
- Every connected browser can issue commands immediately; there is no control
  lease or heartbeat. Finite moves and cycle tests continue if a page closes.
- Emergency Stop and Disable All cancel every job and release shared ENABLE.
- The HTTP/WebSocket interface is unauthenticated. Keep it on a trusted LAN and
  do not expose it to the internet.
- A physical motor-power cutoff is still required. The web button is not a
  hardware emergency stop.

## Tests

Run the hardware-independent motion tests with:

```sh
pio test -e native
```

Build the firmware with:

```sh
pio run -e arduino-uno-bridge
pio run -e esp32-c3-devkitc-02
```
