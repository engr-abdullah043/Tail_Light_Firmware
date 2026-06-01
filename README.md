# Tail Light Firmware

Arduino firmware for an ESP32-S3 based automotive tail light controller. The firmware drives four SK6812 RGBW LED PCB strips, handles brake/turn/hazard signal inputs, provides BLE-controlled show animations, stores selected settings in ESP32 NVS, and prioritizes vehicle lighting behavior over decorative BLE effects.


## 1. Project Overview

This firmware controls a four-board tail light assembly using an ESP32-S3 controller.

The system supports:

- Four SK6812 RGBW LED boards
- 78 LEDs per board
- Brake input
- Left turn input
- Right turn input
- Hazard detection from simultaneous left/right turn activity
- Brake + turn combined behavior
- Brake + hazard combined behavior
- BLE control for show animations
- BLE color control for animation output
- Persistent running-light mode storage
- Persistent BLE animation selection
- Persistent BLE animation color
- Startup light check
- Status LED and heartbeat LED
- Watchdog protection
- LED output recovery refresh

The firmware is written as a single Arduino sketch:

```text
Tail_Light_Firmware.ino
```

---

## 2. Target Hardware

### MCU

- Target controller: ESP32-S3
- Expected module family: ESP32-S3 N16R8 or equivalent
- Recommended flash size: 16 MB
- Framework: Arduino IDE / ESP32 Arduino core

### LED Type

- LED family: SK6812 RGBW or compatible RGBW addressable LEDs
- Driver library: Adafruit NeoPixel
- LED protocol: 800 kHz NeoPixel-style single-wire data
- Pixel format used in firmware: `NEO_GRBW + NEO_KHZ800`

### LED Board Count

The firmware controls four LED PCB strips:

| Board | Side | GPIO |
|---|---:|---:|
| L1 | Left | GPIO 4 |
| L2 | Left | GPIO 16 |
| R1 | Right | GPIO 12 |
| R2 | Right | GPIO 14 |

Each board contains:

```text
78 LEDs per strip
```

---

## 3. Repository Structure

```text
Tail_Light_Firmware/
|-- Tail_Light_Firmware.ino
|-- partitions.csv
|-- README.md
|-- .gitignore
`-- Docs/
```

### File Descriptions

| File / Folder | Purpose |
|---|---|
| `Tail_Light_Firmware.ino` | Main Arduino firmware source file |
| `partitions.csv` | Custom ESP32-S3 16 MB partition layout |
| `README.md` | Project documentation |
| `Docs/` | Hardware references, design notes, diagrams, datasheets, and support material |

---

## 4. Pin Mapping

### Vehicle Signal Inputs

| Signal | ESP32-S3 GPIO | Active Level | Notes |
|---|---:|---:|---|
| Brake | GPIO 1 | HIGH | External circuit must provide stable idle-low / active-high signal |
| Left Turn | GPIO 2 | HIGH | Pulse-qualified by firmware |
| Right Turn | GPIO 38 | HIGH | Pulse-qualified by firmware |

The firmware configures these pins as high-impedance `INPUT` pins. The external signal-conditioning circuit must provide the correct pull-down / idle-low behavior. The ESP32 internal pull-down is not enabled in firmware.

### LED Outputs

| LED Output | ESP32-S3 GPIO |
|---|---:|
| L1 LED Data | GPIO 4 |
| L2 LED Data | GPIO 16 |
| R1 LED Data | GPIO 12 |
| R2 LED Data | GPIO 14 |

### Status LEDs

| Status LED | ESP32-S3 GPIO | Behavior |
|---|---:|---|
| Ready LED | GPIO 17 | Set HIGH after startup initialization begins |
| Heartbeat LED | GPIO 8 | Toggles every 500 ms while firmware is running |

---

## 5. LED Layout

Each PCB has 78 addressable LEDs. The firmware divides each board into six physical rows and three larger line sections.

### Large LED Sections

| Section | LED Index Range | Human LED Number Range |
|---|---:|---:|
| Bottom segment | 0-23 | LEDs 1-24 |
| Middle segment | 24-53 | LEDs 25-54 |
| Top segment | 54-77 | LEDs 55-78 |

### Physical Row Layout

The firmware uses the following six-row geometry:

| Row | Start Index | Length | Direction |
|---|---:|---:|---|
| Row 0 | 0 | 11 | Left to right |
| Row 1 | 11 | 13 | Right to left |
| Row 2 | 24 | 15 | Left to right |
| Row 3 | 39 | 15 | Right to left |
| Row 4 | 54 | 13 | Left to right |
| Row 5 | 67 | 11 | Right to left |

This row layout is used for turn arrows, brake sweep, BLE animations, and geometry-based animation effects.

### Right-Side BLE Animation Mapping

For BLE show animations, the firmware mirrors animation pixel mapping on the right-side strips (`R1` and `R2`). This allows the right boards to visually match the left boards during decorative animations. Vehicle signal rendering still uses explicit left/right side logic for directional behavior.

---

## 6. Runtime Priority

Vehicle lighting has priority over BLE animation output.

The runtime priority order is:

1. Hazard + brake
2. Hazard
3. Turn + brake
4. Turn
5. Brake
6. BLE animation
7. Running light

This means BLE animations are automatically interrupted when brake, turn, or hazard activity is detected.

---

## 7. Lighting Behavior

### 7.1 Startup Light Check

On boot, the firmware performs a startup color check before entering normal running mode.

Startup color sequence:

1. Red
2. Green
3. Blue
4. White
5. Amber

Each color pulses through multiple brightness levels, then the firmware switches into the selected running-light mode.

### 7.2 Running Light Mode

When brake, turn, hazard, and BLE animation modes are inactive, the firmware shows the selected red running-light mode.

Available running-light modes:

| Mode | BLE Command | Description |
|---|---|---|
| All lines | `RUN,ALL` | Bottom, middle, and top sections are on |
| Outer lines | `RUN,OUTER` | Bottom and top sections only |
| Middle line | `RUN,MIDDLE` | Middle section only |

Running-light mode is stored in ESP32 NVS and restored after reset or power cycle.

Default running-light mode:

```text
RUN,ALL
```

Running-light color:

```text
R = 40
G = 0
B = 0
W = 0
```

### 7.3 Brake Mode

When brake input is active:

- All four LED boards switch to brake red.
- The brake output uses all LED sections.
- Brake output overrides the selected running-light mode.
- A center-out brake sweep is used when brake first becomes active.

Brake color:

```text
R = 255
G = 0
B = 0
W = 0
```

Brake sweep timing:

```text
400 ms
```

### 7.4 Turn Mode

Turn mode is pulse-qualified. The firmware does not treat every raw transition as a valid turn signal immediately.

Left turn:

- Animates `L1` and `L2`
- Right side remains in base running/brake state

Right turn:

- Animates `R1` and `R2`
- Left side remains in base running/brake state

The active side renders an amber arrow-shaped sweep using the six-row PCB layout. The middle rows lead the sweep, while the upper and lower rows lag to create a chevron-style turn movement.

Turn color:

```text
R = 255
G = 55
B = 0
W = 4
```

### 7.5 Turn + Brake Mode

When brake and a valid turn pulse are active at the same time:

- The full assembly is first rendered in brake red.
- The active turn side is cleared.
- The active turn side renders the amber turn sweep.
- The non-turning side remains brake red.

This keeps brake visibility while preserving directional indication.

### 7.6 Hazard Mode

Hazard mode is detected when both left and right turn activity are valid at the same time.

When hazard is active without brake:

- All four boards flash amber together.

### 7.7 Hazard + Brake Mode

When hazard and brake are active together:

- All boards are first rendered in brake red.
- During the blink ON phase, only the middle section flashes amber.
- This creates a red / amber / red segmented output.

---

## 8. Signal Qualification and Timing

### Input Debounce

```text
50 ms
```

### Turn Pulse Qualification

A left or right turn pulse is accepted only if the high-time is inside this window:

```text
Minimum high pulse: 80 ms
Maximum high pulse: 750 ms
```

After a valid pulse is detected, turn mode remains active for:

```text
1300 ms
```

This hold window prevents the output from dropping back to running mode during normal automotive flasher gaps.

### Blink Timing

```text
Blink ON time: 500 ms
Blink OFF time: 500 ms
```

### Animation Timing

```text
BLE animation frame interval: 20 ms
Turn sweep frame interval: 20 ms
Brake sweep frame interval: 20 ms
```

### Watchdog

```text
Watchdog timeout: 8 seconds
```

The firmware feeds the watchdog during normal operation and during blocking startup delays.

### LED Recovery

The firmware periodically refreshes/reinitializes LED output to recover from transient output issues.

```text
LED recovery refresh interval: 750 ms
LED recovery reinitialization interval: 5000 ms
```

---

## 9. BLE Configuration

BLE is initialized after startup LED check and running-frame setup.

### BLE Device

```text
Device Name: Tail Light
```

### BLE Service

```text
Service UUID: 0000FF00-0000-1000-8000-00805F9B34FB
```

### BLE Characteristic

```text
Characteristic UUID: 0000FF01-0000-1000-8000-00805F9B34FB
```

Characteristic properties:

```text
WRITE
WRITE_NR
```

The BLE characteristic accepts write commands only.

---

## 10. BLE Commands

The firmware trims whitespace, converts input to uppercase, and removes spaces before processing commands.

### 10.1 Start Solid Blue Mode

```text
ON
```

This enables BLE mode and turns all LEDs solid blue.

Solid blue color:

```text
R = 0
G = 0
B = 255
W = 0
```

### 10.2 Stop BLE Mode

```text
OFF
```

This disables BLE animation/solid-blue mode and returns control to vehicle/running-light behavior.

### 10.3 Select Running-Light Mode

```text
RUN,ALL
RUN,OUTER
RUN,MIDDLE
```

These commands are persistent and are restored after reset/power cycle.

### 10.4 Start BLE Animation

```text
ANIM,<id>
```

Valid animation IDs:

```text
1 to 20
```

Examples:

```text
ANIM,1
ANIM,10
ANIM,20
```


### 10.5 Set Named Animation Color

```text
COLOR,RED
COLOR,GREEN
COLOR,BLUE
COLOR,WHITE
COLOR,WARMWHITE
COLOR,WARM_WHITE
COLOR,AMBER
COLOR,ORANGE
COLOR,YELLOW
COLOR,PURPLE
COLOR,PINK
COLOR,MAGENTA
COLOR,CYAN
COLOR,ICEBLUE
COLOR,ICE_BLUE
COLOR,OFF
COLOR,BLACK
```

Animation color is persistent and is restored after reset/power cycle.

### 10.6 Set Custom RGBW Animation Color

Supported formats:

```text
COLOR,CUSTOM,<R>,<G>,<B>,<W>
COLOR,RGBW,<R>,<G>,<B>,<W>
RGBW,<R>,<G>,<B>,<W>
```

Each value must be from `0` to `255`.

Examples:

```text
COLOR,CUSTOM,255,0,0,0
COLOR,RGBW,0,0,255,0
RGBW,255,55,0,4
```

Default BLE animation color:

```text
R = 0
G = 0
B = 255
W = 0
```

---

## 11. BLE Animation List

The firmware implements 20 BLE animation patterns.

| ID | Firmware Name | Render Function |
|---:|---|---|
| 1 | Circular flow | `patternBlueCircularFlow()` |
| 2 | Row sweep | `patternGreenSweep()` |
| 3 | Tetris stack | `patternBluePerimeterFlow()` |
| 4 | Center-out alert | `patternCenterOutAlert()` |
| 5 | Stack fill | `patternStackFillAll()` |
| 6 | Raindrop fall | `patternRainDrop()` |
| 7 | Night rider scanner | `patternNightRiderScanner()` |
| 8 | Pulse wave | `patternPulseWave()` |
| 9 | Aurora ribbons | `patternAuroraRibbons()` |
| 10 | Return-flow stack | `patternReturnFlowStack()` |
| 11 | Top flash cascade | `patternTopFlashCascade()` |
| 12 | Sequential intensity wash | `patternSequentialIntensityWash()` |
| 13 | Single-row return stack | `patternSingleRowReturnStack()` |
| 14 | Three-zone flicker | `patternThreeZoneFlicker()` |
| 15 | Staged smooth flicker | `patternStagedSmoothFlicker()` |
| 16 | Mirror bridge return | `patternMirrorBridgeReturn()` |
| 17 | Randomized show mix | `patternRandomizedShowMix()` |
| 18 | Right-wipe terminal blink | `patternRightWipeTerminalBlink()` |
| 19 | Center fold out-in | `patternCenterFoldOutIn()` |
| 20 | Wave heartbeat | `patternWaveHeartbeat()` |

---

## 12. Persistent Settings

The firmware uses ESP32 Preferences / NVS storage under this namespace:

```text
tail_light
```

Stored values:

| Setting | NVS Key | Description |
|---|---|---|
| Running-light mode | `run_mode` | Stores selected running mode |
| Selected BLE animation | `anim_id` | Stores last selected valid BLE animation |
| Animation red value | `anim_r` | Stores custom/named animation red channel |
| Animation green value | `anim_g` | Stores custom/named animation green channel |
| Animation blue value | `anim_b` | Stores custom/named animation blue channel |
| Animation white value | `anim_w` | Stores custom/named animation white channel |

Defaults if settings are unavailable or not yet stored:

```text
Running mode: all lines
Animation ID: 1
Animation color: RGBW 0,0,255,0
```

---

## 13. Build Setup

### Recommended Arduino IDE Setup

Use Arduino IDE with the ESP32 board package installed.

Recommended settings:

| Setting | Value |
|---|---|
| Board family | ESP32-S3 |
| Flash size | 16 MB |
| Partition scheme | Custom `partitions.csv` |
| Upload speed | Any stable ESP32-S3 upload speed supported by the board |
| Serial monitor baud rate | 115200 |

### Required External Library

Install this Arduino library:

```text
Adafruit NeoPixel
```

### Libraries Provided by ESP32 Arduino Core

These headers are supplied by the ESP32 Arduino board package / ESP-IDF integration:

```cpp
#include <BLEDevice.h>
#include <Preferences.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
```

---

## 14. Custom Partition Layout

The project includes a custom 16 MB ESP32 partition table:

```text
# Name,   Type, SubType, Offset,   Size,      Flags
nvs,      data, nvs,     0x9000,   0x5000,
otadata,  data, ota,     0xE000,   0x2000,
app0,     app,  ota_0,   0x10000,  0x640000,
app1,     app,  ota_1,   0x650000, 0x640000,
spiffs,   data, spiffs,  0xC90000, 0x370000,
```

This layout provides:

- NVS storage
- OTA metadata
- Two OTA application slots
- SPIFFS region

Use this layout when building for the intended ESP32-S3 16 MB hardware.

---

## 15. Flashing Procedure

1. Install Arduino IDE.
2. Install the ESP32 board package in Arduino IDE.
3. Install the `Adafruit NeoPixel` library.
4. Clone or download this repository.
5. Open `Tail_Light_Firmware.ino` in Arduino IDE.
6. Select the correct ESP32-S3 board target.
7. Select 16 MB flash size if available.
8. Use the project `partitions.csv` custom partition layout if required by the selected board setup.
9. Connect the PCB to the computer through USB/UART.
10. Select the correct COM port.
11. Compile the firmware.
12. Upload the firmware.
13. Open Serial Monitor at `115200 baud`.
14. Confirm the startup log and LED startup check.
15. Test brake, left turn, right turn, hazard, and BLE commands.

---

## 16. Expected Serial Logs

At startup, the firmware prints:

```text
BLE initialised and advertising
Tail light Arduino firmware started
L1 on GPIO 4
L2 on GPIO 16
R1 on GPIO 12
R2 on GPIO 14
```

When BLE commands are received, examples include:

```text
BLE cmd: ON solid blue
BLE cmd: OFF
BLE cmd: RUN,all lines
Selected BLE animation 10: return-flow stack
BLE cmd: return-flow stack
Animation color RGBW: 255,55,0,4
BLE cmd: COLOR
BLE animation interrupted by signal
```

---

## 17. Basic Test Checklist

### Power-On Test

- Ready LED should turn on.
- Heartbeat LED should toggle every 500 ms.
- LED boards should run the startup color check.
- After startup, boards should enter running-light mode.

### Running-Light Test

Send:

```text
RUN,ALL
RUN,OUTER
RUN,MIDDLE
```

Confirm that the selected running-light section changes and remains stored after power cycle.

### Brake Test

Apply brake input HIGH.

Expected result:

- All four boards turn brake red.
- Brake output overrides running-light mode and BLE animations.

### Turn Test

Apply valid left or right turn pulses.

Expected result:

- Left input animates L1/L2.
- Right input animates R1/R2.
- Active side shows amber sweep.
- Non-active side stays in base state.

### Hazard Test

Apply valid left and right turn activity together.

Expected result:

- All boards flash amber together.

### Hazard + Brake Test

Apply brake and both turn activities together.

Expected result:

- Boards stay brake red.
- Middle section flashes amber during blink ON phase.

### BLE Animation Test

Send:

```text
ON
OFF
ANIM,1
ANIM,10
ANIM,20
```

Expected result:

- `ON` shows solid blue.
- `OFF` returns to running-light behavior.
- `ANIM,1` to `ANIM,20` start valid BLE animations.

### BLE Color Test

Send:

```text
COLOR,RED
COLOR,BLUE
COLOR,AMBER
RGBW,255,55,0,4
```

Expected result:

- BLE animation color changes.
- Color is stored and restored after reset/power cycle.


---

## 18. Development Notes

- Main firmware is contained in `Tail_Light_Firmware.ino`.
- Settings are stored in ESP32 NVS using the `Preferences` library.
- BLE runs through the ESP32 Arduino BLE stack.
- LED output is driven through `Adafruit_NeoPixel`.
- Safety signal rendering overrides BLE animation output.
- The firmware includes watchdog feeding during delays and normal loop execution.
- LED output recovery periodically refreshes and reinitializes LED strips.

---

## 19. Current Firmware Summary

The current firmware is a complete ESP32-S3 tail light controller with:

- 4 LED outputs
- 312 total RGBW LEDs
- Brake, turn, hazard, and running-light logic
- BLE show animation mode
- 20 BLE animation patterns
- Persistent running mode
- Persistent animation selection
- Persistent animation color
- Startup light check
- Status/heartbeat LEDs
- Watchdog and LED recovery handling
