#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <Preferences.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
//BEFORE_UPDATE
const char *BLE_DEVICE_NAME = "Tail Light";
const char *BLE_SERVICE_UUID = "0000FF00-0000-1000-8000-00805F9B34FB";
const char *BLE_CHARACTERISTIC_UUID = "0000FF01-0000-1000-8000-00805F9B34FB";

const uint8_t BLE_ANIMATION_MIN_ID = 1;
const uint8_t BLE_ANIMATION_MAX_ID = 20;
const uint8_t BLE_ANIMATION_DEFAULT_ID = 1;

const char *PREF_NAMESPACE = "tail_light";
const char *PREF_RUNNING_MODE_KEY = "run_mode";
const char *PREF_ANIMATION_PATTERN_KEY = "anim_id";
const char *PREF_ANIMATION_RED_KEY = "anim_r";
const char *PREF_ANIMATION_GREEN_KEY = "anim_g";
const char *PREF_ANIMATION_BLUE_KEY = "anim_b";
const char *PREF_ANIMATION_WHITE_KEY = "anim_w";

const uint8_t BTN_BRAKE_GPIO = 1;
const uint8_t BTN_TURN_LEFT_GPIO = 2;
const uint8_t BTN_TURN_RIGHT_GPIO = 38;
const uint8_t BTN_ACTIVE_LEVEL = HIGH;

const uint8_t LED_STRIP_L1_GPIO = 4;
const uint8_t LED_STRIP_L2_GPIO = 16;
const uint8_t LED_STRIP_R1_GPIO = 12;
const uint8_t LED_STRIP_R2_GPIO = 14;

const uint16_t LEDS_PER_STRIP = 78;
const uint8_t STATUS_LED_READY_GPIO = 17;
const uint8_t STATUS_LED_HEARTBEAT_GPIO = 8;

const uint16_t LED_BOTTOM_PAIR_START_IDX = 0;
const uint16_t LED_BOTTOM_PAIR_END_IDX = 23;
const uint16_t LED_MIDDLE_START_IDX = 24;
const uint16_t LED_MIDDLE_END_IDX = 53;
const uint16_t LED_TOP_PAIR_START_IDX = 54;
const uint16_t LED_TOP_PAIR_END_IDX = 77;

const uint8_t COLOR_RUNNING_R = 40;
const uint8_t COLOR_RUNNING_G = 0;
const uint8_t COLOR_RUNNING_B = 0;

const uint8_t COLOR_BRAKE_R = 255;
const uint8_t COLOR_BRAKE_G = 0;
const uint8_t COLOR_BRAKE_B = 0;

const uint8_t COLOR_AMBER_R = 255;
const uint8_t COLOR_AMBER_G = 55;
const uint8_t COLOR_AMBER_B = 0;
const uint8_t COLOR_AMBER_W = 4;

const uint8_t DEFAULT_ANIMATION_R = 0;
const uint8_t DEFAULT_ANIMATION_G = 0;
const uint8_t DEFAULT_ANIMATION_B = 255;
const uint8_t DEFAULT_ANIMATION_W = 0;
const uint8_t BLE_ON_BLUE_R = 0;
const uint8_t BLE_ON_BLUE_G = 0;
const uint8_t BLE_ON_BLUE_B = 255;
const uint8_t BLE_ON_BLUE_W = 0;

const unsigned long BTN_DEBOUNCE_MS = 50;
const unsigned long TURN_BLINK_ON_MS = 500;
const unsigned long TURN_BLINK_OFF_MS = 500;
// current turn signal is set for 60-120 flashes per min
const unsigned long TURN_PULSE_MIN_HIGH_MS = 80;
const unsigned long TURN_PULSE_MAX_HIGH_MS = 750;
const unsigned long TURN_ACTIVITY_HOLD_MS = 1300;
const unsigned long STATUS_LED_HEARTBEAT_MS = 500;
const unsigned long ANIM_FRAME_INTERVAL_MS = 20;
const unsigned long BRAKE_SWEEP_DURATION_MS = 400;
const unsigned long BRAKE_SWEEP_FRAME_INTERVAL_MS = 20;
const unsigned long TURN_SWEEP_FRAME_INTERVAL_MS = 20;
const unsigned long LED_STARTUP_SETTLE_MS = 300;
const unsigned long LED_STARTUP_COLOR_FRAME_MS = 28;
const unsigned long LED_STARTUP_COLOR_HOLD_MS = 100;
const unsigned long LED_STARTUP_REFRESH_WINDOW_MS = 1200;
const unsigned long LED_STARTUP_REFRESH_INTERVAL_MS = 80;
const unsigned long LED_RECOVERY_REFRESH_INTERVAL_MS = 750;
const unsigned long LED_RECOVERY_REINIT_INTERVAL_MS = 5000;
const uint32_t WATCHDOG_TIMEOUT_SECONDS = 8;

enum RenderMode {
  RENDER_MODE_RUNNING = 0,
  RENDER_MODE_BRAKE,
  RENDER_MODE_TURN,
  RENDER_MODE_TURN_BRAKE,
  RENDER_MODE_HAZARD,
  RENDER_MODE_HAZARD_BRAKE,
  RENDER_MODE_BLE,
};

enum RunningLightMode {
  RUNNING_MODE_ALL_LINES = 0,
  RUNNING_MODE_OUTER_LINES,
  RUNNING_MODE_MIDDLE_LINE,
};

enum StripId {
  STRIP_L1 = 0,
  STRIP_L2,
  STRIP_R1,
  STRIP_R2,
  NUM_STRIPS
};

struct DebouncedInput {
  uint8_t pin;
  bool stableState;
  bool lastRawState;
  unsigned long lastChangeMs;
};

struct PulseTracker {
  bool initialized;
  bool lastLevel;
  unsigned long highSinceMs;
  unsigned long lastValidPulseMs;
};

struct RowLayout {
  uint16_t startIndex;
  uint8_t length;
  bool leftToRight;
};

struct StartupSweepColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t white;
};

struct RgbwColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t white;
};

struct WordScene {
  char letters[5];
  RgbwColor color;
};

Adafruit_NeoPixel stripL1(LEDS_PER_STRIP, LED_STRIP_L1_GPIO, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripL2(LEDS_PER_STRIP, LED_STRIP_L2_GPIO, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripR1(LEDS_PER_STRIP, LED_STRIP_R1_GPIO, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel stripR2(LEDS_PER_STRIP, LED_STRIP_R2_GPIO, NEO_GRBW + NEO_KHZ800);

Adafruit_NeoPixel *const strips[NUM_STRIPS] = {
    &stripL1,
    &stripL2,
    &stripR1,
    &stripR2,
};

const char *const stripNames[NUM_STRIPS] = {
    "L1",
    "L2",
    "R1",
    "R2",
};

const uint8_t stripGpios[NUM_STRIPS] = {
    LED_STRIP_L1_GPIO,
    LED_STRIP_L2_GPIO,
    LED_STRIP_R1_GPIO,
    LED_STRIP_R2_GPIO,
};

const uint8_t LEFT_SIDE_STRIPS[] = {STRIP_L1, STRIP_L2};
const uint8_t RIGHT_SIDE_STRIPS[] = {STRIP_R1, STRIP_R2};
const uint8_t PCB_ROW_COUNT = 6;
const RowLayout PCB_ROWS[PCB_ROW_COUNT] = {
    {0, 11, true},
    {11, 13, false},
    {24, 15, true},
    {39, 15, false},
    {54, 13, true},
    {67, 11, false},
};
const StartupSweepColor STARTUP_SWEEP_SEQUENCE[] = {
    {255, 0, 0, 0},
    {0, 255, 0, 0},
    {0, 0, 255, 0},
    {0, 0, 0, 180},
    {COLOR_AMBER_R, COLOR_AMBER_G, COLOR_AMBER_B, COLOR_AMBER_W},
};
const uint8_t BRAKE_SWEEP_EDGE_WIDTH = 52;

DebouncedInput brakeInput;
DebouncedInput leftInput;
DebouncedInput rightInput;

PulseTracker leftTracker = {false, false, 0, 0};
PulseTracker rightTracker = {false, false, 0, 0};

enum BleCommandType {
  BLE_COMMAND_NONE = 0,
  BLE_COMMAND_OFF,
  BLE_COMMAND_ON,
  BLE_COMMAND_RUN_MODE,
  BLE_COMMAND_ANIMATION,
  BLE_COMMAND_COLOR,
};

struct BleCommand {
  BleCommandType type;
  uint8_t value;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t white;
};

QueueHandle_t bleCommandQueue = NULL;
BLEServer *bleServer = NULL;
Preferences settingsPrefs;

bool bleActive = false;
bool bleSolidBlueMode = false;
bool bleDisplayDirty = false;
uint8_t selectedBlePattern = BLE_ANIMATION_DEFAULT_ID;
uint8_t blePattern = BLE_ANIMATION_DEFAULT_ID;
uint8_t animationColorR = DEFAULT_ANIMATION_R;
uint8_t animationColorG = DEFAULT_ANIMATION_G;
uint8_t animationColorB = DEFAULT_ANIMATION_B;
uint8_t animationColorW = DEFAULT_ANIMATION_W;
bool bleInterrupted = false;
volatile bool bleAdvertisingRestartRequested = false;
bool settingsReady = false;
RunningLightMode runningLightMode = RUNNING_MODE_ALL_LINES;
bool runningLightModeDirty = false;

uint32_t animationFrame = 0;
RenderMode lastRenderedMode = RENDER_MODE_RUNNING;
bool lastBlinkOn = false;
bool lastBrakeSignal = false;
unsigned long lastAnimationFrameMs = 0;
unsigned long lastHeartbeatToggleMs = 0;
bool heartbeatState = true;
unsigned long startupRefreshUntilMs = 0;
unsigned long lastStartupRefreshMs = 0;
unsigned long lastLedRecoveryRefreshMs = 0;
unsigned long lastLedRecoveryReinitMs = 0;
unsigned long lastTurnSweepFrameMs = 0;
unsigned long lastBrakeSweepFrameMs = 0;
bool brakeSweepActive = false;
unsigned long brakeSweepStartMs = 0;
uint16_t lastBrakeSweepProgress = 65535;
uint8_t lastTurnSweepColumns = 0;

static bool isInputActive(uint8_t pin) {
  return digitalRead(pin) == BTN_ACTIVE_LEVEL;
}

static void initDebouncedInput(DebouncedInput &input, uint8_t pin) {
  input.pin = pin;
  input.stableState = isInputActive(pin);
  input.lastRawState = input.stableState;
  input.lastChangeMs = millis();
}

static void updateDebouncedInput(DebouncedInput &input, unsigned long nowMs) {
  bool rawState = isInputActive(input.pin);

  if (rawState != input.lastRawState) {
    input.lastRawState = rawState;
    input.lastChangeMs = nowMs;
  }

  if ((nowMs - input.lastChangeMs) >= BTN_DEBOUNCE_MS &&
      input.stableState != input.lastRawState) {
    input.stableState = input.lastRawState;
  }
}

static void beginAllStrips() {
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    strips[stripId]->begin();
    strips[stripId]->clear();
  }
}

static void showAllStrips() {
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    strips[stripId]->show();
  }
}

static void initWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t watchdogConfig = {};
  watchdogConfig.timeout_ms = WATCHDOG_TIMEOUT_SECONDS * 1000;
  watchdogConfig.idle_core_mask = 0;
  watchdogConfig.trigger_panic = true;
  esp_task_wdt_init(&watchdogConfig);
#else
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);
#endif
  esp_task_wdt_add(NULL);
}

static void feedWatchdog() {
  esp_task_wdt_reset();
}

static void watchdogDelay(unsigned long delayMs) {
  unsigned long startMs = millis();
  while ((millis() - startMs) < delayMs) {
    unsigned long elapsedMs = millis() - startMs;
    unsigned long remainingMs = delayMs - elapsedMs;
    feedWatchdog();
    delay(remainingMs > 20 ? 20 : remainingMs);
  }
  feedWatchdog();
}

static uint32_t stripColor(uint8_t stripId, uint8_t red, uint8_t green,
                           uint8_t blue, uint8_t white) {
  return strips[stripId]->Color(red, green, blue, white);
}

static void fillStrip(uint8_t stripId, uint8_t red, uint8_t green,
                      uint8_t blue, uint8_t white) {
  uint32_t color = stripColor(stripId, red, green, blue, white);

  for (uint16_t i = 0; i < LEDS_PER_STRIP; i++) {
    strips[stripId]->setPixelColor(i, color);
  }
}

static void fillAllStrips(uint8_t red, uint8_t green, uint8_t blue,
                          uint8_t white) {
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    fillStrip(stripId, red, green, blue, white);
  }
}

static void fillRangeOnStrip(uint8_t stripId, uint16_t startIdx,
                             uint16_t endIdx, uint8_t red, uint8_t green,
                             uint8_t blue, uint8_t white) {
  if (startIdx >= LEDS_PER_STRIP || endIdx >= LEDS_PER_STRIP ||
      startIdx > endIdx) {
    return;
  }

  uint32_t color = stripColor(stripId, red, green, blue, white);
  for (uint16_t i = startIdx; i <= endIdx; i++) {
    strips[stripId]->setPixelColor(i, color);
  }
}

static void fillRangeAllStrips(uint16_t startIdx, uint16_t endIdx, uint8_t red,
                               uint8_t green, uint8_t blue, uint8_t white) {
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    fillRangeOnStrip(stripId, startIdx, endIdx, red, green, blue, white);
  }
}

static void fillOuterLinePairsAllStrips(uint8_t red, uint8_t green,
                                        uint8_t blue, uint8_t white) {
  fillRangeAllStrips(LED_BOTTOM_PAIR_START_IDX, LED_BOTTOM_PAIR_END_IDX, red,
                     green, blue, white);
  fillRangeAllStrips(LED_TOP_PAIR_START_IDX, LED_TOP_PAIR_END_IDX, red, green,
                     blue, white);
}

static void fillSelectedStrips(const uint8_t *stripIds, uint8_t count,
                               uint8_t red, uint8_t green, uint8_t blue,
                               uint8_t white) {
  for (uint8_t i = 0; i < count; i++) {
    fillStrip(stripIds[i], red, green, blue, white);
  }
}

static void setStripPixel(uint8_t stripId, uint16_t pixelIndex, uint8_t red,
                          uint8_t green, uint8_t blue, uint8_t white) {
  if (stripId >= NUM_STRIPS || pixelIndex >= LEDS_PER_STRIP) {
    return;
  }

  strips[stripId]->setPixelColor(
      pixelIndex, stripColor(stripId, red, green, blue, white));
}

static uint16_t rowColumnToIndex(const RowLayout &row, uint8_t column) {
  if (row.leftToRight) {
    return row.startIndex + column;
  }
  return row.startIndex + (row.length - 1 - column);
}

static bool isRightAnimationStrip(uint8_t stripId) {
  return stripId == STRIP_R1 || stripId == STRIP_R2;
}

static bool animationIndexToRowColumn(uint16_t pixelIndex, uint8_t &rowIdx,
                                      uint8_t &column) {
  for (uint8_t i = 0; i < PCB_ROW_COUNT; i++) {
    const RowLayout &row = PCB_ROWS[i];
    if (pixelIndex < row.startIndex ||
        pixelIndex >= (row.startIndex + row.length)) {
      continue;
    }

    rowIdx = i;
    if (row.leftToRight) {
      column = static_cast<uint8_t>(pixelIndex - row.startIndex);
    } else {
      column = static_cast<uint8_t>(
          row.startIndex + row.length - 1 - pixelIndex);
    }
    return true;
  }

  return false;
}

static uint16_t mapAnimationPixelIndex(uint8_t stripId, uint16_t pixelIndex) {
  if (!isRightAnimationStrip(stripId)) {
    return pixelIndex;
  }

  uint8_t rowIdx = 0;
  uint8_t column = 0;
  if (!animationIndexToRowColumn(pixelIndex, rowIdx, column)) {
    return pixelIndex;
  }

  const RowLayout &mirroredRow = PCB_ROWS[rowIdx];
  uint8_t mirroredColumn =
      static_cast<uint8_t>((mirroredRow.length - 1) - column);
  return rowColumnToIndex(mirroredRow, mirroredColumn);
}

static uint8_t scaleBrightness(uint8_t value, uint8_t brightness) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * brightness) / 255);
}

static void setAnimationPixel(uint8_t stripId, uint16_t pixelIndex,
                              uint8_t brightness) {
  uint16_t mappedPixelIndex = mapAnimationPixelIndex(stripId, pixelIndex);
  setStripPixel(stripId, mappedPixelIndex,
                scaleBrightness(animationColorR, brightness),
                scaleBrightness(animationColorG, brightness),
                scaleBrightness(animationColorB, brightness),
                scaleBrightness(animationColorW, brightness));
}

static uint8_t getMaxRowLength() {
  uint8_t maxLength = 0;
  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    if (PCB_ROWS[rowIdx].length > maxLength) {
      maxLength = PCB_ROWS[rowIdx].length;
    }
  }
  return maxLength;
}

static unsigned long getBlinkPhaseMs(unsigned long nowMs) {
  unsigned long cycleMs = TURN_BLINK_ON_MS + TURN_BLINK_OFF_MS;
  if (cycleMs == 0) {
    return 0;
  }
  return nowMs % cycleMs;
}

static uint8_t getTurnSweepColumns(unsigned long nowMs) {
  if (TURN_BLINK_ON_MS == 0) {
    return 0;
  }

  uint8_t maxColumns = getMaxRowLength();
  if (maxColumns == 0) {
    return 0;
  }

  unsigned long phaseMs = getBlinkPhaseMs(nowMs);
  if (phaseMs >= TURN_BLINK_ON_MS) {
    return 0;
  }

  uint8_t columns =
      1 + static_cast<uint8_t>((phaseMs * maxColumns) / TURN_BLINK_ON_MS);
  if (columns > maxColumns) {
    columns = maxColumns;
  }

  return columns;
}

static uint8_t getTurnArrowRowLag(uint8_t rowIdx) {
  switch (rowIdx) {
    case 0:
    case 5:
      return 4;

    case 1:
    case 4:
      return 2;

    default:
      return 0;
  }
}

static void renderTurnArrowOnStrips(const uint8_t *stripIds, uint8_t count,
                                    bool sweepLeftToRight,
                                    uint8_t visibleColumns) {
  if (visibleColumns == 0) {
    return;
  }

  const int tailColumns = 4;

  for (uint8_t stripPos = 0; stripPos < count; stripPos++) {
    uint8_t stripId = stripIds[stripPos];

    for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
      const RowLayout &row = PCB_ROWS[rowIdx];
      int rowLag = getTurnArrowRowLag(rowIdx);
      int rowHead = static_cast<int>(visibleColumns) - 1 - rowLag;
      if (rowHead < 0) {
        continue;
      }
      if (rowHead >= row.length) {
        rowHead = row.length - 1;
      }

      for (uint8_t column = 0; column < row.length; column++) {
        int visualColumn =
            sweepLeftToRight ? column : (row.length - 1 - column);
        int distanceBehind = rowHead - visualColumn;
        if (distanceBehind < 0 || distanceBehind > tailColumns) {
          continue;
        }

        uint8_t brightness =
            static_cast<uint8_t>(255 - distanceBehind * 35);
        uint16_t pixelIndex = rowColumnToIndex(row, column);
        strips[stripId]->setPixelColor(
            pixelIndex,
            stripColor(stripId, scaleBrightness(COLOR_AMBER_R, brightness),
                       scaleBrightness(COLOR_AMBER_G, brightness),
                       scaleBrightness(COLOR_AMBER_B, brightness),
                       scaleBrightness(COLOR_AMBER_W, brightness)));
      }
    }
  }
}

static RunningLightMode parseRunningLightMode(uint8_t storedValue) {
  switch (storedValue) {
    case RUNNING_MODE_OUTER_LINES:
      return RUNNING_MODE_OUTER_LINES;

    case RUNNING_MODE_MIDDLE_LINE:
      return RUNNING_MODE_MIDDLE_LINE;

    case RUNNING_MODE_ALL_LINES:
    default:
      return RUNNING_MODE_ALL_LINES;
  }
}

static bool isValidBlePattern(uint8_t patternId) {
  return patternId >= BLE_ANIMATION_MIN_ID &&
         patternId <= BLE_ANIMATION_MAX_ID;
}

static uint8_t parseBlePattern(uint8_t storedValue) {
  return isValidBlePattern(storedValue) ? storedValue : BLE_ANIMATION_DEFAULT_ID;
}

static const char *blePatternName(uint8_t patternId) {
  switch (patternId) {
    case 1:
      return "circular flow";

    case 2:
      return "row sweep";

    case 3:
      return "Tetris stack";

    case 4:
      return "center-out alert";

    case 5:
      return "stack fill";

    case 6:
      return "raindrop fall";

    case 7:
      return "night rider scanner";

    case 8:
      return "pulse wave";

    case 9:
      return "aurora ribbons";

    case 10:
      return "return-flow stack";

    case 11:
      return "top flash cascade";

    case 12:
      return "sequential intensity wash";

    case 13:
      return "single-row return stack";

    case 14:
      return "three-zone flicker";

    case 15:
      return "staged smooth flicker";

    case 16:
      return "mirror bridge return";

    case 17:
      return "randomized show mix";

    case 18:
      return "right-wipe terminal blink";

    case 19:
      return "center fold out-in";

    case 20:
      return "wave heartbeat";

    default:
      return "unknown";
  }
}

static const char *runningLightModeName(RunningLightMode mode) {
  switch (mode) {
    case RUNNING_MODE_OUTER_LINES:
      return "outer lines";

    case RUNNING_MODE_MIDDLE_LINE:
      return "middle line";

    case RUNNING_MODE_ALL_LINES:
    default:
      return "all lines";
  }
}

static void setRunningLightMode(RunningLightMode mode, bool persist) {
  if (runningLightMode != mode) {
    runningLightMode = mode;
    runningLightModeDirty = true;
  }

  if (persist && settingsReady) {
    settingsPrefs.putUChar(PREF_RUNNING_MODE_KEY, static_cast<uint8_t>(mode));
  }

  Serial.print("Running light mode: ");
  Serial.println(runningLightModeName(runningLightMode));
}

static void setSelectedBlePattern(uint8_t patternId, bool persist) {
  if (!isValidBlePattern(patternId)) {
    Serial.print("BLE animation rejected: ");
    Serial.println(patternId);
    return;
  }

  selectedBlePattern = patternId;

  if (persist && settingsReady) {
    settingsPrefs.putUChar(PREF_ANIMATION_PATTERN_KEY, selectedBlePattern);
  }

  Serial.print("Selected BLE animation ");
  Serial.print(selectedBlePattern);
  Serial.print(": ");
  Serial.println(blePatternName(selectedBlePattern));
}

static void setAnimationColor(uint8_t red, uint8_t green, uint8_t blue,
                              uint8_t white, bool persist) {
  animationColorR = red;
  animationColorG = green;
  animationColorB = blue;
  animationColorW = white;

  if (persist && settingsReady) {
    settingsPrefs.putUChar(PREF_ANIMATION_RED_KEY, animationColorR);
    settingsPrefs.putUChar(PREF_ANIMATION_GREEN_KEY, animationColorG);
    settingsPrefs.putUChar(PREF_ANIMATION_BLUE_KEY, animationColorB);
    settingsPrefs.putUChar(PREF_ANIMATION_WHITE_KEY, animationColorW);
  }

  Serial.print("Animation color RGBW: ");
  Serial.print(animationColorR);
  Serial.print(",");
  Serial.print(animationColorG);
  Serial.print(",");
  Serial.print(animationColorB);
  Serial.print(",");
  Serial.println(animationColorW);
}

static void initSettings() {
  settingsReady = settingsPrefs.begin(PREF_NAMESPACE, false);
  if (!settingsReady) {
    runningLightMode = RUNNING_MODE_ALL_LINES;
    selectedBlePattern = BLE_ANIMATION_DEFAULT_ID;
    blePattern = selectedBlePattern;
    setAnimationColor(DEFAULT_ANIMATION_R, DEFAULT_ANIMATION_G,
                      DEFAULT_ANIMATION_B, DEFAULT_ANIMATION_W, false);
    Serial.println("Settings unavailable; defaults loaded");
    return;
  }

  uint8_t storedMode =
      settingsPrefs.getUChar(PREF_RUNNING_MODE_KEY,
                             static_cast<uint8_t>(RUNNING_MODE_ALL_LINES));
  runningLightMode = parseRunningLightMode(storedMode);
  selectedBlePattern =
      parseBlePattern(settingsPrefs.getUChar(PREF_ANIMATION_PATTERN_KEY,
                                             BLE_ANIMATION_DEFAULT_ID));
  blePattern = selectedBlePattern;
  setAnimationColor(settingsPrefs.getUChar(PREF_ANIMATION_RED_KEY,
                                           DEFAULT_ANIMATION_R),
                    settingsPrefs.getUChar(PREF_ANIMATION_GREEN_KEY,
                                           DEFAULT_ANIMATION_G),
                    settingsPrefs.getUChar(PREF_ANIMATION_BLUE_KEY,
                                           DEFAULT_ANIMATION_B),
                    settingsPrefs.getUChar(PREF_ANIMATION_WHITE_KEY,
                                           DEFAULT_ANIMATION_W),
                    false);

  Serial.print("Loaded running light mode: ");
  Serial.println(runningLightModeName(runningLightMode));
  Serial.print("Loaded BLE animation ");
  Serial.print(selectedBlePattern);
  Serial.print(": ");
  Serial.println(blePatternName(selectedBlePattern));
}

static void renderRunningFrame() {
  fillAllStrips(0, 0, 0, 0);

  switch (runningLightMode) {
    case RUNNING_MODE_OUTER_LINES:
      fillOuterLinePairsAllStrips(COLOR_RUNNING_R, COLOR_RUNNING_G,
                                  COLOR_RUNNING_B, 0);
      break;

    case RUNNING_MODE_MIDDLE_LINE:
      fillRangeAllStrips(LED_MIDDLE_START_IDX, LED_MIDDLE_END_IDX,
                         COLOR_RUNNING_R, COLOR_RUNNING_G, COLOR_RUNNING_B, 0);
      break;

    case RUNNING_MODE_ALL_LINES:
    default:
      fillAllStrips(COLOR_RUNNING_R, COLOR_RUNNING_G, COLOR_RUNNING_B, 0);
      break;
  }
}

static void renderAllLinesRunningBaseFrame() {
  fillAllStrips(COLOR_RUNNING_R, COLOR_RUNNING_G, COLOR_RUNNING_B, 0);
}

static void renderStartupColorFrame(const StartupSweepColor &color,
                                    uint8_t brightness) {
  fillAllStrips(scaleBrightness(color.red, brightness),
                scaleBrightness(color.green, brightness),
                scaleBrightness(color.blue, brightness),
                scaleBrightness(color.white, brightness));
}

static void playStartupLightCheck() {
  static const uint8_t STARTUP_PULSE_LEVELS[] = {0, 36, 90, 160, 255, 160, 90, 36, 0};
  const uint8_t pulseFrameCount = sizeof(STARTUP_PULSE_LEVELS);

  const uint8_t startupSweepCount = sizeof(STARTUP_SWEEP_SEQUENCE) /
                                    sizeof(STARTUP_SWEEP_SEQUENCE[0]);
  for (uint8_t pass = 0; pass < startupSweepCount; pass++) {
    const StartupSweepColor &color = STARTUP_SWEEP_SEQUENCE[pass];

    for (uint8_t frame = 0; frame < pulseFrameCount; frame++) {
      renderStartupColorFrame(color, STARTUP_PULSE_LEVELS[frame]);
      showAllStrips();
      watchdogDelay(LED_STARTUP_COLOR_FRAME_MS);
    }

    watchdogDelay(LED_STARTUP_COLOR_HOLD_MS);
  }

  fillAllStrips(0, 0, 0, 0);
  showAllStrips();
  watchdogDelay(20);
}

static uint16_t getBrakeSweepFinalProgress() {
  return 255;
}

static uint16_t getBrakeSweepProgress(unsigned long nowMs) {
  unsigned long elapsedMs = nowMs - brakeSweepStartMs;
  if (elapsedMs >= BRAKE_SWEEP_DURATION_MS) {
    return getBrakeSweepFinalProgress();
  }

  return static_cast<uint16_t>(
      (static_cast<uint32_t>(elapsedMs) * getBrakeSweepFinalProgress()) /
      BRAKE_SWEEP_DURATION_MS);
}

static uint8_t brakeSweepRedForDistance(uint16_t progress, uint8_t distance,
                                        uint8_t maxDistance) {
  if (maxDistance == 0) {
    return COLOR_BRAKE_R;
  }

  uint16_t distanceStart =
      (static_cast<uint16_t>(distance) * getBrakeSweepFinalProgress()) /
      maxDistance;
  if (progress + BRAKE_SWEEP_EDGE_WIDTH <= distanceStart) {
    return COLOR_RUNNING_R;
  }

  if (progress >= distanceStart + BRAKE_SWEEP_EDGE_WIDTH) {
    return COLOR_BRAKE_R;
  }

  uint16_t localProgress =
      progress + BRAKE_SWEEP_EDGE_WIDTH > distanceStart
          ? progress + BRAKE_SWEEP_EDGE_WIDTH - distanceStart
          : 0;
  uint16_t red =
      COLOR_RUNNING_R +
      ((static_cast<uint16_t>(COLOR_BRAKE_R - COLOR_RUNNING_R) *
        localProgress) /
       BRAKE_SWEEP_EDGE_WIDTH);
  return red > COLOR_BRAKE_R ? COLOR_BRAKE_R : static_cast<uint8_t>(red);
}

static void renderBrakeSweepFrame(uint16_t progress) {
  renderAllLinesRunningBaseFrame();

  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
      const RowLayout &row = PCB_ROWS[rowIdx];
      int centerColumn = row.length / 2;
      uint8_t maxDistance =
          static_cast<uint8_t>(centerColumn > (row.length - 1 - centerColumn)
                                   ? centerColumn
                                   : (row.length - 1 - centerColumn));

      for (uint8_t column = 0; column < row.length; column++) {
        int distanceFromCenter = column - centerColumn;
        if (distanceFromCenter < 0) {
          distanceFromCenter = -distanceFromCenter;
        }

        uint8_t red =
            brakeSweepRedForDistance(progress,
                                     static_cast<uint8_t>(distanceFromCenter),
                                     maxDistance);
        if (red > COLOR_RUNNING_R) {
          uint16_t pixelIndex = rowColumnToIndex(row, column);
          strips[stripId]->setPixelColor(pixelIndex,
                                         stripColor(stripId, red, 0, 0, 0));
        }
      }
    }
  }
}

static void renderBrakeBaseFrame(unsigned long nowMs) {
  if (brakeSweepActive) {
    uint16_t progress = getBrakeSweepProgress(nowMs);
    if (progress >= getBrakeSweepFinalProgress()) {
      fillAllStrips(COLOR_BRAKE_R, COLOR_BRAKE_G, COLOR_BRAKE_B, 0);
    } else {
      renderBrakeSweepFrame(progress);
    }
    return;
  }

  fillAllStrips(COLOR_BRAKE_R, COLOR_BRAKE_G, COLOR_BRAKE_B, 0);
}

static void renderTurnFrame(bool leftActive, bool rightActive, bool brakeActive,
                            bool blinkOn, unsigned long nowMs) {
  if (brakeActive) {
    renderBrakeBaseFrame(nowMs);
  } else {
    renderRunningFrame();
  }

  if (leftActive) {
    fillSelectedStrips(LEFT_SIDE_STRIPS, 2, 0, 0, 0, 0);
  }
  if (rightActive) {
    fillSelectedStrips(RIGHT_SIDE_STRIPS, 2, 0, 0, 0, 0);
  }

  if (blinkOn) {
    uint8_t visibleColumns = getTurnSweepColumns(nowMs);
    if (leftActive) {
      renderTurnArrowOnStrips(LEFT_SIDE_STRIPS, 2, false, visibleColumns);
    }
    if (rightActive) {
      renderTurnArrowOnStrips(RIGHT_SIDE_STRIPS, 2, true, visibleColumns);
    }
  }
}

static void renderHazardFrame(bool brakeActive, bool blinkOn,
                              unsigned long nowMs) {
  if (brakeActive) {
    renderBrakeBaseFrame(nowMs);
    if (blinkOn) {
      fillRangeAllStrips(LED_MIDDLE_START_IDX, LED_MIDDLE_END_IDX,
                         COLOR_AMBER_R, COLOR_AMBER_G, COLOR_AMBER_B,
                         COLOR_AMBER_W);
    }
  } else {
    fillAllStrips(0, 0, 0, 0);
    if (blinkOn) {
      fillAllStrips(COLOR_AMBER_R, COLOR_AMBER_G, COLOR_AMBER_B, COLOR_AMBER_W);
    }
  }
}

static void patternBlueCircularFlow(uint32_t frame) {
  // All indices are 0-based (LED number - 1).
  static const uint8_t frameA[] = {
    // row 1 (L->R): LEDs 4-8
    3, 4, 5, 6, 7,8,
    // row 2 (R->L): LEDs 12-24
    15, 16, 17, 18, 19, 20,
    // rows 3 and 4: ON as well
    //row 3 (L->R):LEDs 25-39
    34, 35, 36, 37, 38,
    //row 4 (R->L):LEDs 40-54
    39, 40, 41, 42, 43,
    // row 5 (L->R): LEDs 55-67
    54, 55, 56, 57, 60, 61, 62, 65, 66,
    // row 6 (R->L): LEDs 68-78
    67, 68, 69, 72,73,74,
  };
  static const uint8_t frameB[] = {
    // row 1 (L->R): LEDs 1-4, 8-11
    0, 1, 2, 3, 7, 8, 9, 10,
    // row 2: off
    // row 3 (L->R): LEDs 36-39
    24,25,26,27,31,32,33,
    // row 4 (R->L): LEDs 43-46
    46,47,48,49,50,
    // row 5 (L->R): LEDs 55-58, 62-65
    58,59, 63, 64,
    // row 6 (R->L): LEDs 69-72, 75-78
    70,71,75,76,77,
  };

  // Each frame is ON for 500 ms then the other frame turns ON for 500 ms.
  // 500 ms / 20 ms per render tick = 25 ticks per half.
  const uint32_t framesPerHalf = 25;
  bool showA = (frame / framesPerHalf) % 2 == 0;

  const uint8_t *pixels = showA ? frameA : frameB;
  uint8_t count = showA ? sizeof(frameA) : sizeof(frameB);

  fillAllStrips(0, 0, 0, 0);
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
      setAnimationPixel(stripId, pixels[i], 255);
    }
  }
}

static void patternGreenSweep(uint32_t frame) {
  // All 6 rows start very dim green. A 3-row bright window descends from the
  // top pair to the bottom pair (4 steps). When it reaches the bottom, all
  // LEDs flash full green, then all return to very dim and the sweep repeats.
  //
  // Cycle:  sweep (4 × 10 ticks = 800 ms)
  //       + full bright (25 ticks = 500 ms)
  //       + dim pause   (20 ticks = 400 ms)
  //       = 85 ticks ≈ 1.7 s per loop
  //
  // Sweep positions (topRow = highest row index in the 3-row window):
  //   pos 0: rows 5,4,3  (top two rows light up first)
  //   pos 1: rows 4,3,2
  //   pos 2: rows 3,2,1
  //   pos 3: rows 2,1,0  (bottom two rows last)

  static const uint8_t TICKS_PER_STEP = 10;
  static const uint8_t SWEEP_STEPS    = 4;
  static const uint8_t TICKS_FULL     = 25;
  static const uint8_t TICKS_DIM      = 20;
  static const uint32_t CYCLE_TICKS   = (uint32_t)SWEEP_STEPS * TICKS_PER_STEP
                                         + TICKS_FULL + TICKS_DIM;  // 85

  static const uint8_t BASE_G = 15;   // very dim green
  static const uint8_t FULL_G = 255;  // full bright green

  // Row boundaries (row 0 = bottom, row 5 = top)
  static const uint8_t rowStart[6] = {0, 11, 24, 39, 54, 67};
  static const uint8_t rowCount[6] = {11, 13, 15, 15, 13, 11};

  uint32_t tick = frame % CYCLE_TICKS;
  uint32_t sweepEnd = (uint32_t)SWEEP_STEPS * TICKS_PER_STEP;

  uint8_t phase;  // 0=sweep, 1=full bright, 2=dim
  if (tick < sweepEnd) {
    phase = 0;
  } else if (tick < sweepEnd + TICKS_FULL) {
    phase = 1;
  } else {
    phase = 2;
  }

  fillAllStrips(0, 0, 0, 0);

  for (uint8_t s = 0; s < NUM_STRIPS; s++) {
    for (uint8_t r = 0; r < 6; r++) {
      uint8_t g;
      if (phase == 1) {
        g = FULL_G;
      } else if (phase == 2) {
        g = BASE_G;
      } else {
        uint8_t sweepPos = (uint8_t)(tick / TICKS_PER_STEP);
        uint8_t topRow   = 5 - sweepPos;  // topRow: 5,4,3,2 for pos 0,1,2,3
        // 3-row window: topRow, topRow-1, topRow-2  (all >= 0 since topRow >= 2)
        g = (r == topRow || r == topRow - 1 || r == (uint8_t)(topRow - 2))
              ? FULL_G : BASE_G;
      }
      for (uint8_t c = 0; c < rowCount[r]; c++) {
        setAnimationPixel(s, rowStart[r] + c, g);
      }
    }
  }
}

static void setPattern13AnimationPixel(uint8_t stripId, uint16_t pixelIndex,
                                       uint8_t brightness) {
  setAnimationPixel(stripId, pixelIndex, brightness);
}

static const uint8_t PATTERN3_SLOT_A_START[] = {25, 1, 5, 9,
                                                39, 67, 63, 59};
static const uint8_t PATTERN3_SLOT_A_END[] = {29, 4, 8, 11,
                                              36, 64, 60, 55};
static const uint8_t PATTERN3_SLOT_B_START[] = {54, 24, 20, 16,
                                                40, 68, 72, 76};
static const uint8_t PATTERN3_SLOT_B_END[] = {50, 21, 17, 12,
                                              43, 71, 75, 78};
static const uint8_t PATTERN3_SLOT_COUNT =
    sizeof(PATTERN3_SLOT_A_START) / sizeof(PATTERN3_SLOT_A_START[0]);
static const uint8_t PATTERN3_SWEEP_STEP_FRAMES = 6;

static void renderPattern3LedRange(uint8_t firstLed, uint8_t lastLed,
                                   uint8_t brightness) {
  if (brightness == 0 || firstLed < 1 || firstLed > LEDS_PER_STRIP ||
      lastLed < 1 || lastLed > LEDS_PER_STRIP) {
    return;
  }

  int8_t step = firstLed <= lastLed ? 1 : -1;
  for (uint8_t ledNumber = firstLed;; ledNumber += step) {
    uint16_t pixelIndex = static_cast<uint16_t>(ledNumber - 1);
    for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
      setPattern13AnimationPixel(stripId, pixelIndex, brightness);
    }

    if (ledNumber == lastLed) {
      break;
    }
  }
}

static void renderPattern3Slot(uint8_t slot, uint8_t brightness) {
  if (slot >= PATTERN3_SLOT_COUNT) {
    return;
  }

  renderPattern3LedRange(PATTERN3_SLOT_A_START[slot],
                         PATTERN3_SLOT_A_END[slot], brightness);
  renderPattern3LedRange(PATTERN3_SLOT_B_START[slot],
                         PATTERN3_SLOT_B_END[slot], brightness);
}

static uint8_t pattern3LerpLed(uint8_t startLed, uint8_t endLed,
                               uint8_t step, uint8_t maxStep) {
  if (maxStep == 0 || step >= maxStep) {
    return endLed;
  }

  int16_t startValue = startLed;
  int16_t endValue = endLed;
  int16_t value =
      startValue + (((endValue - startValue) * step) / maxStep);
  if (value < 1) {
    value = 1;
  } else if (value > LEDS_PER_STRIP) {
    value = LEDS_PER_STRIP;
  }
  return static_cast<uint8_t>(value);
}

static uint8_t pattern3ScaleBrightness(uint8_t brightness, uint8_t scale) {
  return static_cast<uint8_t>((static_cast<uint16_t>(brightness) * scale) / 255);
}

static uint8_t pattern3EaseInOut(uint8_t t) {
  uint32_t x = t;
  uint32_t eased = (x * x * (765 - (2 * x))) / 65025;
  return static_cast<uint8_t>(eased > 255 ? 255 : eased);
}

static void renderPattern3SingleLed(uint8_t ledNumber, uint8_t brightness) {
  renderPattern3LedRange(ledNumber, ledNumber, brightness);
}

static void renderPattern3LaneCrossfade(const uint8_t *sourceLeds,
                                        const uint8_t *targetLeds,
                                        uint8_t laneCount, uint8_t step,
                                        uint8_t maxStep,
                                        uint8_t brightness) {
  if (laneCount == 0) {
    return;
  }

  uint8_t clampedStep = step > maxStep ? maxStep : step;
  uint8_t baseRaw = static_cast<uint8_t>(
      (static_cast<uint16_t>(clampedStep) * 255) / (maxStep == 0 ? 1 : maxStep));

  for (uint8_t lane = 0; lane < laneCount; lane++) {
    uint8_t laneDelay = static_cast<uint8_t>(lane * 20);
    uint8_t laneRaw = baseRaw > laneDelay ? static_cast<uint8_t>(baseRaw - laneDelay) : 0;
    uint8_t laneEased = pattern3EaseInOut(laneRaw);
    uint8_t sourceBrightness =
        pattern3ScaleBrightness(brightness, static_cast<uint8_t>(255 - laneEased));
    uint8_t targetBrightness = pattern3ScaleBrightness(brightness, laneEased);
    renderPattern3SingleLed(sourceLeds[lane], sourceBrightness);
    renderPattern3SingleLed(targetLeds[lane], targetBrightness);
  }
}

static void renderPattern3TopWindow(uint8_t step, uint8_t brightness) {
  static const uint8_t TOP_SWEEP_LERP_MAX = 11;
  uint8_t topStart = pattern3LerpLed(78, 71, step, TOP_SWEEP_LERP_MAX);
  uint8_t secondStart = pattern3LerpLed(55, 64, step, TOP_SWEEP_LERP_MAX);
  renderPattern3LedRange(topStart, static_cast<uint8_t>(topStart - 3),
                         brightness);
  renderPattern3LedRange(secondStart, static_cast<uint8_t>(secondStart + 3),
                         brightness);
}

static void renderPattern3MiddleWindow(uint8_t brightness) {
  renderPattern3LedRange(39, 36, brightness);
  renderPattern3LedRange(40, 43, brightness);
}

static void renderOppositeMiddleUpperBlock(uint8_t brightness) {
  renderPattern3LedRange(54, 50, brightness);
}

static void renderOppositeMiddleLowerBlock(uint8_t brightness) {
  renderPattern3LedRange(25, 29, brightness);
}

static void renderOppositeMiddleBlock(uint8_t brightness) {
  renderOppositeMiddleUpperBlock(brightness);
  renderOppositeMiddleLowerBlock(brightness);
}

static void renderPattern3TopToMiddleWindow(uint8_t step, uint8_t brightness) {
  static const uint8_t DROP_LERP_MAX = 11;
  static const uint8_t sourceRight[] = {71, 70, 69, 68};
  static const uint8_t targetRight[] = {43, 42, 41, 40};
  static const uint8_t sourceLeft[] = {64, 65, 66, 67};
  static const uint8_t targetLeft[] = {36, 37, 38, 39};

  renderPattern3LaneCrossfade(sourceRight, targetRight, 4, step, DROP_LERP_MAX,
                              brightness);
  renderPattern3LaneCrossfade(sourceLeft, targetLeft, 4, step, DROP_LERP_MAX,
                              brightness);
}

static void renderPattern3MiddleOriginWindow(uint8_t brightness) {
  renderPattern3Slot(0, brightness);
}

static void renderPattern3BottomToMiddleOriginWindow(uint8_t step,
                                                     uint8_t brightness) {
  static const uint8_t DROP_LERP_MAX = 11;
  static const uint8_t sourceUpper[] = {24, 23, 22, 21, 20};
  static const uint8_t targetUpper[] = {54, 53, 52, 51, 50};
  static const uint8_t sourceLower[] = {1, 2, 3, 4, 5};
  static const uint8_t targetLower[] = {25, 26, 27, 28, 29};

  renderPattern3LaneCrossfade(sourceUpper, targetUpper, 5, step, DROP_LERP_MAX,
                              brightness);
  renderPattern3LaneCrossfade(sourceLower, targetLower, 5, step, DROP_LERP_MAX,
                              brightness);
}

static void renderPattern3MiddleToBottomWindow(uint8_t step,
                                               uint8_t brightness) {
  static const uint8_t DROP_LERP_MAX = 11;
  static const uint8_t sourceRight[] = {43, 42, 41, 40};
  static const uint8_t targetRight[] = {15, 14, 13, 12};
  static const uint8_t sourceLeft[] = {39, 38, 37, 36};
  static const uint8_t targetLeft[] = {11, 10, 9, 8};

  renderPattern3LaneCrossfade(sourceRight, targetRight, 4, step, DROP_LERP_MAX,
                              brightness);
  renderPattern3LaneCrossfade(sourceLeft, targetLeft, 4, step, DROP_LERP_MAX,
                              brightness);
}

static void renderPattern3BottomWindow(uint8_t targetSlot, uint8_t step,
                                       uint8_t brightness) {
  static const uint8_t BOTTOM_SWEEP_LERP_MAX = 11;
  static const uint8_t bottomTopFinalStarts[] = {21, 17, 12};
  static const uint8_t bottomLowerFinalStarts[] = {4, 8, 11};
  static const uint8_t bottomTopLengths[] = {4, 4, 5};
  static const uint8_t bottomLowerLengths[] = {4, 4, 3};

  if (targetSlot > 2) {
    return;
  }

  uint8_t upperStart =
      pattern3LerpLed(12, bottomTopFinalStarts[targetSlot], step,
                      BOTTOM_SWEEP_LERP_MAX);
  uint8_t lowerStart =
      pattern3LerpLed(11, bottomLowerFinalStarts[targetSlot], step,
                      BOTTOM_SWEEP_LERP_MAX);
  renderPattern3LedRange(
      upperStart,
      static_cast<uint8_t>(upperStart + bottomTopLengths[targetSlot] - 1),
      brightness);
  renderPattern3LedRange(
      lowerStart,
      static_cast<uint8_t>(lowerStart - bottomLowerLengths[targetSlot] + 1),
      brightness);
}

static void renderPattern3TopTargetWindow(uint8_t targetSlot, uint8_t step,
                                          uint8_t brightness) {
  static const uint8_t TOP_SWEEP_STEPS = 12;
  static const uint8_t FIRST_TOP_SLOT = 5;

  if (targetSlot < FIRST_TOP_SLOT || targetSlot >= PATTERN3_SLOT_COUNT) {
    return;
  }

  if (step < TOP_SWEEP_STEPS) {
    renderPattern3TopWindow(step, brightness);
  } else {
    renderPattern3Slot(targetSlot, brightness);
  }
}

static void renderPattern3ActiveSlotPath(uint8_t activeSlot, uint8_t pathStep,
                                         uint8_t brightness,
                                         uint8_t topSweepSteps,
                                         uint8_t topToMiddleSteps,
                                         uint8_t middleHoldSteps,
                                         uint8_t middleToBottomSteps) {
  if (activeSlot == 0) {
    if (pathStep < topSweepSteps) {
      renderPattern3TopWindow(pathStep, brightness);
    } else if (pathStep < topSweepSteps + topToMiddleSteps) {
      renderPattern3TopToMiddleWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps), brightness);
    } else if (pathStep < topSweepSteps + topToMiddleSteps + middleHoldSteps) {
      renderPattern3MiddleWindow(brightness);
    } else if (pathStep <
               topSweepSteps + topToMiddleSteps + middleHoldSteps +
                   middleToBottomSteps) {
      renderPattern3MiddleToBottomWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps - topToMiddleSteps -
                               middleHoldSteps),
          brightness);
    } else {
      renderPattern3BottomToMiddleOriginWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps - topToMiddleSteps -
                               middleHoldSteps - middleToBottomSteps),
          brightness);
    }
  } else if (activeSlot <= 3) {
    if (pathStep < topSweepSteps) {
      renderPattern3TopWindow(pathStep, brightness);
    } else if (pathStep < topSweepSteps + topToMiddleSteps) {
      renderPattern3TopToMiddleWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps), brightness);
    } else if (pathStep < topSweepSteps + topToMiddleSteps + middleHoldSteps) {
      renderPattern3MiddleWindow(brightness);
    } else if (pathStep <
               topSweepSteps + topToMiddleSteps + middleHoldSteps +
                   middleToBottomSteps) {
      renderPattern3MiddleToBottomWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps - topToMiddleSteps -
                               middleHoldSteps),
          brightness);
    } else {
      renderPattern3BottomWindow(
          static_cast<uint8_t>(activeSlot - 1),
          static_cast<uint8_t>(pathStep - topSweepSteps - topToMiddleSteps -
                                middleHoldSteps - middleToBottomSteps),
          brightness);
    }
  } else if (activeSlot == 4) {
    if (pathStep < topSweepSteps) {
      renderPattern3TopWindow(pathStep, brightness);
    } else if (pathStep < topSweepSteps + topToMiddleSteps) {
      renderPattern3TopToMiddleWindow(
          static_cast<uint8_t>(pathStep - topSweepSteps), brightness);
    } else {
      renderPattern3MiddleWindow(brightness);
    }
  } else {
    renderPattern3TopTargetWindow(activeSlot, pathStep, brightness);
  }
}

static void renderPattern3SweepPhase(bool forwardSweep, uint8_t sweepStep,
                                     uint8_t frameInStep, uint8_t stepFrames,
                                     uint8_t fullBrightness) {
  static const uint8_t FORWARD_ORDER[PATTERN3_SLOT_COUNT] = {7, 6, 5, 4,
                                                             3, 2, 1, 0};
  static const uint8_t BACKWARD_ORDER[PATTERN3_SLOT_COUNT] = {0, 1, 2, 3,
                                                              4, 5, 6, 7};
  const uint8_t *order = forwardSweep ? FORWARD_ORDER : BACKWARD_ORDER;
  uint8_t transitioningBrightness = fullBrightness;

  if (stepFrames > 1) {
    uint8_t phase = static_cast<uint8_t>(
        (static_cast<uint16_t>(frameInStep) * 255) / (stepFrames - 1));
    transitioningBrightness = forwardSweep
                                  ? pattern3ScaleBrightness(
                                        fullBrightness,
                                        static_cast<uint8_t>(255 - phase))
                                  : pattern3ScaleBrightness(fullBrightness, phase);
  }

  for (uint8_t i = 0; i < PATTERN3_SLOT_COUNT; i++) {
    uint8_t slotId = order[i];
    if (forwardSweep) {
      if (i < sweepStep) {
        continue;
      }
      if (i == sweepStep) {
        renderPattern3Slot(slotId, transitioningBrightness);
      } else {
        renderPattern3Slot(slotId, fullBrightness);
      }
    } else {
      if (i < sweepStep) {
        renderPattern3Slot(slotId, fullBrightness);
      } else if (i == sweepStep) {
        renderPattern3Slot(slotId, transitioningBrightness);
      }
    }
  }
}

static void patternBluePerimeterFlow(uint32_t frame) {
  static const uint8_t SPEED_MULTIPLIER = 2;
  static const uint8_t TOP_SWEEP_STEPS = 12;
  static const uint8_t TOP_TO_MIDDLE_STEPS = 12;
  static const uint8_t MIDDLE_HOLD_STEPS = 1;
  static const uint8_t MIDDLE_TO_BOTTOM_STEPS = 12;
  static const uint8_t BOTTOM_SWEEP_STEPS = 12;
  static const uint8_t FRAMES_PER_PATH_STEP = 1;
  static const uint8_t ACTIVE_BRIGHTNESS = 255;
  static const uint8_t TRAIL_BRIGHTNESS = 122;
  static const uint8_t TRAIL2_BRIGHTNESS = 62;
  static const uint8_t LEAD_BRIGHTNESS = 36;
  static const uint8_t LOCKED_BRIGHTNESS = 165;

  const uint8_t pathSteps =
      static_cast<uint8_t>(TOP_SWEEP_STEPS + TOP_TO_MIDDLE_STEPS +
                           MIDDLE_HOLD_STEPS + MIDDLE_TO_BOTTOM_STEPS +
                           BOTTOM_SWEEP_STEPS);
  const uint16_t framesPerSlot =
      static_cast<uint16_t>(pathSteps) * FRAMES_PER_PATH_STEP;
  const uint16_t buildFrames =
      static_cast<uint16_t>(PATTERN3_SLOT_COUNT) * framesPerSlot;
  const uint16_t sweepFrames =
      static_cast<uint16_t>(PATTERN3_SLOT_COUNT) *
      PATTERN3_SWEEP_STEP_FRAMES;
  const uint16_t totalCycleFrames =
      static_cast<uint16_t>(buildFrames + sweepFrames + sweepFrames);
  uint32_t scaledFrame = static_cast<uint32_t>(frame) * SPEED_MULTIPLIER;
  uint16_t cycleFrame = static_cast<uint16_t>(scaledFrame % totalCycleFrames);

  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame >= buildFrames) {
    uint16_t sweepFrame = static_cast<uint16_t>(cycleFrame - buildFrames);
    bool forwardSweep = sweepFrame < sweepFrames;
    uint16_t localSweepFrame =
        forwardSweep ? sweepFrame : static_cast<uint16_t>(sweepFrame - sweepFrames);
    uint8_t sweepStep =
        static_cast<uint8_t>(localSweepFrame / PATTERN3_SWEEP_STEP_FRAMES);
    if (sweepStep >= PATTERN3_SLOT_COUNT) {
      sweepStep = static_cast<uint8_t>(PATTERN3_SLOT_COUNT - 1);
    }
    uint8_t frameInStep =
        static_cast<uint8_t>(localSweepFrame % PATTERN3_SWEEP_STEP_FRAMES);
    renderPattern3SweepPhase(forwardSweep, sweepStep, frameInStep,
                             PATTERN3_SWEEP_STEP_FRAMES, LOCKED_BRIGHTNESS);
    return;
  }

  uint8_t lockedSlots = static_cast<uint8_t>(cycleFrame / framesPerSlot);
  uint8_t activeSlot = lockedSlots;
  uint16_t localFrame = static_cast<uint16_t>(cycleFrame % framesPerSlot);
  uint8_t pathStep = static_cast<uint8_t>(localFrame / FRAMES_PER_PATH_STEP);

  for (uint8_t slot = 0; slot < lockedSlots; slot++) {
    renderPattern3Slot(slot, LOCKED_BRIGHTNESS);
  }

  if (activeSlot >= PATTERN3_SLOT_COUNT) {
    return;
  }

  // Render one trailing frame behind the active block to smooth transitions.
  if (pathStep > 0) {
    renderPattern3ActiveSlotPath(
        activeSlot, static_cast<uint8_t>(pathStep - 1), TRAIL_BRIGHTNESS,
        TOP_SWEEP_STEPS, TOP_TO_MIDDLE_STEPS, MIDDLE_HOLD_STEPS,
        MIDDLE_TO_BOTTOM_STEPS);
  }
  if (pathStep > 1) {
    renderPattern3ActiveSlotPath(
        activeSlot, static_cast<uint8_t>(pathStep - 2), TRAIL2_BRIGHTNESS,
        TOP_SWEEP_STEPS, TOP_TO_MIDDLE_STEPS, MIDDLE_HOLD_STEPS,
        MIDDLE_TO_BOTTOM_STEPS);
  }
  if (pathStep + 1 < pathSteps) {
    renderPattern3ActiveSlotPath(
        activeSlot, static_cast<uint8_t>(pathStep + 1), LEAD_BRIGHTNESS,
        TOP_SWEEP_STEPS, TOP_TO_MIDDLE_STEPS, MIDDLE_HOLD_STEPS,
        MIDDLE_TO_BOTTOM_STEPS);
  }

  renderPattern3ActiveSlotPath(activeSlot, pathStep, ACTIVE_BRIGHTNESS,
                               TOP_SWEEP_STEPS, TOP_TO_MIDDLE_STEPS,
                               MIDDLE_HOLD_STEPS, MIDDLE_TO_BOTTOM_STEPS);
}

static void renderPattern10BaseBandsWithSourceLevel(uint8_t brightness,
                                                     uint8_t sourceBrightness) {
  renderPattern3LedRange(1, 6, brightness);
  renderPattern3LedRange(24, 18, brightness);

  // Keep the fixed top/base LEDs visible, but allow the source blocks
  // (75-72 and 59-62) to look depleted while the shuttle travels down.
  // This makes the eye read those LEDs as moving away instead of seeing
  // the same bright top band redrawn every frame.
  renderPattern3LedRange(78, 76, brightness);
  renderPattern3LedRange(75, 72, sourceBrightness);
  renderPattern3LedRange(55, 58, brightness);
  renderPattern3LedRange(59, 62, sourceBrightness);
}

static void renderPattern10BaseBands(uint8_t brightness) {
  renderPattern10BaseBandsWithSourceLevel(brightness, brightness);
}

static void renderPattern10ProgressiveTopSource(uint8_t completedPasses,
                                                uint8_t brightness) {
  // Each completed down/up pass locks the pair that was just "used" from
  // the top source area. This matches the required sequence:
  // pass 1 -> 75,74 and 59,60 stay ON
  // pass 2 -> 73,72 and 61,62 stay ON
  // pass 3 -> 71,70 and 63,64 stay ON
  // pass 4 -> 69,68 and 65,66,67 stay ON to complete the top pair.
  static const uint8_t LOCK_PASS_0[] = {75, 74, 59, 60};
  static const uint8_t LOCK_PASS_1[] = {73, 72, 61, 62};
  static const uint8_t LOCK_PASS_2[] = {71, 70, 63, 64};
  static const uint8_t LOCK_PASS_3[] = {69, 68, 65, 66, 67};
  static const uint8_t *LOCKS[] = {
      LOCK_PASS_0, LOCK_PASS_1, LOCK_PASS_2, LOCK_PASS_3};
  static const uint8_t COUNTS[] = {4, 4, 4, 5};

  uint8_t passCount = completedPasses > 4 ? 4 : completedPasses;
  for (uint8_t pass = 0; pass < passCount; pass++) {
    for (uint8_t i = 0; i < COUNTS[pass]; i++) {
      renderPattern3LedRange(LOCKS[pass][i], LOCKS[pass][i], brightness);
    }
  }
}

static void renderPattern10TopFull(uint8_t brightness) {
  renderPattern3LedRange(78, 68, brightness);
  renderPattern3LedRange(55, 67, brightness);
}

static void renderPattern10OppositeMiddleUpper(uint8_t brightness) {
  renderOppositeMiddleUpperBlock(brightness);
}

static void renderPattern10OppositeMiddleLower(uint8_t brightness) {
  renderOppositeMiddleLowerBlock(brightness);
}

static void renderPattern10OppositeMiddleBlock(uint8_t brightness) {
  renderOppositeMiddleBlock(brightness);
}

static void renderPattern10TargetRange(uint8_t targetIndex, uint8_t brightness) {
  switch (targetIndex) {
    case 0:
      renderPattern3LedRange(8, 11, brightness);
      break;

    case 1:
      renderPattern3LedRange(12, 15, brightness);
      break;

    case 2:
      renderPattern3LedRange(39, 36, brightness);
      break;

    case 3:
      renderPattern3LedRange(40, 43, brightness);
      break;
  }
}

static void renderPattern10LockedTargets(uint8_t lockedCount, uint8_t brightness) {
  for (uint8_t i = 0; i < lockedCount && i < 4; i++) {
    renderPattern10TargetRange(i, brightness);
  }
}

static bool pattern10ListContains(const uint8_t *leds, uint8_t count,
                                  uint8_t led) {
  for (uint8_t i = 0; i < count; i++) {
    if (leds[i] == led) {
      return true;
    }
  }
  return false;
}

static void renderPattern10LedList(const uint8_t *leds, uint8_t count,
                                   uint8_t brightness) {
  for (uint8_t i = 0; i < count; i++) {
    renderPattern3LedRange(leds[i], leds[i], brightness);
  }
}

static void renderPattern10CrossfadeList(const uint8_t *fromLeds,
                                         uint8_t fromCount,
                                         const uint8_t *toLeds,
                                         uint8_t toCount,
                                         uint8_t blend,
                                         uint8_t brightness) {
  uint8_t eased = pattern3EaseInOut(blend);
  uint8_t fromBrightness = pattern3ScaleBrightness(
      brightness, static_cast<uint8_t>(255 - eased));
  uint8_t toBrightness = pattern3ScaleBrightness(brightness, eased);

  // Shared LEDs must stay fully bright. Only the old-only tail fades down and
  // the new-only head fades up. This is what makes the motion look like a
  // sliding window instead of a hard stepping block.
  for (uint8_t i = 0; i < fromCount; i++) {
    uint8_t led = fromLeds[i];
    if (pattern10ListContains(toLeds, toCount, led)) {
      renderPattern3LedRange(led, led, brightness);
    } else {
      renderPattern3LedRange(led, led, fromBrightness);
    }
  }

  for (uint8_t i = 0; i < toCount; i++) {
    uint8_t led = toLeds[i];
    if (!pattern10ListContains(fromLeds, fromCount, led)) {
      renderPattern3LedRange(led, led, toBrightness);
    }
  }
}

static void renderPattern10CrossfadeKey4(const uint8_t fromKey[4],
                                         const uint8_t toKey[4],
                                         uint8_t blend,
                                         uint8_t brightness) {
  renderPattern10CrossfadeList(fromKey, 4, toKey, 4, blend, brightness);
}

static uint8_t pattern10ScaledKeyPosition(uint8_t step, uint8_t maxSteps,
                                          uint8_t keyCount,
                                          uint8_t *blendOut) {
  if (keyCount <= 1) {
    *blendOut = 255;
    return 0;
  }

  uint8_t clampedStep = step >= maxSteps ? static_cast<uint8_t>(maxSteps - 1)
                                         : step;
  uint16_t totalTransitions = static_cast<uint16_t>(keyCount - 1);
  uint16_t denominator = maxSteps > 1 ? static_cast<uint16_t>(maxSteps - 1) : 1;
  uint16_t scaled = static_cast<uint16_t>(clampedStep) * totalTransitions * 255U;
  uint16_t position = scaled / denominator;
  uint8_t keyIndex = static_cast<uint8_t>(position / 255U);

  if (keyIndex >= keyCount - 1) {
    keyIndex = static_cast<uint8_t>(keyCount - 2);
    *blendOut = 255;
  } else {
    *blendOut = static_cast<uint8_t>(position % 255U);
  }

  return keyIndex;
}

static void renderPattern10TopTransferWindow(uint8_t targetIndex,
                                             uint8_t step, bool outbound,
                                             uint8_t brightness) {
  // The start window moves forward after every completed lock. This prevents
  // the animation from repeatedly pulling from the same top LEDs and makes the
  // top row look like it is being consumed in pairs.
  static const uint8_t T0_TOP[][4] = {
      {75, 74, 73, 72}, {74, 73, 72, 71}, {73, 72, 71, 70},
      {72, 71, 70, 69}, {71, 70, 69, 68}};
  static const uint8_t T0_LOWER[][4] = {
      {59, 60, 61, 62}, {60, 61, 62, 63}, {61, 62, 63, 64},
      {62, 63, 64, 65}, {64, 65, 66, 67}};

  static const uint8_t T1_TOP[][4] = {
      {73, 72, 71, 70}, {72, 71, 70, 69}, {71, 70, 69, 68}};
  static const uint8_t T1_LOWER[][4] = {
      {61, 62, 63, 64}, {62, 63, 64, 65}, {64, 65, 66, 67}};

  static const uint8_t T2_TOP[][4] = {
      {71, 70, 69, 68}, {71, 70, 69, 68}};
  static const uint8_t T2_LOWER[][4] = {
      {63, 64, 65, 66}, {64, 65, 66, 67}};

  static const uint8_t T3_TOP[][4] = {
      {71, 70, 69, 68}, {71, 70, 69, 68}};
  static const uint8_t T3_LOWER[][4] = {
      {64, 65, 66, 67}, {64, 65, 66, 67}};

  const uint8_t (*topKeys)[4] = T0_TOP;
  const uint8_t (*lowerKeys)[4] = T0_LOWER;
  uint8_t keyCount = 5;

  if (targetIndex == 1) {
    topKeys = T1_TOP;
    lowerKeys = T1_LOWER;
    keyCount = 3;
  } else if (targetIndex == 2) {
    topKeys = T2_TOP;
    lowerKeys = T2_LOWER;
    keyCount = 2;
  } else if (targetIndex >= 3) {
    topKeys = T3_TOP;
    lowerKeys = T3_LOWER;
    keyCount = 2;
  }

  static const uint8_t TOP_TRANSFER_STEPS = 12;
  uint8_t directedStep = step;
  if (!outbound) {
    directedStep = step >= TOP_TRANSFER_STEPS
                       ? 0
                       : static_cast<uint8_t>((TOP_TRANSFER_STEPS - 1) - step);
  }

  uint8_t blend = 0;
  uint8_t keyIndex = pattern10ScaledKeyPosition(directedStep,
                                                TOP_TRANSFER_STEPS,
                                                keyCount, &blend);

  renderPattern3LedRange(75, 68, 0);
  renderPattern3LedRange(59, 67, 0);
  renderPattern10CrossfadeKey4(topKeys[keyIndex], topKeys[keyIndex + 1],
                               blend, brightness);
  renderPattern10CrossfadeKey4(lowerKeys[keyIndex], lowerKeys[keyIndex + 1],
                               blend, brightness);
}


static void renderPattern10TopMiddleBridge(uint8_t step, uint8_t maxSteps,
                                           bool down, uint8_t brightness) {
  uint8_t clampedStep = step >= maxSteps ? static_cast<uint8_t>(maxSteps - 1) : step;
  uint8_t directedStep = down
                             ? clampedStep
                             : static_cast<uint8_t>((maxSteps - 1) - clampedStep);

  // Use the same staggered lane-crossfade motion as Animation 3. This avoids
  // the previous row-to-row hard jump and makes the top LEDs look like they
  // are flowing down into the middle pair.
  renderPattern3LedRange(71, 68, 0);
  renderPattern3LedRange(64, 67, 0);
  renderPattern3LedRange(43, 36, 0);
  renderPattern3TopToMiddleWindow(directedStep, brightness);
  renderPattern3LedRange(44, 44, 0);
}


static void renderPattern10MiddleBottomBridge(uint8_t targetIndex,
                                              uint8_t step,
                                              uint8_t maxSteps,
                                              bool down,
                                              uint8_t brightness) {
  (void)targetIndex;
  uint8_t clampedStep = step >= maxSteps ? static_cast<uint8_t>(maxSteps - 1) : step;
  uint8_t directedStep = down
                             ? clampedStep
                             : static_cast<uint8_t>((maxSteps - 1) - clampedStep);

  // Match Animation 3's middle-to-bottom drop: each lane fades out of the
  // middle and fades into the bottom with a small lane delay, so the motion is
  // perceived as a fluent downward slide instead of a stepped block change.
  renderPattern3LedRange(43, 36, 0);
  renderPattern3LedRange(15, 8, 0);
  renderPattern3MiddleToBottomWindow(directedStep, brightness);
  renderPattern3LedRange(44, 44, 0);
}


static void renderPattern10MiddleCascadeDown(uint8_t step, uint8_t maxSteps,
                                             uint8_t brightness) {
  static const uint8_t KEY_COUNT = 5;
  static const uint8_t MIDDLE_KEYS[KEY_COUNT][4] = {
      {40, 41, 42, 43},
      {41, 42, 43, 39},
      {42, 43, 39, 38},
      {43, 39, 38, 37},
      {39, 38, 37, 36},
  };

  uint8_t blend = 0;
  uint8_t keyIndex = pattern10ScaledKeyPosition(step, maxSteps, KEY_COUNT,
                                                &blend);

  renderPattern3LedRange(44, 36, 0);
  renderPattern10CrossfadeKey4(MIDDLE_KEYS[keyIndex],
                               MIDDLE_KEYS[keyIndex + 1], blend,
                               brightness);
  renderPattern3LedRange(44, 44, 0);
}

static void renderPattern10MiddleCascadeUp(uint8_t step, uint8_t maxSteps,
                                           uint8_t brightness) {
  uint8_t rev = maxSteps == 0 ? 0
                              : static_cast<uint8_t>((maxSteps - 1) -
                                                     (step >= maxSteps ? (maxSteps - 1) : step));
  renderPattern10MiddleCascadeDown(rev, maxSteps, brightness);
}

static void renderPattern10BottomCascadeDown(uint8_t targetIndex,
                                             uint8_t step,
                                             uint8_t maxSteps,
                                             uint8_t brightness) {
  static const uint8_t KEY_COUNT_TO_8_11 = 5;
  static const uint8_t BOTTOM_KEYS_TO_8_11[KEY_COUNT_TO_8_11][4] = {
      {12, 13, 14, 15},
      {13, 14, 15, 11},
      {14, 15, 11, 10},
      {15, 11, 10, 9},
      {11, 10, 9, 8},
  };
  static const uint8_t BOTTOM_KEY_12_15[4] = {12, 13, 14, 15};

  renderPattern3LedRange(15, 8, 0);

  if (targetIndex == 1) {
    renderPattern10LedList(BOTTOM_KEY_12_15, 4, brightness);
    return;
  }

  uint8_t blend = 0;
  uint8_t keyIndex = pattern10ScaledKeyPosition(step, maxSteps,
                                                KEY_COUNT_TO_8_11, &blend);
  renderPattern10CrossfadeKey4(BOTTOM_KEYS_TO_8_11[keyIndex],
                               BOTTOM_KEYS_TO_8_11[keyIndex + 1], blend,
                               brightness);
}

static void renderPattern10BottomCascadeUp(uint8_t targetIndex,
                                           uint8_t step,
                                           uint8_t maxSteps,
                                           uint8_t brightness) {
  uint8_t rev = maxSteps == 0 ? 0
                              : static_cast<uint8_t>((maxSteps - 1) -
                                                     (step >= maxSteps ? (maxSteps - 1) : step));
  renderPattern10BottomCascadeDown(targetIndex, rev, maxSteps, brightness);
}

static void renderPattern10Shuttle(uint8_t targetIndex, bool outbound,
                                   uint8_t step, uint8_t brightness) {
  // Match Animation 3 motion cadence: 12 top steps, 12 top-to-middle
  // crossfade steps, and 12 middle-to-bottom crossfade steps. The older
  // animation-10-only middle/bottom cascades made the drop feel different
  // and more stepped, so they are disabled here.
  static const uint8_t TOP_SWEEP_STEPS = 12;
  static const uint8_t TOP_MIDDLE_BRIDGE_STEPS = 12;
  static const uint8_t MIDDLE_CASCADE_STEPS = 0;
  static const uint8_t MIDDLE_BOTTOM_BRIDGE_STEPS = 12;
  static const uint8_t BOTTOM_CASCADE_STEPS = 0;
  static const uint8_t TOP_TO_MIDDLE_STEPS =
      TOP_MIDDLE_BRIDGE_STEPS + MIDDLE_CASCADE_STEPS;
  static const uint8_t MIDDLE_TO_BOTTOM_STEPS =
      MIDDLE_BOTTOM_BRIDGE_STEPS + BOTTOM_CASCADE_STEPS;
  static const uint8_t HOLD_STEPS = 1;

  if (outbound) {
    if (step < TOP_SWEEP_STEPS) {
      renderPattern10TopTransferWindow(targetIndex, step, true, brightness);
    } else if (step < TOP_SWEEP_STEPS + TOP_TO_MIDDLE_STEPS) {
      uint8_t middleStep = static_cast<uint8_t>(step - TOP_SWEEP_STEPS);
      if (middleStep < TOP_MIDDLE_BRIDGE_STEPS) {
        renderPattern10TopMiddleBridge(middleStep, TOP_MIDDLE_BRIDGE_STEPS,
                                       true, brightness);
      } else {
        renderPattern10MiddleCascadeDown(
            static_cast<uint8_t>(middleStep - TOP_MIDDLE_BRIDGE_STEPS),
            MIDDLE_CASCADE_STEPS, brightness);
      }
    } else if (step < TOP_SWEEP_STEPS + TOP_TO_MIDDLE_STEPS +
                          MIDDLE_TO_BOTTOM_STEPS) {
      if (targetIndex < 2) {
        uint8_t bottomStep = static_cast<uint8_t>(
            step - TOP_SWEEP_STEPS - TOP_TO_MIDDLE_STEPS);
        if (bottomStep < MIDDLE_BOTTOM_BRIDGE_STEPS) {
          renderPattern10MiddleBottomBridge(targetIndex, bottomStep,
                                            MIDDLE_BOTTOM_BRIDGE_STEPS,
                                            true, brightness);
        } else {
          renderPattern10BottomCascadeDown(
              targetIndex,
              static_cast<uint8_t>(bottomStep - MIDDLE_BOTTOM_BRIDGE_STEPS),
              BOTTOM_CASCADE_STEPS, brightness);
        }
      } else {
        renderPattern10TargetRange(targetIndex, brightness);
      }
    } else {
      renderPattern10TargetRange(targetIndex, brightness);
    }
    return;
  }

  uint8_t returnStep = step;
  if (returnStep < HOLD_STEPS) {
    renderPattern10TargetRange(targetIndex, brightness);
  } else if (returnStep < HOLD_STEPS + MIDDLE_TO_BOTTOM_STEPS) {
    uint8_t bottomStep = static_cast<uint8_t>(returnStep - HOLD_STEPS);
    if (targetIndex < 2) {
      if (bottomStep < BOTTOM_CASCADE_STEPS) {
        renderPattern10BottomCascadeUp(targetIndex, bottomStep,
                                       BOTTOM_CASCADE_STEPS, brightness);
      } else {
        renderPattern10MiddleBottomBridge(
            targetIndex,
            static_cast<uint8_t>(bottomStep - BOTTOM_CASCADE_STEPS),
            MIDDLE_BOTTOM_BRIDGE_STEPS, false, brightness);
      }
    } else {
      renderPattern10TargetRange(targetIndex, brightness);
    }
  } else if (returnStep <
             HOLD_STEPS + MIDDLE_TO_BOTTOM_STEPS + TOP_TO_MIDDLE_STEPS) {
    uint8_t middleStep = static_cast<uint8_t>(returnStep - HOLD_STEPS -
                                              MIDDLE_TO_BOTTOM_STEPS);
    if (middleStep < MIDDLE_CASCADE_STEPS) {
      renderPattern10MiddleCascadeUp(middleStep, MIDDLE_CASCADE_STEPS,
                                     brightness);
    } else {
      renderPattern10TopMiddleBridge(
          static_cast<uint8_t>(middleStep - MIDDLE_CASCADE_STEPS),
          TOP_MIDDLE_BRIDGE_STEPS, false, brightness);
    }
  } else {
    uint8_t topOffset = static_cast<uint8_t>(
        returnStep - HOLD_STEPS - MIDDLE_TO_BOTTOM_STEPS - TOP_TO_MIDDLE_STEPS);
    renderPattern10TopTransferWindow(targetIndex, topOffset, false, brightness);
  }
}

static void renderPattern10FullLitSet(uint8_t brightness) {
  renderPattern10TopFull(brightness);

  // Right-side middle entry block used by the original sweep path.
  renderPattern3LedRange(39, 36, brightness);
  renderPattern3LedRange(40, 43, brightness);

  // Bottom two rows, excluding LEDs 7, 16, and 17.
  renderPattern3LedRange(1, 6, brightness);
  renderPattern3LedRange(8, 15, brightness);
  renderPattern3LedRange(18, 24, brightness);

  // Added opposite middle-row endpoint. Keep it as a two-row block so it
  // belongs to the full Animation 10 shape, not only the final sweep.
  renderPattern10OppositeMiddleBlock(brightness);
}

static bool isPattern10ExcludedLed(uint8_t led) {
  return led == 7 || led == 16 || led == 17;
}

static void renderPattern10SingleLed(uint8_t led, uint8_t brightness) {
  if (isPattern10ExcludedLed(led)) {
    brightness = 0;
  }
  renderPattern3LedRange(led, led, brightness);
}

static void renderPattern10SweepLedGroup(const uint8_t *leds,
                                        uint8_t count,
                                        uint8_t brightness) {
  for (uint8_t i = 0; i < count; i++) {
    renderPattern10SingleLed(leds[i], brightness);
  }
}

static void renderPattern10SweepGroup(const uint8_t group[3],
                                      uint8_t brightness) {
  renderPattern10SingleLed(group[0], brightness);
  if (group[2] > 1) {
    renderPattern10SingleLed(group[1], brightness);
  }
}

static void renderPattern10Slot(uint8_t slot, uint8_t brightness) {
  if (slot >= PATTERN3_SLOT_COUNT) {
    return;
  }

  int8_t stepA = PATTERN3_SLOT_A_START[slot] <= PATTERN3_SLOT_A_END[slot] ? 1 : -1;
  for (uint8_t led = PATTERN3_SLOT_A_START[slot];; led += stepA) {
    renderPattern10SingleLed(led, brightness);
    if (led == PATTERN3_SLOT_A_END[slot]) {
      break;
    }
  }

  int8_t stepB = PATTERN3_SLOT_B_START[slot] <= PATTERN3_SLOT_B_END[slot] ? 1 : -1;
  for (uint8_t led = PATTERN3_SLOT_B_START[slot];; led += stepB) {
    renderPattern10SingleLed(led, brightness);
    if (led == PATTERN3_SLOT_B_END[slot]) {
      break;
    }
  }
}

static void renderPattern10SweepPhase(bool forwardSweep, uint8_t sweepStep,
                                      uint8_t frameInStep, uint8_t stepFrames,
                                      uint8_t fullBrightness) {
  static const uint8_t FORWARD_ORDER[PATTERN3_SLOT_COUNT] = {7, 6, 5, 4,
                                                             3, 2, 1, 0};
  static const uint8_t BACKWARD_ORDER[PATTERN3_SLOT_COUNT] = {0, 1, 2, 3,
                                                              4, 5, 6, 7};
  const uint8_t *order = forwardSweep ? FORWARD_ORDER : BACKWARD_ORDER;
  uint8_t transitioningBrightness = fullBrightness;

  if (stepFrames > 1) {
    uint8_t phase = static_cast<uint8_t>(
        (static_cast<uint16_t>(frameInStep) * 255) / (stepFrames - 1));
    transitioningBrightness = forwardSweep
                                  ? pattern3ScaleBrightness(
                                        fullBrightness,
                                        static_cast<uint8_t>(255 - phase))
                                  : pattern3ScaleBrightness(fullBrightness, phase);
  }

  for (uint8_t i = 0; i < PATTERN3_SLOT_COUNT; i++) {
    uint8_t slotId = order[i];
    if (forwardSweep) {
      if (i < sweepStep) {
        continue;
      }
      if (i == sweepStep) {
        renderPattern10Slot(slotId, transitioningBrightness);
      } else {
        renderPattern10Slot(slotId, fullBrightness);
      }
    } else {
      if (i < sweepStep) {
        renderPattern10Slot(slotId, fullBrightness);
      } else if (i == sweepStep) {
        renderPattern10Slot(slotId, transitioningBrightness);
      }
    }
  }

  renderPattern3LedRange(44, 44, 0);
}

static void patternReturnFlowStack(uint32_t frame) {
  // Animation 10 now uses the same speed/smoothness basis as Animation 3:
  // 2x frame scaling, 12-step crossfade drops, and the same lead/trail
  // brightness levels around the active moving block.
  static const uint8_t SPEED_MULTIPLIER = 2;
  static const uint8_t TARGET_COUNT = 4;
  static const uint8_t TOP_SWEEP_STEPS = 12;
  static const uint8_t TOP_TO_MIDDLE_STEPS = 12;
  static const uint8_t MIDDLE_TO_BOTTOM_STEPS = 12;
  static const uint8_t HOLD_STEPS = 1;
  static const uint8_t ACTIVE_BRIGHTNESS = 255;
  static const uint8_t TRAIL_BRIGHTNESS = 122;
  static const uint8_t TRAIL2_BRIGHTNESS = 62;
  static const uint8_t LEAD_BRIGHTNESS = 36;
  static const uint8_t LOCKED_BRIGHTNESS = 165;
  static const uint8_t SOURCE_DIM_BRIGHTNESS = 1;
  static const uint8_t TOP_FILL_HOLD_FRAMES = 2;
  static const uint8_t SWEEP_CHUNK_FRAMES = PATTERN3_SWEEP_STEP_FRAMES;
  static const uint8_t SWEEP_PATH_STEPS = PATTERN3_SLOT_COUNT;

  const uint16_t outboundFrames = static_cast<uint16_t>(
      TOP_SWEEP_STEPS + TOP_TO_MIDDLE_STEPS + MIDDLE_TO_BOTTOM_STEPS +
      HOLD_STEPS);
  const uint16_t inboundFrames = outboundFrames;
  const uint16_t targetCycleFrames = outboundFrames + inboundFrames;
  const uint16_t lockPhaseFrames =
      static_cast<uint16_t>(TARGET_COUNT) * targetCycleFrames;
  const uint16_t sweepFrames =
      static_cast<uint16_t>(SWEEP_CHUNK_FRAMES) * SWEEP_PATH_STEPS;
  const uint16_t totalCycleFrames =
      static_cast<uint16_t>(lockPhaseFrames + TOP_FILL_HOLD_FRAMES +
                            sweepFrames + sweepFrames);

  uint32_t scaledFrame = static_cast<uint32_t>(frame) * SPEED_MULTIPLIER;
  uint16_t cycleFrame = static_cast<uint16_t>(scaledFrame % totalCycleFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < lockPhaseFrames) {
    uint8_t targetIndex = static_cast<uint8_t>(cycleFrame / targetCycleFrames);
    uint16_t localCycleFrame =
        static_cast<uint16_t>(cycleFrame % targetCycleFrames);
    bool outbound = localCycleFrame < outboundFrames;
    uint8_t shuttleStep =
        outbound ? static_cast<uint8_t>(localCycleFrame)
                 : static_cast<uint8_t>(localCycleFrame - outboundFrames);
    uint8_t lockedCount = static_cast<uint8_t>(
        targetIndex + (outbound ? 0 : 1));

    renderPattern10BaseBandsWithSourceLevel(LOCKED_BRIGHTNESS,
                                            SOURCE_DIM_BRIGHTNESS);
    renderPattern10ProgressiveTopSource(lockedCount, LOCKED_BRIGHTNESS);

    // Same visual smoothing style as Animation 3: a dim trail behind the
    // moving block and a faint lead frame ahead of it. This removes the
    // harsher single-keyframe look from Animation 10.
    if (shuttleStep > 0) {
      renderPattern10Shuttle(targetIndex, outbound,
                             static_cast<uint8_t>(shuttleStep - 1),
                             TRAIL_BRIGHTNESS);
    }
    if (shuttleStep > 1) {
      renderPattern10Shuttle(targetIndex, outbound,
                             static_cast<uint8_t>(shuttleStep - 2),
                             TRAIL2_BRIGHTNESS);
    }
    if (shuttleStep + 1 < outboundFrames) {
      renderPattern10Shuttle(targetIndex, outbound,
                             static_cast<uint8_t>(shuttleStep + 1),
                             LEAD_BRIGHTNESS);
    }
    renderPattern10Shuttle(targetIndex, outbound, shuttleStep, ACTIVE_BRIGHTNESS);
    renderPattern10LockedTargets(lockedCount, LOCKED_BRIGHTNESS);
    // Keep the newly added middle endpoint visible during the full Animation 10
    // cycle. It should not appear only when the final sweep starts.
    renderPattern10OppositeMiddleBlock(LOCKED_BRIGHTNESS);
    renderPattern3LedRange(44, 44, 0);
    return;
  }

  uint16_t postLockFrame = static_cast<uint16_t>(cycleFrame - lockPhaseFrames);
  if (postLockFrame < TOP_FILL_HOLD_FRAMES) {
    renderPattern10FullLitSet(LOCKED_BRIGHTNESS);
    return;
  }

  uint16_t sweepFrame = static_cast<uint16_t>(postLockFrame - TOP_FILL_HOLD_FRAMES);
  bool forwardSweepOff = sweepFrame < sweepFrames;
  uint16_t localSweepFrame =
      forwardSweepOff ? sweepFrame : static_cast<uint16_t>(sweepFrame - sweepFrames);
  uint8_t sweepStep = static_cast<uint8_t>(localSweepFrame / SWEEP_CHUNK_FRAMES);
  if (sweepStep >= SWEEP_PATH_STEPS) {
    sweepStep = static_cast<uint8_t>(SWEEP_PATH_STEPS - 1);
  }
  uint8_t frameInStep =
      static_cast<uint8_t>(localSweepFrame % SWEEP_CHUNK_FRAMES);
  renderPattern10SweepPhase(forwardSweepOff, sweepStep, frameInStep,
                            SWEEP_CHUNK_FRAMES, LOCKED_BRIGHTNESS);
}


static void renderPattern11CornerBlock(uint8_t brightness) {
  // Top-right corner block reached after the initial top sweep.
  renderPattern3LedRange(71, 68, brightness);
  renderPattern3LedRange(64, 67, brightness);
}

static void renderPattern11MiddleBlock(uint8_t brightness) {
  // Original middle crossing plus the added opposite-end middle pair.
  renderOppositeMiddleBlock(brightness);
  renderPattern3LedRange(43, 40, brightness);
  renderPattern3LedRange(36, 39, brightness);
}

static void renderPattern11BottomEntryBlock(uint8_t brightness) {
  // First bottom block reached by the downward motion.
  renderPattern3LedRange(15, 12, brightness);
  renderPattern3LedRange(11, 8, brightness);
}

static void renderPattern11FullShape(uint8_t brightness) {
  renderPattern3LedRange(78, 68, brightness);
  renderPattern3LedRange(55, 67, brightness);
  renderPattern11MiddleBlock(brightness);
  renderPattern3LedRange(12, 24, brightness);
  renderPattern3LedRange(1, 11, brightness);
}

static void renderPattern11TopMover(uint8_t step, uint8_t maxStep,
                                    uint8_t brightness) {
  if (maxStep == 0) {
    maxStep = 1;
  }

  uint8_t clampedStep = step > maxStep ? maxStep : step;

  // Use the same 4-LED moving top window style as Animation 3. A dim
  // previous window and a faint next window give Animation 11 the same
  // fluent sweep instead of a separate single-pixel chase look.
  if (clampedStep > 0) {
    renderPattern3TopWindow(static_cast<uint8_t>(clampedStep - 1),
                            pattern3ScaleBrightness(brightness, 122));
  }
  if (clampedStep + 1 <= maxStep) {
    renderPattern3TopWindow(static_cast<uint8_t>(clampedStep + 1),
                            pattern3ScaleBrightness(brightness, 36));
  }
  renderPattern3TopWindow(clampedStep, brightness);
}

static uint8_t pattern11EaseFrame(uint8_t frame, uint8_t totalFrames) {
  if (totalFrames <= 1) {
    return 255;
  }
  uint8_t clamped = frame >= totalFrames ? static_cast<uint8_t>(totalFrames - 1) : frame;
  uint8_t raw = static_cast<uint8_t>((static_cast<uint16_t>(clamped) * 255) /
                                     (totalFrames - 1));
  return pattern3EaseInOut(raw);
}

static uint8_t pattern11RevealCount(uint8_t frame, uint8_t totalFrames,
                                    uint8_t count) {
  if (totalFrames == 0) {
    return count;
  }

  uint16_t scaled = static_cast<uint16_t>(frame + 1) * count;
  uint8_t value = static_cast<uint8_t>(scaled / totalFrames);
  return value > count ? count : value;
}

static void renderPattern11RevealList(const uint8_t *leds, uint8_t count,
                                      uint8_t revealCount,
                                      uint8_t brightness) {
  if (revealCount > count) {
    revealCount = count;
  }

  for (uint8_t i = 0; i < revealCount; i++) {
    renderPattern3SingleLed(leds[i], brightness);
  }
}

static void renderPattern11CrossfadeLanes(const uint8_t *sourceLeds,
                                          const uint8_t *targetLeds,
                                          uint8_t laneCount, uint8_t frame,
                                          uint8_t totalFrames,
                                          uint8_t brightness) {
  uint8_t eased = pattern11EaseFrame(frame, totalFrames);
  uint8_t sourceBrightness =
      pattern3ScaleBrightness(brightness, static_cast<uint8_t>(255 - eased));
  uint8_t targetBrightness = pattern3ScaleBrightness(brightness, eased);

  for (uint8_t lane = 0; lane < laneCount; lane++) {
    renderPattern3SingleLed(sourceLeds[lane], sourceBrightness);
    renderPattern3SingleLed(targetLeds[lane], targetBrightness);
  }
}

static void renderPattern11TopReturnWindow(uint8_t windowStep,
                                           uint8_t brightness) {
  // Sliding window moves backward from the top-right corner toward the
  // original top starting side.
  static const uint8_t TOP_MAX_STEP = 9;
  uint8_t step = windowStep > TOP_MAX_STEP ? TOP_MAX_STEP : windowStep;

  uint8_t topOffset = static_cast<uint8_t>((static_cast<uint16_t>(step) * 7 + 4) / 9);
  uint8_t topStart = static_cast<uint8_t>(68 + topOffset);
  renderPattern3LedRange(static_cast<uint8_t>(topStart + 3), topStart,
                         brightness);

  uint8_t lowerStart = static_cast<uint8_t>(64 - step);
  renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart + 3),
                         brightness);
}

static void renderPattern11MiddleReturnWindow(uint8_t windowStep,
                                              uint8_t brightness) {
  static const uint8_t MIDDLE_MAX_STEP = 11;
  uint8_t step = windowStep > MIDDLE_MAX_STEP ? MIDDLE_MAX_STEP : windowStep;

  uint8_t upperStart = static_cast<uint8_t>(40 + step);
  renderPattern3LedRange(upperStart, static_cast<uint8_t>(upperStart + 3),
                         brightness);

  uint8_t lowerStart = static_cast<uint8_t>(39 - step);
  renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart - 3),
                         brightness);
}

static void renderPattern11BottomReturnWindow(uint8_t windowStep,
                                               uint8_t brightness) {
  // Bottom sliding window moves toward the original bottom LEDs 24 and 1.
  static const uint8_t BOTTOM_MAX_STEP = 9;
  uint8_t step = windowStep > BOTTOM_MAX_STEP ? BOTTOM_MAX_STEP : windowStep;

  uint8_t upperStart = static_cast<uint8_t>(12 + step);
  renderPattern3LedRange(upperStart, static_cast<uint8_t>(upperStart + 3),
                         brightness);

  uint8_t lowerStart = static_cast<uint8_t>(8 - (step > 7 ? 7 : step));
  renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart + 3),
                          brightness);
}

static void renderPattern11VerticalTransfer(uint8_t frame,
                                            uint8_t totalFrames,
                                            uint8_t brightness) {
  if (totalFrames <= 1) {
    renderPattern3TopToMiddleWindow(0, brightness);
    renderPattern3MiddleToBottomWindow(0, brightness);
    return;
  }

  uint8_t clamped = frame >= totalFrames ? static_cast<uint8_t>(totalFrames - 1) : frame;
  uint16_t scaled = static_cast<uint16_t>(clamped) * 11;
  uint8_t step = static_cast<uint8_t>((scaled + ((totalFrames - 1) / 2)) /
                                      (totalFrames - 1));
  if (step > 11) {
    step = 11;
  }

  renderPattern3TopToMiddleWindow(step, brightness);
  renderPattern3MiddleToBottomWindow(step, brightness);
}

static void renderPattern11DownPathWindow(uint8_t pathStep,
                                           uint8_t brightness) {
  // One active block drops from the top into the middle, moves left across the
  // full middle pair, drops into the bottom, then moves left across the bottom.
  if (pathStep < 12) {
    renderPattern3TopToMiddleWindow(pathStep, brightness);
    return;
  }

  if (pathStep < 24) {
    renderPattern11MiddleReturnWindow(static_cast<uint8_t>(pathStep - 12),
                                      brightness);
    return;
  }

  if (pathStep < 36) {
    renderPattern3MiddleToBottomWindow(static_cast<uint8_t>(pathStep - 24),
                                       brightness);
    return;
  }

  renderPattern11BottomReturnWindow(static_cast<uint8_t>(pathStep - 36),
                                    brightness);
}

static uint8_t pattern11BlendStep(uint8_t frame, uint8_t totalFrames,
                                  uint8_t stepCount, uint8_t &blend) {
  if (stepCount <= 1 || totalFrames <= 1) {
    blend = 0;
    return 0;
  }

  uint32_t progress = static_cast<uint32_t>(frame) * (stepCount - 1) * 255;
  progress = progress / (totalFrames - 1);
  uint8_t step = static_cast<uint8_t>(progress / 255);
  blend = static_cast<uint8_t>(progress % 255);
  if (step >= stepCount - 1) {
    step = static_cast<uint8_t>(stepCount - 1);
    blend = 0;
  }
  return step;
}

static void renderPattern11SynchronizedLeftReturn(uint8_t frame,
                                                  uint8_t totalFrames,
                                                  uint8_t brightness) {
  static const uint8_t RETURN_STEPS = 12;
  uint8_t blend = 0;
  uint8_t step = pattern11BlendStep(frame, totalFrames, RETURN_STEPS, blend);

  uint8_t currentBrightness = pattern3ScaleBrightness(
      brightness, static_cast<uint8_t>(255 - blend));
  uint8_t nextBrightness = pattern3ScaleBrightness(brightness, blend);

  if (step + 1 < RETURN_STEPS) {
    if (currentBrightness < nextBrightness) {
      renderPattern11TopReturnWindow(step, currentBrightness);
      renderPattern11MiddleReturnWindow(step, currentBrightness);
      renderPattern11BottomReturnWindow(step, currentBrightness);
      renderPattern11TopReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11MiddleReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11BottomReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
    } else {
      renderPattern11TopReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11MiddleReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11BottomReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11TopReturnWindow(step, currentBrightness);
      renderPattern11MiddleReturnWindow(step, currentBrightness);
      renderPattern11BottomReturnWindow(step, currentBrightness);
    }
  } else {
    renderPattern11TopReturnWindow(step, brightness);
    renderPattern11MiddleReturnWindow(step, brightness);
    renderPattern11BottomReturnWindow(step, brightness);
  }
}

static void renderPattern11InterpolatedTopReturn(uint8_t frame,
                                                  uint8_t totalFrames,
                                                  uint8_t brightness) {
  static const uint8_t WINDOW_STEPS = 10;
  uint8_t blend = 0;
  uint8_t step = pattern11BlendStep(frame, totalFrames, WINDOW_STEPS, blend);

  uint8_t currentBrightness = pattern3ScaleBrightness(
      brightness, static_cast<uint8_t>(255 - blend));
  uint8_t nextBrightness = pattern3ScaleBrightness(brightness, blend);

  if (step + 1 < WINDOW_STEPS) {
    if (currentBrightness < nextBrightness) {
      renderPattern11TopReturnWindow(step, currentBrightness);
      renderPattern11TopReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
    } else {
      renderPattern11TopReturnWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11TopReturnWindow(step, currentBrightness);
    }
  } else {
    renderPattern11TopReturnWindow(step, brightness);
  }
}

static void renderPattern11InterpolatedDownPath(uint8_t frame,
                                                uint8_t totalFrames,
                                                uint8_t brightness) {
  static const uint8_t PATH_STEPS = 46;
  uint8_t blend = 0;
  uint8_t step = pattern11BlendStep(frame, totalFrames, PATH_STEPS, blend);

  uint8_t currentBrightness = pattern3ScaleBrightness(
      brightness, static_cast<uint8_t>(255 - blend));
  uint8_t nextBrightness = pattern3ScaleBrightness(brightness, blend);

  if (step + 1 < PATH_STEPS) {
    if (currentBrightness < nextBrightness) {
      renderPattern11DownPathWindow(step, currentBrightness);
      renderPattern11DownPathWindow(static_cast<uint8_t>(step + 1), nextBrightness);
    } else {
      renderPattern11DownPathWindow(static_cast<uint8_t>(step + 1), nextBrightness);
      renderPattern11DownPathWindow(step, currentBrightness);
    }
  } else {
    renderPattern11DownPathWindow(step, brightness);
  }
}

static void renderPattern11FinalFade(uint8_t frame, uint8_t totalFrames,
                                     uint8_t brightness) {
  if (totalFrames <= 1) {
    return;
  }
  uint8_t clamped = frame >= totalFrames ? static_cast<uint8_t>(totalFrames - 1) : frame;
  uint8_t scale = static_cast<uint8_t>(255 - ((static_cast<uint16_t>(clamped) * 255) /
                                              (totalFrames - 1)));
  uint8_t faded = pattern3ScaleBrightness(brightness, scale);
  renderPattern11TopReturnWindow(11, faded);
  renderPattern11MiddleReturnWindow(11, faded);
  renderPattern11BottomReturnWindow(11, faded);
}

static void patternTopFlashCascade(uint32_t frame) {
  // Animation 11: top window moves from the starting side to the top-right
  // corner, drops into the middle and bottom rows, then all three row pairs
  // slide left together from the same progress step.
  static const uint8_t SPEED_MULTIPLIER = 1;
  static const uint8_t TRAVEL_FRAMES = 15;
  static const uint8_t CORNER_HOLD_FRAMES = 3;
  static const uint8_t TRANSITION_FRAMES = 12;
  static const uint8_t SYNC_FLOW_FRAMES = 24;
  static const uint8_t SPLIT_FLOW_FRAMES = TRANSITION_FRAMES + SYNC_FLOW_FRAMES;
  static const uint8_t END_FADE_FRAMES = 6;
  static const uint8_t BLANK_FRAMES = 2;
  static const uint8_t ACTIVE_BRIGHTNESS = 255;

  static const uint16_t TOTAL_FRAMES =
      TRAVEL_FRAMES + CORNER_HOLD_FRAMES + SPLIT_FLOW_FRAMES +
      END_FADE_FRAMES + BLANK_FRAMES;

  uint32_t scaledFrame = static_cast<uint32_t>(frame) * SPEED_MULTIPLIER;
  uint16_t cycleFrame = static_cast<uint16_t>(scaledFrame % TOTAL_FRAMES);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < TRAVEL_FRAMES) {
    renderPattern11TopMover(static_cast<uint8_t>(cycleFrame),
                            static_cast<uint8_t>(TRAVEL_FRAMES - 1),
                            ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - TRAVEL_FRAMES);
  if (cycleFrame < CORNER_HOLD_FRAMES) {
    // Small readable corner flash before the split motion starts.
    // This makes the arrival at 71-68 / 64-67 visible without adding
    // the old end-marker behavior.
    uint8_t flashBrightness = (cycleFrame == 1) ? 120 : ACTIVE_BRIGHTNESS;
    renderPattern11CornerBlock(flashBrightness);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - CORNER_HOLD_FRAMES);
  if (cycleFrame < SPLIT_FLOW_FRAMES) {
    if (cycleFrame < TRANSITION_FRAMES) {
      renderPattern11VerticalTransfer(static_cast<uint8_t>(cycleFrame),
                                      TRANSITION_FRAMES, ACTIVE_BRIGHTNESS);
    } else {
      renderPattern11SynchronizedLeftReturn(static_cast<uint8_t>(cycleFrame - TRANSITION_FRAMES),
          SYNC_FLOW_FRAMES,
          ACTIVE_BRIGHTNESS);
    }
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - SPLIT_FLOW_FRAMES);
  if (cycleFrame < END_FADE_FRAMES) {
    renderPattern11FinalFade(static_cast<uint8_t>(cycleFrame), END_FADE_FRAMES,
                             ACTIVE_BRIGHTNESS);
    return;
  }
}


static uint8_t pattern12FlickerScale(uint32_t frame, uint8_t ledNumber,
                                   uint8_t strength) {
  // Deterministic pseudo-flicker. It avoids random() so the animation remains
  // repeatable and does not disturb other firmware timing/state behavior.
  uint8_t hash = static_cast<uint8_t>((frame * 37U) + (ledNumber * 17U) +
                                      ((frame >> 1) * 11U));
  uint8_t wobble = static_cast<uint8_t>(hash % (strength + 1));
  return static_cast<uint8_t>(255 - wobble);
}

static void renderPattern12SweepGroup(const uint8_t group[3],
                                      uint8_t brightness, uint8_t scale) {
  uint8_t scaledBrightness = pattern3ScaleBrightness(brightness, scale);
  renderPattern3LedRange(group[0], group[0], scaledBrightness);
  if (group[2] > 1 && group[1] > 0) {
    renderPattern3LedRange(group[1], group[1], scaledBrightness);
  }
}

static uint8_t pattern12SweepGroupCount() {
  // 30 original groups + lower/upper vertical endpoint groups.
  return 32;
}

static void renderPattern12GroupByIndex(uint8_t groupIndex,
                                        uint8_t brightness,
                                        uint8_t scale) {
  // Top-to-bottom path, then a vertical endpoint finish into LEDs 25-29
  // and 54-50 so the wash does not stop at LEDs 1/24.
  static const uint8_t GROUPS[][3] = {
      {78, 55, 2}, {77, 56, 2}, {76, 57, 2}, {75, 58, 2},
      {74, 59, 2}, {73, 60, 2}, {72, 61, 2}, {71, 62, 2},
      {70, 63, 2}, {69, 64, 2}, {68, 65, 2}, {66, 0, 1},
      {67, 0, 1},
      {43, 39, 2}, {42, 38, 2}, {41, 37, 2}, {40, 36, 2},
      {12, 11, 2}, {13, 10, 2}, {14, 9, 2},  {15, 8, 2},
      {16, 7, 2},  {17, 6, 2},  {18, 5, 2},  {19, 4, 2},
      {20, 3, 2},  {21, 2, 2},  {22, 1, 2},  {23, 0, 1},
      {24, 0, 1},
  };

  static const uint8_t BASE_GROUP_COUNT = sizeof(GROUPS) / sizeof(GROUPS[0]);
  uint8_t scaledBrightness = pattern3ScaleBrightness(brightness, scale);
  if (groupIndex < BASE_GROUP_COUNT) {
    renderPattern12SweepGroup(GROUPS[groupIndex], brightness, scale);
  } else if (groupIndex == BASE_GROUP_COUNT) {
    renderOppositeMiddleLowerBlock(scaledBrightness);
  } else if (groupIndex == BASE_GROUP_COUNT + 1) {
    renderOppositeMiddleUpperBlock(scaledBrightness);
  }
}

static void renderPattern12FullShapeSolid(uint8_t brightness) {
  static const uint8_t LEDS[] = {
      78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
      54, 53, 52, 51, 50,
      43, 42, 41, 40, 39, 38, 37, 36,
      25, 26, 27, 28, 29,
      12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
      11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
  };

  for (uint8_t i = 0; i < sizeof(LEDS); i++) {
    renderPattern3LedRange(LEDS[i], LEDS[i], brightness);
  }
}

static void renderPattern12FullShapeFlicker(uint8_t brightness,
                                            uint32_t frame,
                                            uint8_t flickerStrength) {
  static const uint8_t LEDS[] = {
      78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
      54, 53, 52, 51, 50,
      43, 42, 41, 40, 39, 38, 37, 36,
      25, 26, 27, 28, 29,
      12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
      11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
  };

  for (uint8_t i = 0; i < sizeof(LEDS); i++) {
    uint8_t scale = pattern12FlickerScale(frame, LEDS[i], flickerStrength);
    renderPattern3LedRange(LEDS[i], LEDS[i],
                           pattern3ScaleBrightness(brightness, scale));
  }
}

static void renderPattern12BuildPass(uint8_t sweepStep, uint8_t frameInStep,
                                     uint8_t stepFrames, uint8_t brightness) {
  uint8_t groupCount = pattern12SweepGroupCount();
  uint8_t completedGroups = sweepStep >= groupCount
                                ? groupCount
                                : sweepStep;

  // Keep completed LEDs clean and steady. The previous version applied
  // flicker to every already-filled LED, which made the sweep look stuck and
  // noisy instead of a smooth low -> medium -> full brightness wash.
  for (uint8_t i = 0; i < completedGroups; i++) {
    renderPattern12GroupByIndex(i, brightness, 255);
  }

  if (sweepStep >= groupCount) {
    return;
  }

  uint8_t progress = stepFrames <= 1
                         ? 255
                         : static_cast<uint8_t>((static_cast<uint16_t>(frameInStep) * 255) /
                                                (stepFrames - 1));
  uint8_t eased = pattern3EaseInOut(progress);
  uint8_t headScale = eased < 70 ? 70 : eased;
  uint8_t trailScale = pattern3ScaleBrightness(headScale, 130);
  uint8_t leadScale = pattern3ScaleBrightness(static_cast<uint8_t>(255 - eased), 55);

  if (sweepStep > 0) {
    renderPattern12GroupByIndex(static_cast<uint8_t>(sweepStep - 1),
                                brightness, trailScale);
  }

  renderPattern12GroupByIndex(sweepStep, brightness, headScale);

  if (sweepStep + 1 < groupCount) {
    renderPattern12GroupByIndex(static_cast<uint8_t>(sweepStep + 1),
                                brightness, leadScale);
  }
}

static void renderPattern12FadeOff(uint8_t sweepStep, uint8_t frameInStep,
                                   uint8_t stepFrames) {
  static const uint8_t FULL_BRIGHTNESS = 255;
  uint8_t groupCount = pattern12SweepGroupCount();

  renderPattern12FullShapeSolid(FULL_BRIGHTNESS);

  uint8_t completedGroups = sweepStep >= groupCount
                                ? groupCount
                                : sweepStep;
  for (uint8_t i = 0; i < completedGroups; i++) {
    renderPattern12GroupByIndex(i, 0, 255);
  }

  if (sweepStep >= groupCount) {
    return;
  }

  uint8_t progress = stepFrames <= 1
                         ? 255
                         : static_cast<uint8_t>((static_cast<uint16_t>(frameInStep) * 255) /
                                                (stepFrames - 1));
  uint8_t offScale = static_cast<uint8_t>(255 - pattern3EaseInOut(progress));
  renderPattern12GroupByIndex(sweepStep, FULL_BRIGHTNESS, offScale);
}

static void patternSequentialIntensityWash(uint32_t frame) {
  // Animation 12: three top-to-bottom sliding passes through the same active
  // LED path as animations 3/10/11. First pass leaves a dim wash, second pass
  // raises it, third pass reaches full brightness. Then the full shape turns
  // off with a slower top-to-bottom sweep and a small deterministic flicker.
  static const uint8_t BUILD_STEP_FRAMES = 1;
  static const uint8_t OFF_STEP_FRAMES = 2;
  static const uint8_t LOW_BRIGHTNESS = 42;
  static const uint8_t MID_BRIGHTNESS = 125;
  static const uint8_t FULL_BRIGHTNESS = 255;
  static const uint8_t HOLD_FRAMES = 3;
  static const uint8_t BLANK_FRAMES = 2;
  static const uint8_t PASS_COUNT = 3;

  const uint8_t groupCount = pattern12SweepGroupCount();
  const uint16_t framesPerBuildPass = static_cast<uint16_t>(groupCount) * BUILD_STEP_FRAMES;
  const uint16_t buildFrames = static_cast<uint16_t>(PASS_COUNT) * framesPerBuildPass;
  const uint16_t offFrames = static_cast<uint16_t>(groupCount) * OFF_STEP_FRAMES;
  const uint16_t totalFrames = static_cast<uint16_t>(buildFrames + HOLD_FRAMES +
                                                     offFrames + BLANK_FRAMES);

  uint16_t cycleFrame = static_cast<uint16_t>(frame % totalFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < buildFrames) {
    uint8_t pass = static_cast<uint8_t>(cycleFrame / framesPerBuildPass);
    uint16_t passFrame = static_cast<uint16_t>(cycleFrame % framesPerBuildPass);
    uint8_t sweepStep = static_cast<uint8_t>(passFrame / BUILD_STEP_FRAMES);
    uint8_t frameInStep = static_cast<uint8_t>(passFrame % BUILD_STEP_FRAMES);

    if (pass > 0) {
      renderPattern12FullShapeSolid(LOW_BRIGHTNESS);
    }
    if (pass > 1) {
      renderPattern12FullShapeSolid(MID_BRIGHTNESS);
    }

    uint8_t passBrightness = pass == 0 ? LOW_BRIGHTNESS
                           : pass == 1 ? MID_BRIGHTNESS
                                       : FULL_BRIGHTNESS;
    renderPattern12BuildPass(sweepStep, frameInStep, BUILD_STEP_FRAMES,
                             passBrightness);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - buildFrames);
  if (cycleFrame < HOLD_FRAMES) {
    // Only a tiny visible shimmer at the full-bright hold. The moving sweep
    // itself stays clean and non-flickery.
    uint8_t holdBrightness = (cycleFrame == 1) ? 230 : FULL_BRIGHTNESS;
    renderPattern12FullShapeSolid(holdBrightness);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - HOLD_FRAMES);
  if (cycleFrame < offFrames) {
    uint8_t sweepStep = static_cast<uint8_t>(cycleFrame / OFF_STEP_FRAMES);
    uint8_t frameInStep = static_cast<uint8_t>(cycleFrame % OFF_STEP_FRAMES);
    renderPattern12FadeOff(sweepStep, frameInStep, OFF_STEP_FRAMES);
    return;
  }
}


static const uint8_t PATTERN13_TOTAL_PATH_STEPS = 40;

static void renderPattern13TopFiveWindow(uint8_t step, uint8_t brightness) {
  // Five-LED window starts at 78-74 and 55-59, then travels to the
  // rightmost top corner region 72-68 and 63-67.
  static const uint8_t MAX_STEP = 8;
  uint8_t clamped = step > MAX_STEP ? MAX_STEP : step;
  uint8_t upperStart = pattern3LerpLed(78, 72, clamped, MAX_STEP);
  uint8_t lowerStart = pattern3LerpLed(55, 63, clamped, MAX_STEP);

  renderPattern3LedRange(upperStart, static_cast<uint8_t>(upperStart - 4), brightness);
  renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart + 4), brightness);
}

static void renderPattern13FiveLaneCrossfade(const uint8_t *sourceA,
                                             const uint8_t *sourceB,
                                             const uint8_t *targetA,
                                             const uint8_t *targetB,
                                             uint8_t step,
                                             uint8_t maxStep,
                                             uint8_t brightness) {
  if (maxStep == 0) {
    maxStep = 1;
  }

  uint8_t clamped = step > maxStep ? maxStep : step;
  uint8_t raw = static_cast<uint8_t>((static_cast<uint16_t>(clamped) * 255) / maxStep);

  for (uint8_t lane = 0; lane < 5; lane++) {
    // Small lane delay gives the drop a motion direction instead of one hard jump.
    uint8_t laneDelay = static_cast<uint8_t>(lane * 13);
    uint8_t laneRaw = raw > laneDelay ? static_cast<uint8_t>(raw - laneDelay) : 0;
    uint8_t eased = pattern3EaseInOut(laneRaw);
    uint8_t sourceBrightness =
        pattern3ScaleBrightness(brightness, static_cast<uint8_t>(255 - eased));
    uint8_t targetBrightness = pattern3ScaleBrightness(brightness, eased);

    renderPattern3SingleLed(sourceA[lane], sourceBrightness);
    renderPattern3SingleLed(sourceB[lane], sourceBrightness);
    renderPattern3SingleLed(targetA[lane], targetBrightness);
    renderPattern3SingleLed(targetB[lane], targetBrightness);
  }
}

static void renderPattern13TopToMiddle(uint8_t step, uint8_t brightness) {
  static const uint8_t MAX_STEP = 7;
  static const uint8_t sourceA[] = {72, 71, 70, 69, 68};
  static const uint8_t sourceB[] = {63, 64, 65, 66, 67};
  // Five-lane middle crossing now includes LED 44 and LED 35.
  static const uint8_t targetA[] = {44, 43, 42, 41, 40};
  static const uint8_t targetB[] = {35, 36, 37, 38, 39};
  renderPattern13FiveLaneCrossfade(sourceA, sourceB, targetA, targetB,
                                   step, MAX_STEP, brightness);
}

static void renderPattern13MiddleToBottomSingleRow(uint8_t step,
                                                   uint8_t brightness) {
  static const uint8_t MAX_STEP = 7;
  static const uint8_t sourceA[] = {44, 43, 42, 41, 40};
  static const uint8_t sourceB[] = {35, 36, 37, 38, 39};
  // Both bottom rows now participate instead of only the lower row.
  static const uint8_t targetA[] = {24, 23, 22, 21, 20};
  static const uint8_t targetB[] = {1, 2, 3, 4, 5};
  renderPattern13FiveLaneCrossfade(sourceA, sourceB, targetA, targetB,
                                   step, MAX_STEP, brightness);
}

static void renderPattern13BottomSingleRowWindow(uint8_t step,
                                                 uint8_t brightness) {
  // Five-LED bottom window now uses both bottom rows and rises into the
  // added middle endpoint near the end of the travel.
  static const uint8_t MAX_STEP = 14;
  uint8_t clamped = step > MAX_STEP ? MAX_STEP : step;

  if (clamped <= 6) {
    uint8_t upperStart = pattern3LerpLed(24, 20, clamped, 6);
    uint8_t lowerStart = pattern3LerpLed(1, 5, clamped, 6);
    renderPattern3LedRange(upperStart, static_cast<uint8_t>(upperStart - 4), brightness);
    renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart + 4), brightness);
    return;
  }

  uint8_t verticalStep = static_cast<uint8_t>(clamped - 7);
  renderPattern3LedRange(24, 20, pattern3ScaleBrightness(brightness, 120));
  renderPattern3LedRange(1, 5, pattern3ScaleBrightness(brightness, 120));
  renderOppositeMiddleLowerBlock(brightness);
  if (verticalStep >= 4) {
    renderOppositeMiddleUpperBlock(brightness);
  }
}

static void renderPattern13PathWindow(uint8_t pathStep, uint8_t brightness) {
  if (pathStep < 9) {
    renderPattern13TopFiveWindow(pathStep, brightness);
    return;
  }

  if (pathStep < 17) {
    renderPattern13TopToMiddle(static_cast<uint8_t>(pathStep - 9), brightness);
    return;
  }

  if (pathStep < 25) {
    renderPattern13MiddleToBottomSingleRow(static_cast<uint8_t>(pathStep - 17),
                                           brightness);
    return;
  }

  renderPattern13BottomSingleRowWindow(static_cast<uint8_t>(pathStep - 25),
                                       brightness);
}

static void renderPattern13MovingWindow(uint8_t pathStep, uint8_t brightness) {
  if (pathStep > 0) {
    renderPattern13PathWindow(static_cast<uint8_t>(pathStep - 1),
                              pattern3ScaleBrightness(brightness, 112));
  }
  if (pathStep + 1 < PATTERN13_TOTAL_PATH_STEPS) {
    renderPattern13PathWindow(static_cast<uint8_t>(pathStep + 1),
                              pattern3ScaleBrightness(brightness, 42));
  }
  renderPattern13PathWindow(pathStep, brightness);
}

static void renderPattern13FillToStep(uint8_t pathStep, uint8_t lockedBrightness,
                                      uint8_t activeBrightness) {
  uint8_t limit = pathStep >= PATTERN13_TOTAL_PATH_STEPS
                      ? static_cast<uint8_t>(PATTERN13_TOTAL_PATH_STEPS - 1)
                      : pathStep;
  for (uint8_t i = 0; i <= limit; i++) {
    renderPattern13PathWindow(i, lockedBrightness);
  }
  renderPattern13PathWindow(limit, activeBrightness);
}

static void renderPattern13FullPath(uint8_t brightness) {
  // Final/full stage: show the complete intended route at one brightness.
  renderPattern3LedRange(78, 68, brightness);
  renderPattern3LedRange(55, 67, brightness);
  renderOppositeMiddleBlock(brightness);
  renderPattern3LedRange(44, 40, brightness);
  renderPattern3LedRange(35, 39, brightness);
  renderPattern3LedRange(12, 24, brightness);
  renderPattern3LedRange(11, 1, brightness);
}

static void patternSingleRowReturnStack(uint32_t frame) {
  // Animation 13 has three visual stages:
  // 1) A five-LED moving window travels from the top pair down into the single
  //    lower bottom row and ends on LEDs 5-1.
  // 2) The same moving window returns upward/backward without leaving a fill.
  // 3) The path repeats as a stack/fill pass where previous LEDs remain ON;
  //    once the path reaches LED 1, the complete path flickers three times.
  static const uint8_t MOVE_FRAME_SCALE = 1;
  static const uint8_t ACTIVE_BRIGHTNESS = 255;
  static const uint8_t LOCKED_BRIGHTNESS = 170;
  static const uint8_t BOTTOM_HOLD_FRAMES = 4;
  static const uint8_t TOP_HOLD_FRAMES = 3;
  static const uint8_t FILL_HOLD_FRAMES = 4;
  static const uint8_t FLICKER_ON_FRAMES = 4;
  static const uint8_t FLICKER_OFF_FRAMES = 4;
  static const uint8_t FLICKER_COUNT = 3;
  static const uint8_t BLANK_FRAMES = 3;

  const uint16_t outboundFrames = PATTERN13_TOTAL_PATH_STEPS;
  const uint16_t returnFrames = PATTERN13_TOTAL_PATH_STEPS;
  const uint16_t fillFrames = PATTERN13_TOTAL_PATH_STEPS;
  const uint16_t flickerFrames =
      static_cast<uint16_t>(FLICKER_COUNT) *
      static_cast<uint16_t>(FLICKER_ON_FRAMES + FLICKER_OFF_FRAMES);
  const uint16_t totalFrames =
      static_cast<uint16_t>(outboundFrames + BOTTOM_HOLD_FRAMES + returnFrames +
                            TOP_HOLD_FRAMES + fillFrames + FILL_HOLD_FRAMES +
                            flickerFrames + BLANK_FRAMES);

  uint16_t cycleFrame = static_cast<uint16_t>((frame * MOVE_FRAME_SCALE) % totalFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < outboundFrames) {
    renderPattern13MovingWindow(static_cast<uint8_t>(cycleFrame), ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - outboundFrames);
  if (cycleFrame < BOTTOM_HOLD_FRAMES) {
    renderPattern13BottomSingleRowWindow(14, ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - BOTTOM_HOLD_FRAMES);
  if (cycleFrame < returnFrames) {
    uint8_t reverseStep = static_cast<uint8_t>(
        (PATTERN13_TOTAL_PATH_STEPS - 1) - cycleFrame);
    renderPattern13MovingWindow(reverseStep, ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - returnFrames);
  if (cycleFrame < TOP_HOLD_FRAMES) {
    renderPattern13TopFiveWindow(0, ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - TOP_HOLD_FRAMES);
  if (cycleFrame < fillFrames) {
    renderPattern13FillToStep(static_cast<uint8_t>(cycleFrame), LOCKED_BRIGHTNESS,
                              ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - fillFrames);
  if (cycleFrame < FILL_HOLD_FRAMES) {
    renderPattern13FullPath(ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint16_t>(cycleFrame - FILL_HOLD_FRAMES);
  if (cycleFrame < flickerFrames) {
    uint8_t phaseFrame = static_cast<uint8_t>(cycleFrame %
        (FLICKER_ON_FRAMES + FLICKER_OFF_FRAMES));
    if (phaseFrame < FLICKER_ON_FRAMES) {
      renderPattern13FullPath(ACTIVE_BRIGHTNESS);
    }
    return;
  }
}


static void renderPattern14TopPair(uint8_t brightness) {
  renderPattern3LedRange(78, 68, brightness);
  renderPattern3LedRange(55, 67, brightness);
  renderOppositeMiddleBlock(brightness);
}

static void renderPattern14MiddleFive(uint8_t brightness) {
  // Selected middle pair for this pattern uses five LEDs on each side.
  // This intentionally includes LEDs 44 and 35 because the requested final
  // block was extended from four LEDs to five LEDs for Animation 14.
  renderPattern3LedRange(40, 44, brightness);
  renderPattern3LedRange(39, 35, brightness);
}

static void renderPattern14BottomPair(uint8_t brightness) {
  renderPattern3LedRange(12, 24, brightness);
  renderPattern3LedRange(11, 1, brightness);
}

static void renderPattern14Scene(uint8_t scene, uint8_t brightness) {
  switch (scene) {
    case 0:
      renderPattern14TopPair(brightness);
      break;

    case 1:
      renderPattern14BottomPair(brightness);
      renderPattern14MiddleFive(brightness);
      break;

    default:
      renderPattern14TopPair(brightness);
      renderPattern14MiddleFive(brightness);
      renderPattern14BottomPair(brightness);
      break;
  }
}

static uint8_t pattern14FlickerBrightness(uint8_t onFrame,
                                          uint8_t onFrames) {
  // A tiny leading/trailing brightness shape makes the flicker visible without
  // becoming a harsh instant blink.
  if (onFrames <= 2) {
    return 255;
  }

  if (onFrame == 0) {
    return 190;
  }

  if (onFrame + 1 >= onFrames) {
    return 220;
  }

  return 255;
}

static void patternThreeZoneFlicker(uint32_t frame) {
  // Animation 14:
  // 1) Full top two rows flicker three times.
  // 2) Bottom pair plus the selected five-LED middle pair flicker three times.
  // 3) Top, bottom, and selected middle LEDs flicker together three times.
  // Then the pattern blanks briefly and repeats.
  static const uint8_t FLICKER_COUNT = 3;
  static const uint8_t ON_FRAMES = 5;
  static const uint8_t OFF_FRAMES = 4;
  static const uint8_t SCENE_GAP_FRAMES = 3;
  static const uint8_t BLANK_FRAMES = 5;

  const uint8_t flickerCycleFrames = ON_FRAMES + OFF_FRAMES;
  const uint8_t flickerFrames = FLICKER_COUNT * flickerCycleFrames;
  const uint8_t sceneFrames = flickerFrames + SCENE_GAP_FRAMES;
  const uint16_t totalFrames = static_cast<uint16_t>(sceneFrames) * 3U +
                               BLANK_FRAMES;

  uint16_t cycleFrame = static_cast<uint16_t>(frame % totalFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame >= static_cast<uint16_t>(sceneFrames) * 3U) {
    return;
  }

  uint8_t scene = static_cast<uint8_t>(cycleFrame / sceneFrames);
  uint8_t sceneFrame = static_cast<uint8_t>(cycleFrame % sceneFrames);

  if (sceneFrame >= flickerFrames) {
    return;
  }

  uint8_t phaseFrame = static_cast<uint8_t>(sceneFrame % flickerCycleFrames);
  if (phaseFrame >= ON_FRAMES) {
    return;
  }

  uint8_t brightness = pattern14FlickerBrightness(phaseFrame, ON_FRAMES);
  renderPattern14Scene(scene, brightness);
}


static void renderPattern15TopFirstFive(uint8_t brightness) {
  renderPattern3LedRange(78, 74, brightness);
  renderPattern3LedRange(55, 59, brightness);
}

static void renderPattern15TopSecondFive(uint8_t brightness) {
  renderPattern3LedRange(73, 69, brightness);
  renderPattern3LedRange(60, 64, brightness);
}

static void renderPattern15MiddleEntry(uint8_t brightness) {
  renderPattern3LedRange(68, 68, brightness);
  renderPattern3LedRange(65, 67, brightness);
  renderPattern3LedRange(44, 40, brightness);
  renderPattern3LedRange(35, 39, brightness);
  renderPattern3LedRange(15, 12, brightness);
  renderPattern3LedRange(8, 11, brightness);
}

static void renderPattern15BottomFinish(uint8_t brightness) {
  renderPattern3LedRange(1, 7, brightness);
  renderPattern3LedRange(24, 16, brightness);
  // Added circular close-out: left-reference middle endpoint flickers with
  // the bottom finish so the final stage reads as a closed perimeter.
  renderOppositeMiddleBlock(brightness);
}

static void renderPattern15Group(uint8_t group, uint8_t brightness) {
  switch (group) {
    case 0:
      renderPattern15TopFirstFive(brightness);
      break;

    case 1:
      renderPattern15TopSecondFive(brightness);
      break;

    case 2:
      renderPattern15MiddleEntry(brightness);
      break;

    default:
      renderPattern15BottomFinish(brightness);
      break;
  }
}

static void renderPattern15Scene(uint8_t scene, uint8_t brightness) {
  // Cumulative scene renderer:
  // Scene 0 = group 0 only.
  // Scene 1 = group 0 + group 1.
  // Scene 2 = group 0 + group 1 + group 2.
  // Scene 3 = all groups together.
  for (uint8_t group = 0; group <= scene && group < 4; group++) {
    renderPattern15Group(group, brightness);
  }
}

static uint8_t pattern15Scale(uint8_t value, uint8_t scale) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale) / 255U);
}

static uint8_t pattern15FlickerBrightness(uint8_t onFrame,
                                          uint8_t onFrames) {
  if (onFrames <= 2) {
    return 255;
  }

  if (onFrame == 0) {
    return 150;
  }

  if (onFrame == 1) {
    return 220;
  }

  if (onFrame + 1 >= onFrames) {
    return 205;
  }

  return 255;
}

static void patternStagedSmoothFlicker(uint32_t frame) {
  // Animation 15:
  // 1) First top 5+5 LEDs flicker three times.
  // 2) First group + second top 5+5 LEDs flicker together three times.
  // 3) First two groups + remaining top/middle/bottom-entry LEDs flicker
  //    together three times.
  // 4) All groups flicker together three times, then the cycle repeats.
  // Transition frames fade the newly added group into the already-active
  // cumulative scene so the next section does not pop on harshly.
  static const uint8_t SCENE_COUNT = 4;
  static const uint8_t FLICKER_COUNT = 3;
  static const uint8_t ON_FRAMES = 5;
  static const uint8_t OFF_FRAMES = 3;
  static const uint8_t TRANSITION_FRAMES = 4;
  static const uint8_t BLANK_FRAMES = 5;

  const uint8_t flickerCycleFrames = ON_FRAMES + OFF_FRAMES;
  const uint8_t flickerFrames = FLICKER_COUNT * flickerCycleFrames;
  const uint8_t sceneFrames = TRANSITION_FRAMES + flickerFrames;
  const uint16_t activeFrames = static_cast<uint16_t>(sceneFrames) * SCENE_COUNT;
  const uint16_t totalFrames = activeFrames + BLANK_FRAMES;

  uint16_t cycleFrame = static_cast<uint16_t>(frame % totalFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame >= activeFrames) {
    return;
  }

  uint8_t scene = static_cast<uint8_t>(cycleFrame / sceneFrames);
  uint8_t sceneFrame = static_cast<uint8_t>(cycleFrame % sceneFrames);

  if (sceneFrame < TRANSITION_FRAMES) {
    uint8_t baseHold = 135;
    uint8_t newGroupFade = static_cast<uint8_t>(35 +
        (static_cast<uint16_t>(sceneFrame) * 145U) /
            (TRANSITION_FRAMES - 1));

    if (scene > 0) {
      renderPattern15Scene(static_cast<uint8_t>(scene - 1), baseHold);
    }

    renderPattern15Group(scene, newGroupFade);
    return;
  }

  uint8_t flickerFrame = static_cast<uint8_t>(sceneFrame - TRANSITION_FRAMES);
  uint8_t phaseFrame = static_cast<uint8_t>(flickerFrame % flickerCycleFrames);

  if (phaseFrame < ON_FRAMES) {
    uint8_t brightness = pattern15FlickerBrightness(phaseFrame, ON_FRAMES);
    renderPattern15Scene(scene, brightness);
  } else {
    // Very low residual light during the OFF half keeps the flicker visible but
    // avoids a harsh hard-black cut between stages.
    renderPattern15Scene(scene, 8);
  }
}



static void setAnimationPixelAllStrips(uint8_t rowIdx, uint8_t column,
                                       uint8_t brightness) {
  if (rowIdx >= PCB_ROW_COUNT || column >= PCB_ROWS[rowIdx].length) {
    return;
  }

  uint16_t pixelIndex = rowColumnToIndex(PCB_ROWS[rowIdx], column);
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    setAnimationPixel(stripId, pixelIndex, brightness);
  }
}

static uint8_t mapRainColumnToRow(uint8_t baseColumn, const RowLayout &row,
                                  uint8_t maxColumns) {
  if (maxColumns <= 1 || row.length <= 1) {
    return 0;
  }

  return static_cast<uint8_t>(
      (static_cast<uint16_t>(baseColumn) * (row.length - 1) +
       (maxColumns - 1) / 2) /
      (maxColumns - 1));
}

static void renderRainDropPixel(uint8_t visualRowFromTop, uint8_t baseColumn,
                                uint8_t brightness) {
  if (visualRowFromTop >= PCB_ROW_COUNT) {
    return;
  }

  uint8_t rowIdx = static_cast<uint8_t>((PCB_ROW_COUNT - 1) - visualRowFromTop);
  const RowLayout &row = PCB_ROWS[rowIdx];
  uint8_t maxColumns = getMaxRowLength();
  uint8_t column = mapRainColumnToRow(baseColumn, row, maxColumns);

  setAnimationPixelAllStrips(rowIdx, column, brightness);

  if (brightness > 70 && column > 0) {
    setAnimationPixelAllStrips(rowIdx, column - 1, brightness / 4);
  }
  if (brightness > 70 && column + 1 < row.length) {
    setAnimationPixelAllStrips(rowIdx, column + 1, brightness / 4);
  }
}


static void renderPattern16TopWindow(uint8_t step, uint8_t brightness) {
  static const uint8_t TOP_LERP_MAX = 11;
  uint8_t topStart = pattern3LerpLed(78, 72, step, TOP_LERP_MAX);
  uint8_t secondStart = pattern3LerpLed(55, 63, step, TOP_LERP_MAX);

  renderPattern3LedRange(topStart, static_cast<uint8_t>(topStart - 4),
                         brightness);
  renderPattern3LedRange(secondStart, static_cast<uint8_t>(secondStart + 4),
                         brightness);
}

static void renderPattern16BottomHorizontalWindow(uint8_t step,
                                                   uint8_t maxStep,
                                                   uint8_t brightness) {
  uint8_t clampedStep = step > maxStep ? maxStep : step;
  uint8_t upperStart = pattern3LerpLed(24, 16, clampedStep, maxStep);
  uint8_t lowerStart = pattern3LerpLed(1, 7, clampedStep, maxStep);

  renderPattern3LedRange(upperStart, static_cast<uint8_t>(upperStart - 4),
                         brightness);
  renderPattern3LedRange(lowerStart, static_cast<uint8_t>(lowerStart + 4),
                         brightness);
}

static void renderPattern16BottomToMiddleWindow(uint8_t step,
                                                uint8_t brightness) {
  static const uint8_t STEP_MAX = 11;
  static const uint8_t HORIZONTAL_END_STEP = 6;
  static const uint8_t upperBottomRight[] = {16, 15, 14, 13, 12};
  static const uint8_t upperMiddleTarget[] = {44, 43, 42, 41, 40};
  static const uint8_t lowerBottomRight[] = {7, 8, 9, 10, 11};
  static const uint8_t lowerMiddleTarget[] = {35, 36, 37, 38, 39};

  uint8_t clampedStep = step > STEP_MAX ? STEP_MAX : step;

  if (clampedStep <= HORIZONTAL_END_STEP) {
    renderPattern16BottomHorizontalWindow(clampedStep, HORIZONTAL_END_STEP,
                                          brightness);
    return;
  }

  uint8_t bridgeStep = static_cast<uint8_t>(clampedStep - HORIZONTAL_END_STEP);
  uint8_t bridgeMax = static_cast<uint8_t>(STEP_MAX - HORIZONTAL_END_STEP);

  renderPattern3LaneCrossfade(upperBottomRight, upperMiddleTarget, 5,
                              bridgeStep, bridgeMax, brightness);
  renderPattern3LaneCrossfade(lowerBottomRight, lowerMiddleTarget, 5,
                              bridgeStep, bridgeMax, brightness);
}

static void renderPattern16LeftEndpointTransition(uint8_t progress,
                                                  uint8_t brightness) {
  static const uint8_t STEP_MAX = 11;
  static const uint8_t HORIZONTAL_END_STEP = 6;
  static const uint8_t upperTopLeft[] = {55, 56, 57, 58, 59};
  static const uint8_t upperMiddleLeft[] = {54, 53, 52, 51, 50};
  static const uint8_t lowerBottomLeft[] = {24, 23, 22, 21, 20};
  static const uint8_t lowerMiddleLeft[] = {25, 26, 27, 28, 29};

  uint8_t clamped = progress > STEP_MAX ? STEP_MAX : progress;
  if (clamped <= HORIZONTAL_END_STEP) {
    return;
  }

  uint8_t bridgeStep = static_cast<uint8_t>(clamped - HORIZONTAL_END_STEP);
  uint8_t bridgeMax = static_cast<uint8_t>(STEP_MAX - HORIZONTAL_END_STEP);
  renderPattern3LaneCrossfade(upperTopLeft, upperMiddleLeft, 5,
                              bridgeStep, bridgeMax, brightness);
  renderPattern3LaneCrossfade(lowerBottomLeft, lowerMiddleLeft, 5,
                              bridgeStep, bridgeMax, brightness);
}

static void forcePattern16ExcludedOff() {
  // Hardware test showed LED 49 as an unwanted artifact in this animation.
  // Keep it dark after every pattern-16 draw call, regardless of overlay order.
  renderPattern3SingleLed(49, 0);
}

static void renderPattern16ForwardWindow(uint8_t step, uint8_t brightness) {
  renderPattern16TopWindow(step, brightness);
  renderPattern16BottomToMiddleWindow(step, brightness);
  forcePattern16ExcludedOff();
}

static void renderPattern16ReturnWindow(uint8_t step, uint8_t brightness) {
  static const uint8_t STEP_MAX = 11;
  uint8_t leftProgress = step > STEP_MAX ? 0
                                         : static_cast<uint8_t>(STEP_MAX - step);

  renderPattern16TopWindow(step, brightness);
  renderPattern16BottomHorizontalWindow(step, STEP_MAX, brightness);
  renderPattern16LeftEndpointTransition(leftProgress, brightness);
  forcePattern16ExcludedOff();
}

static void patternMirrorBridgeReturn(uint32_t frame) {
  static const uint8_t MOVE_FRAMES = 14;
  static const uint8_t RIGHT_HOLD_FRAMES = 5;
  static const uint8_t RETURN_FRAMES = 14;
  static const uint8_t LEFT_HOLD_FRAMES = RIGHT_HOLD_FRAMES;
  static const uint8_t BLANK_FRAMES = 5;
  static const uint8_t STEP_MAX = 11;
  static const uint8_t ACTIVE_BRIGHTNESS = 255;
  static const uint8_t TRAIL_BRIGHTNESS = 92;
  static const uint8_t HOLD_BRIGHTNESS = 230;

  const uint8_t totalFrames = static_cast<uint8_t>(
      MOVE_FRAMES + RIGHT_HOLD_FRAMES + RETURN_FRAMES + LEFT_HOLD_FRAMES +
      BLANK_FRAMES);
  uint8_t cycleFrame = static_cast<uint8_t>(frame % totalFrames);

  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < MOVE_FRAMES) {
    uint8_t step = static_cast<uint8_t>(
        (static_cast<uint16_t>(cycleFrame) * STEP_MAX) / (MOVE_FRAMES - 1));
    if (step > 0) {
      renderPattern16ForwardWindow(static_cast<uint8_t>(step - 1),
                                   TRAIL_BRIGHTNESS);
    }
    renderPattern16ForwardWindow(step, ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint8_t>(cycleFrame - MOVE_FRAMES);
  if (cycleFrame < RIGHT_HOLD_FRAMES) {
    uint8_t holdBrightness = (cycleFrame & 0x01) ? HOLD_BRIGHTNESS
                                                 : ACTIVE_BRIGHTNESS;
    renderPattern16ForwardWindow(STEP_MAX, holdBrightness);
    return;
  }

  cycleFrame = static_cast<uint8_t>(cycleFrame - RIGHT_HOLD_FRAMES);
  if (cycleFrame < RETURN_FRAMES) {
    uint8_t forwardStep = static_cast<uint8_t>(
        (static_cast<uint16_t>(cycleFrame) * STEP_MAX) / (RETURN_FRAMES - 1));
    uint8_t step = static_cast<uint8_t>(STEP_MAX - forwardStep);
    if (step < STEP_MAX) {
      renderPattern16ReturnWindow(static_cast<uint8_t>(step + 1),
                                  TRAIL_BRIGHTNESS);
    }
    renderPattern16ReturnWindow(step, ACTIVE_BRIGHTNESS);
    return;
  }

  cycleFrame = static_cast<uint8_t>(cycleFrame - RETURN_FRAMES);
  if (cycleFrame < LEFT_HOLD_FRAMES) {
    uint8_t holdBrightness = (cycleFrame & 0x01) ? HOLD_BRIGHTNESS
                                                 : ACTIVE_BRIGHTNESS;
    renderPattern16ReturnWindow(0, holdBrightness);
    return;
  }
}


static uint8_t pattern17ScaleByEnvelope(uint8_t brightness, uint8_t localFrame,
                                       uint8_t duration) {
  if (duration <= 2) {
    return brightness;
  }

  uint8_t scale = 255;
  const uint8_t rampFrames = 5;
  if (localFrame < rampFrames) {
    scale = static_cast<uint8_t>(80 +
        (static_cast<uint16_t>(localFrame) * 175U) / rampFrames);
  } else if (localFrame + rampFrames >= duration) {
    uint8_t remaining = static_cast<uint8_t>(duration - 1 - localFrame);
    scale = static_cast<uint8_t>(65 +
        (static_cast<uint16_t>(remaining) * 190U) / rampFrames);
  }

  return pattern3ScaleBrightness(brightness, scale);
}

static uint8_t pattern17ProgressStep(uint8_t localFrame, uint8_t duration,
                                     uint8_t maxStep) {
  if (duration <= 1 || maxStep == 0) {
    return 0;
  }
  uint16_t scaled = static_cast<uint16_t>(localFrame) * maxStep;
  uint8_t step = static_cast<uint8_t>(scaled / (duration - 1));
  return step > maxStep ? maxStep : step;
}

static const uint8_t PATTERN17_CLIP_COUNT = 20;

static uint8_t triangleWave8(uint16_t value, uint16_t period);
static uint8_t distanceFalloff(uint8_t distance, uint8_t radius);
static uint8_t pattern19FoldScore(uint8_t rowIdx, uint8_t column);
static uint8_t pattern19FoldPixelBrightness(uint8_t score, uint8_t progress,
                                            uint8_t baseBrightness);
static uint8_t pattern20HeartbeatGain(uint8_t beatFrame);
static void renderPattern18RightWipe(uint8_t step, uint8_t maxStep,
                                     uint8_t brightness);

static void renderPattern17RandomSparkles(uint8_t localFrame, uint8_t variant,
                                          uint8_t brightness) {
  static const uint8_t LEDS[] = {
      78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
      54, 53, 52, 51, 50,
      44, 43, 42, 41, 40, 39, 38, 37, 36, 35,
      25, 26, 27, 28, 29,
      12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
      11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
  };
  static const uint8_t LED_COUNT = sizeof(LEDS) / sizeof(LEDS[0]);

  for (uint8_t i = 0; i < 8; i++) {
    uint32_t hash = (static_cast<uint32_t>(localFrame + 1) * 1103515245UL) ^
                    (static_cast<uint32_t>(variant + 17) * 2654435761UL) ^
                    (static_cast<uint32_t>(i) * 2246822519UL);
    uint8_t led = LEDS[hash % LED_COUNT];
    uint8_t sparkleScale = static_cast<uint8_t>(90 + ((hash >> 11) % 166));
    renderPattern3SingleLed(led, pattern3ScaleBrightness(brightness,
                                                         sparkleScale));
  }
}

static void renderPattern17Clip(uint8_t clipId, uint8_t localFrame,
                                uint8_t duration, uint8_t variant,
                                uint8_t brightness) {
  if (duration == 0) {
    duration = 1;
  }

  uint8_t b = pattern17ScaleByEnvelope(brightness, localFrame, duration);
  uint8_t phase3 = static_cast<uint8_t>(
      (static_cast<uint16_t>(localFrame) * 3U) / duration);
  if (phase3 > 2) {
    phase3 = 2;
  }

  switch (clipId % PATTERN17_CLIP_COUNT) {
    case 0: {
      // Animation-1 style: alternating circular flow clusters.
      if (((localFrame / 8U) + variant) & 0x01U) {
        renderPattern3LedRange(4, 8, b);
        renderPattern3LedRange(16, 21, b);
        renderPattern3LedRange(35, 39, pattern3ScaleBrightness(b, 180));
        renderPattern3LedRange(68, 74, b);
      } else {
        renderPattern3LedRange(1, 4, b);
        renderPattern3LedRange(8, 11, b);
        renderPattern3LedRange(25, 33, pattern3ScaleBrightness(b, 180));
        renderPattern3LedRange(55, 67, b);
      }
      break;
    }

    case 1: {
      // Animation-2 style: top-to-bottom row sweep.
      uint8_t sweep = pattern17ProgressStep(localFrame, duration, 3);
      for (uint8_t visualRow = 0; visualRow < 3; visualRow++) {
        uint8_t rowFromTop = static_cast<uint8_t>(sweep + visualRow);
        if (rowFromTop >= PCB_ROW_COUNT) {
          continue;
        }
        uint8_t rowIdx = static_cast<uint8_t>((PCB_ROW_COUNT - 1) - rowFromTop);
        const RowLayout &row = PCB_ROWS[rowIdx];
        for (uint8_t column = 0; column < row.length; column++) {
          setAnimationPixelAllStrips(rowIdx, column,
                                     visualRow == 0 ? b : pattern3ScaleBrightness(b, 110));
        }
      }
      break;
    }

    case 2: {
      // Animation-3 style: top sweep, then drop through middle/bottom.
      uint8_t step = pattern17ProgressStep(localFrame, duration, 35);
      if (step < 12) {
        if (step > 0) {
          renderPattern3TopWindow(static_cast<uint8_t>(step - 1),
                                  pattern3ScaleBrightness(b, 95));
        }
        renderPattern3TopWindow(step, b);
      } else if (step < 24) {
        renderPattern3TopToMiddleWindow(static_cast<uint8_t>(step - 12), b);
      } else {
        uint8_t bottomStep = static_cast<uint8_t>(step - 24);
        renderPattern3MiddleToBottomWindow(bottomStep, b);
        if (bottomStep > 8) {
          renderOppositeMiddleLowerBlock(pattern3ScaleBrightness(b, 150));
        }
        if (bottomStep > 10) {
          renderOppositeMiddleUpperBlock(pattern3ScaleBrightness(b, 150));
        }
      }
      break;
    }

    case 3: {
      // Animation-4 style: center-out alert pulse.
      uint8_t stage = pattern17ProgressStep(localFrame, duration, 11);
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        int center = row.length / 2;
        for (uint8_t column = 0; column < row.length; column++) {
          int distance = static_cast<int>(column) - center;
          if (distance < 0) {
            distance = -distance;
          }
          if (distance <= stage && distance + 2 >= stage) {
            setAnimationPixelAllStrips(rowIdx, column, b);
          }
        }
      }
      break;
    }

    case 4: {
      // Animation-5 style: stacked fill over the full PCB shape.
      uint8_t totalColumns = 0;
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        totalColumns += PCB_ROWS[rowIdx].length;
      }
      uint8_t activeCount = pattern17ProgressStep(localFrame, duration,
                                                  totalColumns);
      uint8_t cursor = 0;
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        for (uint8_t column = 0; column < row.length; column++) {
          if (cursor < activeCount) {
            setAnimationPixelAllStrips(rowIdx, column,
                                       cursor + 4 > activeCount ? b
                                                                : pattern3ScaleBrightness(b, 95));
          }
          cursor++;
        }
      }
      break;
    }

    case 5: {
      // Animation-6 style: raindrop fragment.
      uint8_t maxColumns = getMaxRowLength();
      for (uint8_t drop = 0; drop < 5; drop++) {
        uint8_t head = static_cast<uint8_t>((localFrame + (drop * 11U) + variant) % 54U);
        uint8_t baseColumn = static_cast<uint8_t>((drop * 3U + variant) % maxColumns);
        for (uint8_t visualRow = 0; visualRow < PCB_ROW_COUNT; visualRow++) {
          int16_t distance = static_cast<int16_t>(head) -
                             static_cast<int16_t>(visualRow * 8U);
          if (distance >= 0 && distance <= 14) {
            renderRainDropPixel(visualRow, baseColumn,
                                pattern3ScaleBrightness(b, static_cast<uint8_t>(255 - distance * 13)));
          }
        }
      }
      break;
    }

    case 6: {
      // Animation-7 style: scanner.
      uint8_t maxColumns = getMaxRowLength();
      uint8_t period = static_cast<uint8_t>((maxColumns - 1U) * 2U);
      uint8_t phase = static_cast<uint8_t>((localFrame + variant) % period);
      uint8_t head = phase < maxColumns ? phase : static_cast<uint8_t>(period - phase);
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        uint8_t rowHead = mapRainColumnToRow(head, row, maxColumns);
        for (uint8_t column = 0; column < row.length; column++) {
          uint8_t distance = column > rowHead ? column - rowHead : rowHead - column;
          uint8_t brightness = distanceFalloff(distance, 5);
          if (brightness > 0) {
            setAnimationPixelAllStrips(rowIdx, column,
                                       pattern3ScaleBrightness(b, brightness));
          }
        }
      }
      break;
    }

    case 7: {
      // Animation-8 style: pulse wave.
      uint8_t pulse = static_cast<uint8_t>(
          80 + triangleWave8(static_cast<uint16_t>(localFrame * 5U + variant * 17U), 180) / 2);
      renderPattern12FullShapeSolid(pattern3ScaleBrightness(b, pulse));
      break;
    }

    case 8: {
      // Animation-9 style: aurora ribbons over the row geometry.
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        for (uint8_t column = 0; column < row.length; column++) {
          uint8_t shimmer = triangleWave8(
              static_cast<uint16_t>(localFrame * 7U + column * 23U + rowIdx * 31U + variant),
              132);
          if (shimmer > 70) {
            setAnimationPixelAllStrips(rowIdx, column,
                                       pattern3ScaleBrightness(b, shimmer));
          }
        }
      }
      break;
    }

    case 9: {
      // Animation-10 style slot sweep fragment.
      uint8_t step =
          pattern17ProgressStep(localFrame, duration, PATTERN3_SLOT_COUNT);
      uint8_t frameInStep =
          static_cast<uint8_t>(localFrame % PATTERN3_SWEEP_STEP_FRAMES);
      bool forwardOff = (variant & 0x01) == 0;
      renderPattern10SweepPhase(forwardOff, step, frameInStep,
                                PATTERN3_SWEEP_STEP_FRAMES, b);
      break;
    }

    case 10: {
      // Animation-11 style split: a top branch returns while another drops.
      uint8_t half = duration / 2;
      if (half < 2) {
        half = 2;
      }
      if (localFrame < half) {
        uint8_t step = pattern17ProgressStep(localFrame, half, 11);
        renderPattern11TopMover(step, 11, b);
      } else {
        uint8_t splitFrame = static_cast<uint8_t>(localFrame - half);
        uint8_t splitDuration = static_cast<uint8_t>(duration - half);
        renderPattern11InterpolatedTopReturn(splitFrame, splitDuration, b);
        renderPattern11InterpolatedDownPath(splitFrame, splitDuration, b);
      }
      break;
    }

    case 11: {
      // Animation-12 style intensity pass. Variant chooses dim/medium/full.
      static const uint8_t levels[] = {48, 120, 255};
      uint8_t level = levels[variant % 3];
      uint8_t groupCount = pattern12SweepGroupCount();
      uint8_t sweep = pattern17ProgressStep(localFrame, duration,
                                            static_cast<uint8_t>(groupCount - 1));
      uint8_t localStepFrame = localFrame % 3;
      renderPattern12BuildPass(sweep, localStepFrame, 3,
                               pattern3ScaleBrightness(level, b));
      break;
    }

    case 12: {
      // Animation-13 style single-row lower return fragment.
      if (phase3 == 0) {
        uint8_t step = pattern17ProgressStep(localFrame, duration, 8);
        renderPattern3LedRange(pattern3LerpLed(78, 72, step, 8),
                               pattern3LerpLed(74, 68, step, 8), b);
        renderPattern3LedRange(pattern3LerpLed(55, 63, step, 8),
                               pattern3LerpLed(59, 67, step, 8), b);
      } else if (phase3 == 1) {
        uint8_t step = pattern17ProgressStep(localFrame, duration, 11);
        renderPattern3TopToMiddleWindow(step, b);
        renderPattern3MiddleToBottomWindow(step, pattern3ScaleBrightness(b, 130));
      } else {
        uint8_t step = pattern17ProgressStep(localFrame, duration, 14);
        renderPattern13BottomSingleRowWindow(step, b);
      }
      break;
    }

    case 13: {
      // Animation-14/15 style accumulated flicker fragment.
      uint8_t scene = variant % 3;
      uint8_t flickerOn = ((localFrame / 4) & 0x01) == 0;
      uint8_t flickerBrightness = flickerOn ? b : pattern3ScaleBrightness(b, 35);
      renderPattern14Scene(scene, flickerBrightness);
      break;
    }

    case 14: {
      // Animation-15 style staged smooth flicker fragment.
      uint8_t scene = variant % 4;
      uint8_t flickerOn = ((localFrame / 4) & 0x01) == 0;
      uint8_t flickerBrightness = flickerOn ? b : pattern3ScaleBrightness(b, 35);
      renderPattern15Scene(scene, flickerBrightness);
      break;
    }

    case 15: {
      // Animation-16 mirror bridge fragment.
      uint8_t step = pattern17ProgressStep(localFrame, duration, 11);
      if (variant & 0x01) {
        step = static_cast<uint8_t>(11 - step);
        if (step < 11) {
          renderPattern16ReturnWindow(static_cast<uint8_t>(step + 1),
                                      pattern3ScaleBrightness(b, 90));
        }
        renderPattern16ReturnWindow(step, b);
        break;
      }
      if (step > 0 && !(variant & 0x01)) {
        renderPattern16ForwardWindow(static_cast<uint8_t>(step - 1),
                                     pattern3ScaleBrightness(b, 90));
      }
      renderPattern16ForwardWindow(step, b);
      break;
    }

    case 16: {
      // Animation-17 self-fragment: randomized sparkles and a soft full-shape pulse.
      renderPattern12FullShapeSolid(pattern3ScaleBrightness(b, 45));
      renderPattern17RandomSparkles(localFrame, variant, b);
      break;
    }

    case 17: {
      // Animation-18 style right wipe and terminal blink fragment.
      uint8_t step = pattern17ProgressStep(localFrame, duration, 23);
      renderPattern18RightWipe(step, 23, b);
      break;
    }

    case 18: {
      // Animation-19 style center fold, without clearing existing overlays.
      uint8_t progress = pattern17ProgressStep(localFrame, duration, 15);
      if (variant & 0x01U) {
        progress = static_cast<uint8_t>(15 - progress);
      }
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        for (uint8_t column = 0; column < row.length; column++) {
          uint8_t score = pattern19FoldScore(rowIdx, column);
          uint8_t pixelBrightness = pattern19FoldPixelBrightness(score, progress, b);
          if (pixelBrightness > 0) {
            setAnimationPixelAllStrips(rowIdx, column, pixelBrightness);
          }
        }
      }
      break;
    }

    case 19: {
      // Animation-20 style heartbeat wave fragment.
      uint8_t heartbeatGain = pattern20HeartbeatGain(
          static_cast<uint8_t>((localFrame + variant * 3U) % 54U));
      for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
        const RowLayout &row = PCB_ROWS[rowIdx];
        for (uint8_t column = 0; column < row.length; column++) {
          uint8_t wave = triangleWave8(
              static_cast<uint16_t>(localFrame * 4U + column * 31U + rowIdx * 19U),
              120);
          if (wave > 90) {
            uint16_t value = (static_cast<uint16_t>(wave) * heartbeatGain) / 255U;
            setAnimationPixelAllStrips(rowIdx, column,
                                       pattern3ScaleBrightness(b, static_cast<uint8_t>(value)));
          }
        }
      }
      break;
    }

    default: {
      // Sparkle/pulse fragment across the same visible target region.
      uint8_t pulse = static_cast<uint8_t>(
          80 + ((static_cast<uint16_t>((localFrame * 23U) + (variant * 31U)) % 176U)));
      renderPattern12FullShapeSolid(pattern3ScaleBrightness(b, pulse));
      renderPattern17RandomSparkles(localFrame, variant, b);
      break;
    }
  }
}

static void patternRandomizedShowMix(uint32_t frame) {
  // Animation 17: randomizer/show-mix mode. It rotates through the existing
  // animation families in shuffled order, then drops back into blended
  // fragment segments so the result alternates between recognizable patterns
  // and new combinations built from the same animation language.
  static bool initialized = false;
  static uint32_t segmentStartFrame = 0;
  static uint8_t segmentDuration = 44;
  static uint8_t segmentMode = 0;
  static uint8_t directPatternId = 1;
  static uint8_t directFrameScale = 1;
  static uint16_t directFrameOffset = 0;
  static uint8_t primaryClip = 0;
  static uint8_t secondaryClip = 3;
  static uint8_t variant = 0;
  static uint8_t secondaryVariant = 0;
  static uint8_t primaryBrightness = 255;
  static uint8_t secondaryBrightness = 110;
  static bool useSecondary = true;
  static bool useSparkles = false;

  auto nextRandomizedPatternId = [](bool resetBag) -> uint8_t {
    static const uint8_t DIRECT_SOURCE_PATTERNS[] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 18, 19, 20,
    };
    static uint8_t shuffledBag[sizeof(DIRECT_SOURCE_PATTERNS)];
    static uint8_t bagIndex = sizeof(DIRECT_SOURCE_PATTERNS);

    const uint8_t count = sizeof(DIRECT_SOURCE_PATTERNS);
    if (resetBag || bagIndex >= count) {
      for (uint8_t i = 0; i < count; i++) {
        shuffledBag[i] = DIRECT_SOURCE_PATTERNS[i];
      }

      for (int8_t i = static_cast<int8_t>(count) - 1; i > 0; i--) {
        uint8_t j = static_cast<uint8_t>(esp_random() % (i + 1));
        uint8_t tmp = shuffledBag[i];
        shuffledBag[i] = shuffledBag[j];
        shuffledBag[j] = tmp;
      }

      bagIndex = 0;
    }

    return shuffledBag[bagIndex++];
  };

  if (frame == 0 || !initialized || frame < segmentStartFrame ||
      (frame - segmentStartFrame) >= segmentDuration) {
    initialized = true;
    segmentStartFrame = frame;

    uint32_t r = esp_random() ^ (static_cast<uint32_t>(micros()) << 1) ^
                 (static_cast<uint32_t>(millis()) << 16);
    segmentMode = static_cast<uint8_t>(r % 4U);
    directPatternId = nextRandomizedPatternId(frame == 0 || !initialized);
    directFrameScale = static_cast<uint8_t>(1U + ((r >> 3) % 3U));
    directFrameOffset = static_cast<uint16_t>((r >> 7) % 96U);
    primaryClip = static_cast<uint8_t>((r >> 13) % PATTERN17_CLIP_COUNT);
    secondaryClip = static_cast<uint8_t>((r >> 17) % PATTERN17_CLIP_COUNT);
    if (secondaryClip == primaryClip) {
      secondaryClip = static_cast<uint8_t>((secondaryClip + 7U) % PATTERN17_CLIP_COUNT);
    }

    variant = static_cast<uint8_t>((r >> 21) & 0x07U);
    secondaryVariant = static_cast<uint8_t>((r >> 24) & 0x07U);
    segmentDuration = static_cast<uint8_t>(24U + ((r >> 11) % 56U));
    primaryBrightness = static_cast<uint8_t>(185U + ((r >> 27) % 71U));
    secondaryBrightness = static_cast<uint8_t>(76U + ((r >> 19) % 92U));
    useSecondary = ((r >> 31) & 0x01U) != 0U;
    useSparkles = ((r >> 30) & 0x01U) != 0U;
  }

  uint8_t localFrame = static_cast<uint8_t>(frame - segmentStartFrame);
  fillAllStrips(0, 0, 0, 0);

  if (segmentMode == 0 || segmentMode == 2) {
    uint32_t directFrame =
        directFrameOffset +
        (static_cast<uint32_t>(localFrame) * directFrameScale);
    renderAnimationFrame(directPatternId, directFrame);
  }

  if (segmentMode != 0) {
    renderPattern17Clip(primaryClip, localFrame, segmentDuration, variant,
                        primaryBrightness);
  }

  if (segmentMode == 1 || segmentMode == 2 || segmentMode == 3) {
    uint8_t shiftedFrame = static_cast<uint8_t>(
        (localFrame + (segmentDuration / 3U)) % segmentDuration);
    if (useSecondary || segmentMode != 1) {
      renderPattern17Clip(secondaryClip, shiftedFrame, segmentDuration,
                          secondaryVariant, secondaryBrightness);
    }
  }

  if (segmentMode == 3) {
    uint8_t accentFrame = static_cast<uint8_t>(
        (localFrame + (segmentDuration / 2U)) % segmentDuration);
    renderPattern17Clip(static_cast<uint8_t>((primaryClip + 5U) % PATTERN17_CLIP_COUNT),
                        accentFrame, segmentDuration,
                        static_cast<uint8_t>(variant + 5U),
                        pattern3ScaleBrightness(primaryBrightness, 110));
  }

  if (useSparkles) {
    renderPattern17RandomSparkles(
        static_cast<uint8_t>(localFrame + directFrameOffset),
        static_cast<uint8_t>(variant + secondaryVariant),
        pattern3ScaleBrightness(primaryBrightness, 130));
  }
}


static void renderPattern18FullShape(uint8_t brightness) {
  // Active region for Animation 18 now includes the added middle endpoint,
  // LED 44, and LEDs 35-39 so the final blink uses four visible rows.
  renderPattern3LedRange(78, 68, brightness);
  renderPattern3LedRange(55, 67, brightness);
  renderOppositeMiddleBlock(brightness);
  renderPattern3LedRange(44, 40, brightness);
  renderPattern3LedRange(35, 39, brightness);
  renderPattern3LedRange(12, 24, brightness);
  renderPattern3LedRange(11, 1, brightness);
}

static void renderPattern18TerminalMarker(uint8_t brightness) {
  // Final visible end position: right side of the top pair plus four middle rows.
  renderPattern3LedRange(72, 68, brightness);
  renderPattern3LedRange(63, 67, brightness);
  renderPattern3LedRange(44, 40, brightness);
  renderPattern3LedRange(35, 39, brightness);
}

static void renderPattern18ClearLedList(const uint8_t *leds, uint8_t count,
                                        uint8_t clearCount) {
  if (clearCount > count) {
    clearCount = count;
  }

  for (uint8_t i = 0; i < clearCount; i++) {
    renderPattern3SingleLed(leds[i], 0);
  }
}

static uint8_t pattern18ClearCountForStep(uint8_t step, uint8_t maxStep,
                                          uint8_t count) {
  if (count == 0) {
    return 0;
  }
  if (step >= maxStep) {
    return count;
  }

  uint16_t scaled = static_cast<uint16_t>(step) * count;
  uint8_t clearCount = static_cast<uint8_t>(scaled / maxStep);
  return clearCount > count ? count : clearCount;
}

static void renderPattern18RightWipe(uint8_t step, uint8_t maxStep,
                                     uint8_t brightness) {
  static const uint8_t TOP_UPPER_CLEAR[] = {78, 77, 76, 75, 74, 73};
  static const uint8_t TOP_LOWER_CLEAR[] = {55, 56, 57, 58, 59, 60, 61, 62};
  static const uint8_t ENDPOINT_UPPER_CLEAR[] = {54, 53, 52, 51, 50};
  static const uint8_t ENDPOINT_LOWER_CLEAR[] = {25, 26, 27, 28, 29};
  static const uint8_t MIDDLE_LOWER_CLEAR[] = {39, 38, 37, 36, 35};
  static const uint8_t BOTTOM_UPPER_CLEAR[] = {24, 23, 22, 21, 20, 19, 18,
                                               17, 16, 15, 14, 13, 12};
  static const uint8_t BOTTOM_LOWER_CLEAR[] = {1, 2, 3, 4, 5, 6,
                                               7, 8, 9, 10, 11};

  renderPattern18FullShape(brightness);

  renderPattern18ClearLedList(
      TOP_UPPER_CLEAR, sizeof(TOP_UPPER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(TOP_UPPER_CLEAR)));
  renderPattern18ClearLedList(
      TOP_LOWER_CLEAR, sizeof(TOP_LOWER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(TOP_LOWER_CLEAR)));
  renderPattern18ClearLedList(
      ENDPOINT_UPPER_CLEAR, sizeof(ENDPOINT_UPPER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(ENDPOINT_UPPER_CLEAR)));
  renderPattern18ClearLedList(
      ENDPOINT_LOWER_CLEAR, sizeof(ENDPOINT_LOWER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(ENDPOINT_LOWER_CLEAR)));
  renderPattern18ClearLedList(
      MIDDLE_LOWER_CLEAR, sizeof(MIDDLE_LOWER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(MIDDLE_LOWER_CLEAR)));
  renderPattern18ClearLedList(
      BOTTOM_UPPER_CLEAR, sizeof(BOTTOM_UPPER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(BOTTOM_UPPER_CLEAR)));
  renderPattern18ClearLedList(
      BOTTOM_LOWER_CLEAR, sizeof(BOTTOM_LOWER_CLEAR),
      pattern18ClearCountForStep(step, maxStep, sizeof(BOTTOM_LOWER_CLEAR)));

  // Keep the intended final marker clean and bright at the end of the wipe.
  if (step >= maxStep) {
    renderPattern18TerminalMarker(brightness);
  }
}

static void patternRightWipeTerminalBlink(uint32_t frame) {
  static const uint8_t FAST_BLINK_PULSES = 3;
  static const uint8_t FAST_ON_FRAMES = 3;
  static const uint8_t FAST_OFF_FRAMES = 2;
  static const uint8_t WIPE_FRAMES = 24;
  static const uint8_t MARKER_ON_FRAMES = 5;
  static const uint8_t MARKER_OFF_FRAMES = 4;
  static const uint8_t BLANK_FRAMES = 5;
  static const uint8_t WIPE_MAX_STEP = 23;
  static const uint8_t ON_BRIGHTNESS = 255;
  static const uint8_t OFF_RESIDUAL_BRIGHTNESS = 38;
  static const uint8_t WIPE_DIM_BRIGHTNESS = 120;

  const uint8_t introFrames = static_cast<uint8_t>(
      FAST_BLINK_PULSES * (FAST_ON_FRAMES + FAST_OFF_FRAMES));
  const uint8_t markerFrames = static_cast<uint8_t>(
      FAST_BLINK_PULSES * (MARKER_ON_FRAMES + MARKER_OFF_FRAMES));
  const uint8_t totalFrames = static_cast<uint8_t>(
      introFrames + WIPE_FRAMES + markerFrames + BLANK_FRAMES);

  uint8_t cycleFrame = static_cast<uint8_t>(frame % totalFrames);
  fillAllStrips(0, 0, 0, 0);

  if (cycleFrame < introFrames) {
    uint8_t pulseFrame = static_cast<uint8_t>(
        cycleFrame % (FAST_ON_FRAMES + FAST_OFF_FRAMES));
    uint8_t brightness = pulseFrame < FAST_ON_FRAMES
                             ? ON_BRIGHTNESS
                             : OFF_RESIDUAL_BRIGHTNESS;
    renderPattern18FullShape(brightness);
    return;
  }

  cycleFrame = static_cast<uint8_t>(cycleFrame - introFrames);
  if (cycleFrame < WIPE_FRAMES) {
    uint8_t step = static_cast<uint8_t>(
        (static_cast<uint16_t>(cycleFrame) * WIPE_MAX_STEP) /
        (WIPE_FRAMES - 1));
    bool brightBeat = ((cycleFrame / 2) & 0x01) == 0;
    uint8_t brightness = brightBeat ? ON_BRIGHTNESS : WIPE_DIM_BRIGHTNESS;
    renderPattern18RightWipe(step, WIPE_MAX_STEP, brightness);
    return;
  }

  cycleFrame = static_cast<uint8_t>(cycleFrame - WIPE_FRAMES);
  if (cycleFrame < markerFrames) {
    uint8_t pulseFrame = static_cast<uint8_t>(
        cycleFrame % (MARKER_ON_FRAMES + MARKER_OFF_FRAMES));
    uint8_t brightness = pulseFrame < MARKER_ON_FRAMES
                             ? ON_BRIGHTNESS
                             : OFF_RESIDUAL_BRIGHTNESS;
    renderPattern18TerminalMarker(brightness);
    return;
  }
}

static void patternRainDrop(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);

  static const uint8_t dropColumns[] = {1, 4, 8, 12, 14, 2, 6, 10, 13, 5};
  static const uint8_t dropOffsets[] = {0, 11, 23, 34, 46, 58, 69, 81, 93, 105};
  static const uint8_t DROP_COUNT = sizeof(dropColumns) / sizeof(dropColumns[0]);
  static const uint8_t ROW_UNITS = 8;
  static const uint8_t TRAIL_UNITS = 18;
  static const uint8_t CYCLE_UNITS = 126;

  uint8_t maxColumns = getMaxRowLength();
  if (maxColumns == 0) {
    return;
  }

  for (uint8_t pass = 0; pass < 2; pass++) {
    for (uint8_t drop = 0; drop < DROP_COUNT; drop++) {
      uint8_t headUnits =
          static_cast<uint8_t>((frame + dropOffsets[drop]) % CYCLE_UNITS);
      if (headUnits > (PCB_ROW_COUNT - 1) * ROW_UNITS + TRAIL_UNITS) {
        continue;
      }

      uint8_t baseColumn = dropColumns[drop] % maxColumns;
      for (uint8_t visualRow = 0; visualRow < PCB_ROW_COUNT; visualRow++) {
        uint8_t rowUnits = visualRow * ROW_UNITS;
        int16_t distance =
            static_cast<int16_t>(headUnits) - static_cast<int16_t>(rowUnits);

        if (distance < 0 || distance > TRAIL_UNITS) {
          continue;
        }

        uint8_t brightness =
            static_cast<uint8_t>(230 - (static_cast<uint16_t>(distance) * 185) /
                                           TRAIL_UNITS);
        bool isHead = distance <= 2;
        if ((pass == 0 && isHead) || (pass == 1 && !isHead)) {
          continue;
        }

        renderRainDropPixel(visualRow, baseColumn, brightness);
      }
    }
  }
}

static uint8_t triangleWave8(uint16_t value, uint16_t period) {
  if (period == 0) {
    return 0;
  }

  uint16_t phase = value % period;
  uint16_t halfPeriod = period / 2;
  if (halfPeriod == 0) {
    return 0;
  }

  if (phase >= halfPeriod) {
    phase = period - phase;
  }

  return static_cast<uint8_t>((phase * 255) / halfPeriod);
}

static uint8_t distanceFalloff(uint8_t distance, uint8_t radius) {
  if (distance >= radius || radius == 0) {
    return 0;
  }

  uint16_t brightness =
      255 - (static_cast<uint16_t>(distance) * 255) / radius;
  return static_cast<uint8_t>(brightness);
}

static uint8_t rowColumnDistanceFromCenter(uint8_t rowIdx, uint8_t column) {
  const RowLayout &row = PCB_ROWS[rowIdx];
  uint8_t center = row.length / 2;
  return column > center ? column - center : center - column;
}

static void patternNightRiderScanner(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);

  uint8_t maxColumns = getMaxRowLength();
  if (maxColumns <= 1) {
    return;
  }

  uint8_t period = static_cast<uint8_t>((maxColumns - 1) * 2);
  uint8_t phase = static_cast<uint8_t>((frame / 2) % period);
  uint8_t head = phase < maxColumns ? phase : static_cast<uint8_t>(period - phase);

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    uint8_t rowHead = mapRainColumnToRow(head, row, maxColumns);

    for (uint8_t column = 0; column < row.length; column++) {
      uint8_t distance = column > rowHead ? column - rowHead : rowHead - column;
      uint8_t brightness = distanceFalloff(distance, 6);
      if (brightness == 0) {
        continue;
      }

      if (rowIdx == 2 || rowIdx == 3) {
        uint16_t boostedBrightness = static_cast<uint16_t>(brightness) + 28;
        brightness = boostedBrightness > 255 ? 255 : boostedBrightness;
      }

      setAnimationPixelAllStrips(rowIdx, column, brightness);
    }
  }
}

static void patternPulseWave(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    for (uint8_t column = 0; column < row.length; column++) {
      uint8_t centerDistance = rowColumnDistanceFromCenter(rowIdx, column);
      uint16_t phase = static_cast<uint16_t>(frame * 5 + centerDistance * 34 +
                                             rowIdx * 11);
      uint8_t wave = triangleWave8(phase, 180);

      if (wave < 42) {
        continue;
      }

      uint8_t rowBoost = (rowIdx == 2 || rowIdx == 3) ? 32 : 0;
      uint16_t brightnessValue = 18 + wave + rowBoost;
      uint8_t brightness = brightnessValue > 255 ? 255 : brightnessValue;
      setAnimationPixelAllStrips(rowIdx, column, brightness);
    }
  }
}

static void patternAuroraRibbons(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);

  uint8_t maxColumns = getMaxRowLength();
  if (maxColumns == 0) {
    return;
  }

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    for (uint8_t column = 0; column < row.length; column++) {
      uint8_t visualColumn =
          maxColumns <= 1
              ? 0
              : static_cast<uint8_t>((static_cast<uint16_t>(column) *
                                      (maxColumns - 1)) /
                                     (row.length - 1));

      uint8_t slowWave = triangleWave8(
          static_cast<uint16_t>(frame * 2 + visualColumn * 21 + rowIdx * 17),
          210);
      uint8_t wideWave = triangleWave8(
          static_cast<uint16_t>(frame + visualColumn * 13 + rowIdx * 39), 160);
      uint8_t shimmer = triangleWave8(
          static_cast<uint16_t>(frame * 7 + visualColumn * 29 + rowIdx * 23),
          96);

      uint16_t brightness = 16 + slowWave / 2 + wideWave / 3 + shimmer / 5;
      if (rowIdx == 0 || rowIdx == 5) {
        brightness = (brightness * 4) / 5;
      }

      if (brightness < 28) {
        continue;
      }

      setAnimationPixelAllStrips(
          rowIdx, column,
          static_cast<uint8_t>(brightness > 255 ? 255 : brightness));
    }
  }
}

static void patternCenterOutAlert(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);
  uint8_t stage = static_cast<uint8_t>((frame / 3) % 12);

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    int center = row.length / 2;
    for (uint8_t column = 0; column < row.length; column++) {
      int distance = static_cast<int>(column) - center;
      if (distance < 0) {
        distance = -distance;
      }
      if (distance > stage || distance + 2 < stage) {
        continue;
      }
      uint8_t brightness = static_cast<uint8_t>(255 - (stage - distance) * 50);
      setAnimationPixelAllStrips(rowIdx, column, brightness);
    }
  }
}

static void patternStackFillAll(uint32_t frame) {
  fillAllStrips(0, 0, 0, 0);
  uint8_t totalColumns = 0;
  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    totalColumns += PCB_ROWS[rowIdx].length;
  }

  uint8_t activeCount = static_cast<uint8_t>((frame / 2) % (totalColumns + 16));
  uint8_t cursor = 0;
  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    for (uint8_t column = 0; column < row.length; column++) {
      if (cursor < activeCount) {
        uint8_t brightness = cursor + 6 > activeCount ? 255 : 105;
        setAnimationPixelAllStrips(rowIdx, column, brightness);
      }
      cursor++;
    }
  }
}


static uint8_t pattern19AbsDiff(uint8_t a, uint8_t b) {
  return a > b ? static_cast<uint8_t>(a - b) : static_cast<uint8_t>(b - a);
}

static uint8_t pattern19FoldScore(uint8_t rowIdx, uint8_t column) {
  const RowLayout &row = PCB_ROWS[rowIdx];
  uint8_t centerColumn = static_cast<uint8_t>((row.length - 1) / 2);
  uint8_t horizontalDistance = pattern19AbsDiff(column, centerColumn);

  // Six physical rows give two middle rows. Rows 2 and 3 are the fold hinge,
  // rows 1 and 4 are the next fold layer, and rows 0 and 5 are the outside.
  uint8_t verticalDistance;
  if (rowIdx <= 2) {
    verticalDistance = static_cast<uint8_t>(2 - rowIdx);
  } else {
    verticalDistance = static_cast<uint8_t>(rowIdx - 3);
  }

  return static_cast<uint8_t>((verticalDistance * 4U) + horizontalDistance);
}

static uint8_t pattern19FoldPixelBrightness(uint8_t score, uint8_t progress,
                                            uint8_t baseBrightness) {
  if (score <= progress) {
    return baseBrightness;
  }

  // Soft leading edge: the next layer is partially visible, so the fold feels
  // like a smooth opening panel instead of a hard one-step pop.
  uint8_t delta = static_cast<uint8_t>(score - progress);
  if (delta == 1) {
    return pattern3ScaleBrightness(baseBrightness, 96);
  }
  if (delta == 2) {
    return pattern3ScaleBrightness(baseBrightness, 28);
  }
  return 0;
}

static void renderPattern19FoldFrame(uint8_t progress, uint8_t brightness) {
  fillAllStrips(0, 0, 0, 0);

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    for (uint8_t column = 0; column < row.length; column++) {
      uint8_t score = pattern19FoldScore(rowIdx, column);
      uint8_t pixelBrightness =
          pattern19FoldPixelBrightness(score, progress, brightness);
      if (pixelBrightness > 0) {
        setAnimationPixelAllStrips(rowIdx, column, pixelBrightness);
      }
    }
  }
}

static void patternCenterFoldOutIn(uint32_t frame) {
  // Animation 19: fold out from the physical center of the six-row PCB, hold
  // briefly at full-open, then fold back into the center. It uses row/column
  // geometry instead of raw LED index order so the motion reads like a real
  // center hinge across the PCB.
  static const uint8_t MAX_FOLD_SCORE = 15;
  static const uint8_t FRAMES_PER_LAYER = 2;
  static const uint8_t OPEN_FRAMES =
      static_cast<uint8_t>((MAX_FOLD_SCORE + 1U) * FRAMES_PER_LAYER);
  static const uint8_t HOLD_OPEN_FRAMES = 7;
  static const uint8_t CLOSE_FRAMES = OPEN_FRAMES;
  static const uint8_t HOLD_CENTER_FRAMES = 5;
  static const uint8_t CYCLE_FRAMES = static_cast<uint8_t>(
      OPEN_FRAMES + HOLD_OPEN_FRAMES + CLOSE_FRAMES + HOLD_CENTER_FRAMES);

  uint8_t cycleFrame = static_cast<uint8_t>(frame % CYCLE_FRAMES);
  uint8_t progress = 0;
  uint8_t brightness = 255;

  if (cycleFrame < OPEN_FRAMES) {
    progress = static_cast<uint8_t>(cycleFrame / FRAMES_PER_LAYER);
  } else if (cycleFrame < OPEN_FRAMES + HOLD_OPEN_FRAMES) {
    progress = MAX_FOLD_SCORE;
    // Small full-open pulse so the fold peak is readable before closing.
    uint8_t holdFrame = static_cast<uint8_t>(cycleFrame - OPEN_FRAMES);
    brightness = (holdFrame % 2U == 0U) ? 255 : 190;
  } else if (cycleFrame < OPEN_FRAMES + HOLD_OPEN_FRAMES + CLOSE_FRAMES) {
    uint8_t closeFrame = static_cast<uint8_t>(
        cycleFrame - OPEN_FRAMES - HOLD_OPEN_FRAMES);
    uint8_t closeProgress = static_cast<uint8_t>(closeFrame / FRAMES_PER_LAYER);
    progress = closeProgress >= MAX_FOLD_SCORE
                   ? 0
                   : static_cast<uint8_t>(MAX_FOLD_SCORE - closeProgress);
  } else {
    // End with only the center hinge visible for a few frames, then repeat.
    progress = 0;
    brightness = 175;
  }

  renderPattern19FoldFrame(progress, brightness);
}

static uint8_t pattern20ClampBrightness(int16_t value) {
  if (value <= 0) {
    return 0;
  }
  if (value >= 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

static uint8_t pattern20HeartbeatGain(uint8_t beatFrame) {
  // Two visible ECG-style pulses per cycle: a smaller pre-beat followed by a
  // stronger main beat. The output is a brightness gain, not a time delay, so
  // the wave keeps moving while the beat flashes through it.
  if (beatFrame < 6) {
    return static_cast<uint8_t>(150 + (beatFrame * 12));
  }
  if (beatFrame < 12) {
    return static_cast<uint8_t>(222 - ((beatFrame - 6) * 14));
  }
  if (beatFrame >= 18 && beatFrame < 26) {
    return static_cast<uint8_t>(170 + ((beatFrame - 18) * 10));
  }
  if (beatFrame >= 26 && beatFrame < 36) {
    uint8_t fall = static_cast<uint8_t>((beatFrame - 26) * 13);
    return fall >= 245 ? 20 : static_cast<uint8_t>(245 - fall);
  }
  return 115;
}

static void patternWaveHeartbeat(uint32_t frame) {
  // Animation 20: a sine/heartbeat wave rendered over the real six-row PCB
  // stack. A smooth sine curve travels horizontally across the rows while a
  // soft heartbeat gain makes the wave pulse twice per cycle.
  static const uint8_t WAVE_FRAMES = 72;
  static const uint8_t HEARTBEAT_FRAMES = 54;
  static const float TWO_PI_F = 6.28318531f;
  static const float ROW_CENTER = 2.5f;
  static const float ROW_AMPLITUDE = 2.05f;
  static const float COLUMN_PHASE_STEP = 0.58f;
  static const float GLOW_WIDTH = 1.25f;
  static const uint8_t BASE_GLOW = 5;
  static const uint8_t CURVE_PEAK = 220;
  static const uint8_t CENTER_TRACE = 24;

  fillAllStrips(0, 0, 0, 0);

  uint8_t waveFrame = static_cast<uint8_t>(frame % WAVE_FRAMES);
  uint8_t beatFrame = static_cast<uint8_t>(frame % HEARTBEAT_FRAMES);
  float phase = (static_cast<float>(waveFrame) * TWO_PI_F) /
                static_cast<float>(WAVE_FRAMES);
  uint8_t heartbeatGain = pattern20HeartbeatGain(beatFrame);

  for (uint8_t rowIdx = 0; rowIdx < PCB_ROW_COUNT; rowIdx++) {
    const RowLayout &row = PCB_ROWS[rowIdx];
    for (uint8_t column = 0; column < row.length; column++) {
      float normalizedColumn = row.length <= 1
                                   ? 0.0f
                                   : (static_cast<float>(column) * 14.0f) /
                                         static_cast<float>(row.length - 1);
      float waveCenter = ROW_CENTER +
                         (ROW_AMPLITUDE * sinf(phase +
                                                (normalizedColumn * COLUMN_PHASE_STEP)));
      float distance = fabsf(static_cast<float>(rowIdx) - waveCenter);

      int16_t brightness = 0;
      if (distance < GLOW_WIDTH) {
        float shape = 1.0f - (distance / GLOW_WIDTH);
        float shapedPeak = static_cast<float>(CURVE_PEAK) * shape * shape;
        brightness = static_cast<int16_t>(BASE_GLOW +
            ((shapedPeak * static_cast<float>(heartbeatGain)) / 180.0f));
      } else {
        // A faint center trace keeps the heartbeat/wave readable on sparse
        // columns without filling the whole PCB solid.
        float centerDistance = fabsf(static_cast<float>(rowIdx) - ROW_CENTER);
        if (centerDistance < 0.65f) {
          brightness = CENTER_TRACE;
        }
      }

      // Add a very small travelling glint at the waveform crest so the user can
      // see direction without introducing random sparkle.
      uint8_t crestColumn = static_cast<uint8_t>((waveFrame / 4U) % 15U);
      uint8_t mappedColumn = static_cast<uint8_t>(normalizedColumn + 0.5f);
      if (mappedColumn == crestColumn && distance < 0.55f) {
        brightness += 38;
      }

      setAnimationPixelAllStrips(rowIdx, column,
                                 pattern20ClampBrightness(brightness));
    }
  }
}


static void renderAnimationFrame(uint8_t patternId, uint32_t frame) {
  switch (patternId) {
    case 1:
      patternBlueCircularFlow(frame);
      break;

    case 2:
      patternGreenSweep(frame);
      break;

    case 3:
      patternBluePerimeterFlow(frame);
      break;

    case 4:
      patternCenterOutAlert(frame);
      break;

    case 5:
      patternStackFillAll(frame);
      break;

    case 6:
      patternRainDrop(frame);
      break;

    case 7:
      patternNightRiderScanner(frame);
      break;

    case 8:
      patternPulseWave(frame);
      break;

    case 9:
      patternAuroraRibbons(frame);
      break;

    case 10:
      patternReturnFlowStack(frame);
      break;

    case 11:
      patternTopFlashCascade(frame);
      break;

    case 12:
      patternSequentialIntensityWash(frame);
      break;

    case 13:
      patternSingleRowReturnStack(frame);
      break;

    case 14:
      patternThreeZoneFlicker(frame);
      break;

    case 15:
      patternStagedSmoothFlicker(frame);
      break;

    case 16:
      patternMirrorBridgeReturn(frame);
      break;

    case 17:
      patternRandomizedShowMix(frame);
      break;

    case 18:
      patternRightWipeTerminalBlink(frame);
      break;

    case 19:
      patternCenterFoldOutIn(frame);
      break;

    case 20:
      patternWaveHeartbeat(frame);
      break;

    default:
      patternBlueCircularFlow(frame);
      break;
  }
}

static bool isBlinkOn(unsigned long nowMs) {
  unsigned long cycleMs = TURN_BLINK_ON_MS + TURN_BLINK_OFF_MS;
  if (cycleMs == 0) {
    return true;
  }

  unsigned long phaseMs = nowMs % cycleMs;
  return phaseMs < TURN_BLINK_ON_MS;
}

static void seedPulseTracker(PulseTracker &tracker, bool level,
                             unsigned long nowMs) {
  tracker.initialized = true;
  tracker.lastLevel = level;
  tracker.highSinceMs = level ? nowMs : 0;
  tracker.lastValidPulseMs = 0;
}

static void updatePulseTracker(PulseTracker &tracker, bool rawLevel,
                               bool brakeActive, unsigned long nowMs) {
  if (!tracker.initialized) {
    seedPulseTracker(tracker, rawLevel, nowMs);
    return;
  }

  if (rawLevel != tracker.lastLevel) {
    if (rawLevel) {
      tracker.highSinceMs = nowMs;
    } else if (tracker.highSinceMs != 0) {
      unsigned long highTimeMs = nowMs - tracker.highSinceMs;
      if (highTimeMs >= TURN_PULSE_MIN_HIGH_MS &&
          highTimeMs <= TURN_PULSE_MAX_HIGH_MS) {
        tracker.lastValidPulseMs = nowMs;
      }
      tracker.highSinceMs = 0;
    }

    tracker.lastLevel = rawLevel;
  }

  if (rawLevel && !brakeActive && tracker.highSinceMs != 0) {
    unsigned long highTimeMs = nowMs - tracker.highSinceMs;
    if (highTimeMs >= TURN_PULSE_MIN_HIGH_MS &&
        highTimeMs <= TURN_PULSE_MAX_HIGH_MS) {
      tracker.lastValidPulseMs = nowMs;
    }
  }
}

static bool pulseIsActive(const PulseTracker &tracker, unsigned long nowMs) {
  return tracker.lastValidPulseMs != 0 &&
         (nowMs - tracker.lastValidPulseMs) <= TURN_ACTIVITY_HOLD_MS;
}

static void startBlePattern(uint8_t patternId, const char *label) {
  if (!isValidBlePattern(patternId)) {
    Serial.print("BLE animation rejected: ");
    Serial.println(patternId);
    return;
  }

  selectedBlePattern = patternId;
  blePattern = patternId;
  bleActive = true;
  bleSolidBlueMode = false;
  bleDisplayDirty = true;
  bleInterrupted = false;
  animationFrame = 0;
  lastAnimationFrameMs = 0;
  Serial.print("BLE cmd: ");
  Serial.println(label);
}

static void startBleSolidBlue() {
  bleActive = true;
  bleSolidBlueMode = true;
  bleDisplayDirty = true;
  bleInterrupted = false;
  animationFrame = 0;
  lastAnimationFrameMs = 0;
  Serial.println("BLE cmd: ON solid blue");
}

static void processBleCommands() {
  if (bleCommandQueue == NULL) {
    return;
  }

  BleCommand cmd = {BLE_COMMAND_NONE, 0};
  while (xQueueReceive(bleCommandQueue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case BLE_COMMAND_OFF:
        bleActive = false;
        bleSolidBlueMode = false;
        bleDisplayDirty = true;
        bleInterrupted = false;
        animationFrame = 0;
        Serial.println("BLE cmd: OFF");
        break;

      case BLE_COMMAND_ON:
        startBleSolidBlue();
        break;

      case BLE_COMMAND_RUN_MODE:
        bleActive = false;
        bleSolidBlueMode = false;
        bleDisplayDirty = true;
        bleInterrupted = false;
        setRunningLightMode(static_cast<RunningLightMode>(cmd.value), true);
        Serial.print("BLE cmd: RUN,");
        Serial.println(runningLightModeName(runningLightMode));
        break;

      case BLE_COMMAND_ANIMATION:
        setSelectedBlePattern(cmd.value, true);
        startBlePattern(selectedBlePattern, blePatternName(selectedBlePattern));
        break;

      case BLE_COMMAND_COLOR:
        setAnimationColor(cmd.red, cmd.green, cmd.blue, cmd.white, true);
        if (bleActive && !bleInterrupted) {
          animationFrame = 0;
          lastAnimationFrameMs = 0;
        }
        Serial.println("BLE cmd: COLOR");
        break;

      case BLE_COMMAND_NONE:
      default:
        Serial.println("BLE cmd: ignored");
        break;
    }
  }
}

static void updateStatusLeds(unsigned long nowMs) {
  if ((nowMs - lastHeartbeatToggleMs) >= STATUS_LED_HEARTBEAT_MS) {
    lastHeartbeatToggleMs = nowMs;
    heartbeatState = !heartbeatState;
    digitalWrite(STATUS_LED_HEARTBEAT_GPIO, heartbeatState ? HIGH : LOW);
  }
}

static void ensureBleAdvertising() {
  if (bleAdvertisingRestartRequested) {
    bleAdvertisingRestartRequested = false;
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted");
  }
}

class TailLightServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    Serial.println("BLE client disconnected");

    BleCommand stopCmd = {BLE_COMMAND_OFF, 0};
    if (bleCommandQueue != NULL) {
      xQueueSend(bleCommandQueue, &stopCmd, 0);
    }
    bleAdvertisingRestartRequested = true;
  }
};

static bool parseByteToken(const String &value, int &startIndex,
                           uint8_t &parsedValue) {
  int endIndex = value.indexOf(',', startIndex);
  String token;
  if (endIndex < 0) {
    token = value.substring(startIndex);
    startIndex = value.length();
  } else {
    token = value.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
  }

  if (token.length() == 0) {
    return false;
  }

  for (uint16_t i = 0; i < token.length(); i++) {
    if (!isDigit(token[i])) {
      return false;
    }
  }

  int valueInt = token.toInt();
  if (valueInt < 0 || valueInt > 255) {
    return false;
  }

  parsedValue = static_cast<uint8_t>(valueInt);
  return true;
}

static bool parseCustomColorCommand(const String &value, int startIndex,
                                    BleCommand &cmd) {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t white;

  if (!parseByteToken(value, startIndex, red) ||
      !parseByteToken(value, startIndex, green) ||
      !parseByteToken(value, startIndex, blue) ||
      !parseByteToken(value, startIndex, white) ||
      startIndex != value.length()) {
    return false;
  }

  cmd.type = BLE_COMMAND_COLOR;
  cmd.red = red;
  cmd.green = green;
  cmd.blue = blue;
  cmd.white = white;
  return true;
}

static bool setNamedColorCommand(const String &name, BleCommand &cmd) {
  cmd.type = BLE_COMMAND_COLOR;
  cmd.red = 0;
  cmd.green = 0;
  cmd.blue = 0;
  cmd.white = 0;

  if (name == "RED") {
    cmd.red = 255;
  } else if (name == "GREEN") {
    cmd.green = 255;
  } else if (name == "BLUE") {
    cmd.blue = 255;
  } else if (name == "WHITE") {
    cmd.white = 255;
  } else if (name == "WARMWHITE" || name == "WARM_WHITE") {
    cmd.red = 255;
    cmd.green = 120;
    cmd.white = 120;
  } else if (name == "AMBER") {
    cmd.red = COLOR_AMBER_R;
    cmd.green = COLOR_AMBER_G;
    cmd.blue = COLOR_AMBER_B;
    cmd.white = COLOR_AMBER_W;
  } else if (name == "ORANGE") {
    cmd.red = 255;
    cmd.green = 90;
  } else if (name == "YELLOW") {
    cmd.red = 255;
    cmd.green = 180;
  } else if (name == "PURPLE") {
    cmd.red = 130;
    cmd.blue = 255;
  } else if (name == "PINK") {
    cmd.red = 255;
    cmd.green = 20;
    cmd.blue = 80;
  } else if (name == "MAGENTA") {
    cmd.red = 255;
    cmd.blue = 180;
  } else if (name == "CYAN") {
    cmd.green = 180;
    cmd.blue = 255;
  } else if (name == "ICEBLUE" || name == "ICE_BLUE") {
    cmd.green = 90;
    cmd.blue = 255;
    cmd.white = 35;
  } else if (name == "OFF" || name == "BLACK") {
    cmd.red = 0;
    cmd.green = 0;
    cmd.blue = 0;
    cmd.white = 0;
  } else {
    cmd.type = BLE_COMMAND_NONE;
    return false;
  }

  return true;
}

static bool parseBleCommand(const String &value, BleCommand &cmd) {
  cmd.type = BLE_COMMAND_NONE;
  cmd.value = 0;
  cmd.red = 0;
  cmd.green = 0;
  cmd.blue = 0;
  cmd.white = 0;

  if (value == "OFF") {
    cmd.type = BLE_COMMAND_OFF;
    return true;
  }

  if (value == "ON") {
    cmd.type = BLE_COMMAND_ON;
    return true;
  }

  if (value == "RUN,ALL") {
    cmd.type = BLE_COMMAND_RUN_MODE;
    cmd.value = RUNNING_MODE_ALL_LINES;
    return true;
  }

  if (value == "RUN,OUTER") {
    cmd.type = BLE_COMMAND_RUN_MODE;
    cmd.value = RUNNING_MODE_OUTER_LINES;
    return true;
  }

  if (value == "RUN,MIDDLE") {
    cmd.type = BLE_COMMAND_RUN_MODE;
    cmd.value = RUNNING_MODE_MIDDLE_LINE;
    return true;
  }

  if (value.startsWith("ANIM,")) {
    String patternValue = value.substring(5);
    if (patternValue.length() == 0) {
      return false;
    }

    for (uint16_t i = 0; i < patternValue.length(); i++) {
      if (!isDigit(patternValue[i])) {
        return false;
      }
    }

    int patternId = patternValue.toInt();
    if (patternId >= BLE_ANIMATION_MIN_ID &&
        patternId <= BLE_ANIMATION_MAX_ID) {
      cmd.type = BLE_COMMAND_ANIMATION;
      cmd.value = static_cast<uint8_t>(patternId);
      return true;
    }
  }

  if (value.startsWith("COLOR,")) {
    String colorValue = value.substring(6);
    if (colorValue.startsWith("CUSTOM,")) {
      return parseCustomColorCommand(value, 13, cmd);
    }
    if (colorValue.startsWith("RGBW,")) {
      return parseCustomColorCommand(value, 11, cmd);
    }
    return setNamedColorCommand(colorValue, cmd);
  }

  if (value.startsWith("RGBW,")) {
    return parseCustomColorCommand(value, 5, cmd);
  }

  return false;
}

class TailLightCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();
    value.trim();
    value.toUpperCase();
    value.replace(" ", "");
    if (value.length() == 0) {
      return;
    }

    BleCommand cmd;
    if (!parseBleCommand(value, cmd)) {
      Serial.print("BLE cmd: unknown ");
      Serial.println(value);
      return;
    }

    if (bleCommandQueue != NULL) {
      xQueueSend(bleCommandQueue, &cmd, 0);
    }
  }
};

static void initBle() {
  bleCommandQueue = xQueueCreate(8, sizeof(BleCommand));

  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new TailLightServerCallbacks());

  BLEService *service = bleServer->createService(BLE_SERVICE_UUID);
  BLECharacteristic *characteristic =
      service->createCharacteristic(BLE_CHARACTERISTIC_UUID,
                                    BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR);
  characteristic->setCallbacks(new TailLightCharacteristicCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE initialised and advertising");
}

static void renderMode(RenderMode mode, bool leftTurnActive,
                       bool rightTurnActive, bool blinkOn,
                       unsigned long nowMs) {
  switch (mode) {
    case RENDER_MODE_BRAKE:
      renderBrakeBaseFrame(nowMs);
      break;

    case RENDER_MODE_TURN:
      renderTurnFrame(leftTurnActive, rightTurnActive, false, blinkOn, nowMs);
      break;

    case RENDER_MODE_TURN_BRAKE:
      renderTurnFrame(leftTurnActive, rightTurnActive, true, blinkOn, nowMs);
      break;

    case RENDER_MODE_HAZARD:
      renderHazardFrame(false, blinkOn, nowMs);
      break;

    case RENDER_MODE_HAZARD_BRAKE:
      renderHazardFrame(true, blinkOn, nowMs);
      break;

    case RENDER_MODE_BLE:
      if (bleSolidBlueMode) {
        fillAllStrips(BLE_ON_BLUE_R, BLE_ON_BLUE_G, BLE_ON_BLUE_B,
                      BLE_ON_BLUE_W);
      } else {
        renderAnimationFrame(blePattern, animationFrame);
        animationFrame++;
      }
      lastAnimationFrameMs = nowMs;
      break;

    case RENDER_MODE_RUNNING:
    default:
      renderRunningFrame();
      break;
  }

  showAllStrips();
}

static void recoverLedOutputIfDue(unsigned long nowMs, RenderMode targetMode,
                                  bool leftTurnActive, bool rightTurnActive,
                                  bool blinkOn) {
  if ((nowMs - lastLedRecoveryRefreshMs) < LED_RECOVERY_REFRESH_INTERVAL_MS) {
    return;
  }

  if ((nowMs - lastLedRecoveryReinitMs) >= LED_RECOVERY_REINIT_INTERVAL_MS) {
    beginAllStrips();
    lastLedRecoveryReinitMs = nowMs;
  }

  renderMode(targetMode, leftTurnActive, rightTurnActive, blinkOn, nowMs);
  lastRenderedMode = targetMode;
  lastBlinkOn = blinkOn;
  lastLedRecoveryRefreshMs = nowMs;
}

void setup() {
  Serial.begin(115200);
  delay(50);
  initWatchdog();

  // Use high-impedance inputs so the external signal-conditioning stage
  // controls the line voltage without the ESP32's internal pulldown loading it.
  pinMode(BTN_BRAKE_GPIO, INPUT);
  pinMode(BTN_TURN_LEFT_GPIO, INPUT);
  pinMode(BTN_TURN_RIGHT_GPIO, INPUT);

  pinMode(STATUS_LED_READY_GPIO, OUTPUT);
  pinMode(STATUS_LED_HEARTBEAT_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_READY_GPIO, HIGH);
  digitalWrite(STATUS_LED_HEARTBEAT_GPIO, HIGH);
  lastHeartbeatToggleMs = millis();

  initDebouncedInput(brakeInput, BTN_BRAKE_GPIO);
  initDebouncedInput(leftInput, BTN_TURN_LEFT_GPIO);
  initDebouncedInput(rightInput, BTN_TURN_RIGHT_GPIO);
  initSettings();

  beginAllStrips();
  watchdogDelay(LED_STARTUP_SETTLE_MS);
  fillAllStrips(0, 0, 0, 0);
  showAllStrips();
  watchdogDelay(20);
  playStartupLightCheck();
  renderRunningFrame();
  showAllStrips();

  initBle();

  lastBlinkOn = isBlinkOn(millis());
  lastAnimationFrameMs = millis();
  lastTurnSweepFrameMs = millis();
  startupRefreshUntilMs = millis() + LED_STARTUP_REFRESH_WINDOW_MS;
  lastStartupRefreshMs = 0;
  lastLedRecoveryRefreshMs = millis();
  lastLedRecoveryReinitMs = millis();

  Serial.println("Tail light Arduino firmware started");
  for (uint8_t stripId = 0; stripId < NUM_STRIPS; stripId++) {
    Serial.print(stripNames[stripId]);
    Serial.print(" on GPIO ");
    Serial.println(stripGpios[stripId]);
  }
}

void loop() {
  unsigned long nowMs = millis();
  feedWatchdog();

  updateStatusLeds(nowMs);
  ensureBleAdvertising();
  processBleCommands();

  updateDebouncedInput(brakeInput, nowMs);
  updateDebouncedInput(leftInput, nowMs);
  updateDebouncedInput(rightInput, nowMs);

  bool brake = brakeInput.stableState;
  bool leftRaw = leftInput.stableState;
  bool rightRaw = rightInput.stableState;

  updatePulseTracker(leftTracker, leftRaw, brake, nowMs);
  updatePulseTracker(rightTracker, rightRaw, brake, nowMs);

  bool leftTurn = pulseIsActive(leftTracker, nowMs);
  bool rightTurn = pulseIsActive(rightTracker, nowMs);
  bool hazard = leftTurn && rightTurn;
  bool anyTurn = leftTurn || rightTurn;
  bool blinkOn = isBlinkOn(nowMs);
  uint8_t turnSweepColumns = getTurnSweepColumns(nowMs);

  if (brake && !lastBrakeSignal) {
    brakeSweepActive = true;
    brakeSweepStartMs = nowMs;
    lastBrakeSweepProgress = 65535;
    lastBrakeSweepFrameMs = 0;
  } else if (!brake) {
    brakeSweepActive = false;
    lastBrakeSweepProgress = 65535;
    lastBrakeSweepFrameMs = 0;
  }
  lastBrakeSignal = brake;

  uint16_t brakeSweepProgress = getBrakeSweepFinalProgress();
  if (brakeSweepActive) {
    brakeSweepProgress = getBrakeSweepProgress(nowMs);
  }

  if ((brake || anyTurn) && bleActive && !bleInterrupted) {
    bleInterrupted = true;
    Serial.println("BLE animation interrupted by signal");
  }

  RenderMode targetMode = RENDER_MODE_RUNNING;
  if (hazard && brake) {
    targetMode = RENDER_MODE_HAZARD_BRAKE;
  } else if (hazard) {
    targetMode = RENDER_MODE_HAZARD;
  } else if (anyTurn && brake) {
    targetMode = RENDER_MODE_TURN_BRAKE;
  } else if (anyTurn) {
    targetMode = RENDER_MODE_TURN;
  } else if (brake) {
    targetMode = RENDER_MODE_BRAKE;
  } else if (bleActive && !bleInterrupted) {
    targetMode = RENDER_MODE_BLE;
  }

  bool blinkDrivenMode = targetMode == RENDER_MODE_TURN ||
                         targetMode == RENDER_MODE_TURN_BRAKE ||
                         targetMode == RENDER_MODE_HAZARD ||
                         targetMode == RENDER_MODE_HAZARD_BRAKE;
  bool turnSweepDrivenMode =
      (targetMode == RENDER_MODE_TURN ||
       targetMode == RENDER_MODE_TURN_BRAKE) &&
      blinkOn;
  bool brakeDrivenMode = targetMode == RENDER_MODE_BRAKE ||
                         targetMode == RENDER_MODE_TURN_BRAKE ||
                         targetMode == RENDER_MODE_HAZARD_BRAKE;

  bool shouldRender = false;
  if (targetMode != lastRenderedMode) {
    shouldRender = true;
  } else if (blinkDrivenMode && blinkOn != lastBlinkOn) {
    shouldRender = true;
  } else if (turnSweepDrivenMode && turnSweepColumns != lastTurnSweepColumns &&
             (nowMs - lastTurnSweepFrameMs) >= TURN_SWEEP_FRAME_INTERVAL_MS) {
    shouldRender = true;
  } else if (brakeDrivenMode && brakeSweepActive &&
             brakeSweepProgress != lastBrakeSweepProgress &&
             (lastBrakeSweepFrameMs == 0 ||
              brakeSweepProgress >= getBrakeSweepFinalProgress() ||
              (nowMs - lastBrakeSweepFrameMs) >=
                  BRAKE_SWEEP_FRAME_INTERVAL_MS)) {
    shouldRender = true;
  } else if (runningLightModeDirty && targetMode != RENDER_MODE_BLE) {
    shouldRender = true;
  } else if (bleDisplayDirty) {
    shouldRender = true;
  } else if (targetMode == RENDER_MODE_BLE &&
             !bleSolidBlueMode &&
             (nowMs - lastAnimationFrameMs) >= ANIM_FRAME_INTERVAL_MS) {
    shouldRender = true;
  } else if (nowMs <= startupRefreshUntilMs &&
             (lastStartupRefreshMs == 0 ||
              (nowMs - lastStartupRefreshMs) >= LED_STARTUP_REFRESH_INTERVAL_MS)) {
    shouldRender = true;
  }

  if (shouldRender) {
    renderMode(targetMode, leftTurn, rightTurn, blinkOn, nowMs);
    lastRenderedMode = targetMode;
    lastBlinkOn = blinkOn;
    lastLedRecoveryRefreshMs = nowMs;
    lastTurnSweepColumns = turnSweepColumns;
    if (turnSweepDrivenMode) {
      lastTurnSweepFrameMs = nowMs;
    }
    if (brakeDrivenMode && brakeSweepActive) {
      lastBrakeSweepProgress = brakeSweepProgress;
      lastBrakeSweepFrameMs = nowMs;
      if (brakeSweepProgress >= getBrakeSweepFinalProgress()) {
        brakeSweepActive = false;
      }
    }
    if (nowMs <= startupRefreshUntilMs) {
      lastStartupRefreshMs = nowMs;
    }
    runningLightModeDirty = false;
    bleDisplayDirty = false;
  }

  recoverLedOutputIfDue(nowMs, targetMode, leftTurn, rightTurn, blinkOn);
  feedWatchdog();
  delay(1);
  feedWatchdog();
}
