/* Perfume filling gun - Arduino Nano ATmega328P
   OLED I2C: SDA=A4, SCL=A5. U8g2 library required.
   Encoder: CLK=D3, DT=D4, SW=D5. Trigger NO=D6, COM=GND.
   YFS401 pulse output=D2. MOSFET driver control=D9.
   Regulated 5V powers Nano and compatible peripherals; all grounds common.
   Battery monitoring intentionally disabled. Calibrate FLOW_PULSES_PER_ML
   using the actual liquid and pump before relying on preset fill volumes.
   Animated boot logo comes from BootLogo.h (keep it in the sketch folder).
*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "BootLogo.h"

#define OLED_SH1106 1  // 0 for SSD1306
#if OLED_SH1106
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif

const uint8_t OLED_ADDRESS = 0x3C; // Try 0x3D if your module uses it
const uint8_t ENC_A = 3, ENC_B = 4, ENC_SW = 5, TRIGGER = 6;
const uint8_t FLOW_SENSOR_PIN = 2; // YFS401 yellow pulse output; external interrupt INT0
const uint8_t PUMP_PIN = 9;        // MOSFET driver input, active HIGH (verify your driver)

// Common nominal YFS401 calibration is around 5880 pulses/litre, but the exact
// value varies by sensor, liquid, flow rate and installation. CALIBRATE THIS.
const float FLOW_PULSES_PER_ML = 5.88f;
const unsigned long FLOW_START_GRACE_MS = 2500UL;
const unsigned long FLOW_STALL_TIMEOUT_MS = 1500UL;
const int8_t ENCODER_DIRECTION = -1; // Reversed: clockwise increases on this encoder
const int8_t COUNTS_PER_DETENT = 4; // Some encoders need 2
const uint16_t MIN_ML = 1, MAX_ML = 500;
const uint8_t MAX_PRESETS = 8;
uint16_t PRESETS[MAX_PRESETS] = {8, 10, 20, 50, 100};
uint8_t PRESET_COUNT = 5;
uint8_t brightness = 70, editBrightness = 70;
uint8_t settingsSelected = 0;
const char *notice = "";
unsigned long noticeUntil = 0;
// Fixed EEPROM layout: magic, version, brightness, count, 8 uint16 presets, CRC.
// CRC and range checks reject incomplete or invalid saved configurations.
const uint8_t SAVE_SIZE = 22;
uint8_t configCRC(const uint8_t *data, uint8_t count) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < count; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
void saveSettings() {
  uint8_t data[SAVE_SIZE] = {0x50, 0x47, 2, brightness, PRESET_COUNT};
  for (uint8_t i = 0; i < MAX_PRESETS; i++) {
    data[5 + i * 2] = PRESETS[i] & 0xFF;
    data[6 + i * 2] = PRESETS[i] >> 8;
  }
  data[21] = configCRC(data, 21);
  // Invalidate first, then commit magic last so an interrupted save is rejected.
  EEPROM.update(0, 0);
  for (uint8_t i = 1; i < SAVE_SIZE; i++) EEPROM.update(i, data[i]);
  EEPROM.update(0, data[0]);
}
void loadSettings() {
  uint8_t data[SAVE_SIZE];
  for (uint8_t i = 0; i < SAVE_SIZE; i++) data[i] = EEPROM.read(i);
  if (data[0] != 0x50 || data[1] != 0x47 || (data[2] != 1 && data[2] != 2) ||
      data[3] < 10 || data[3] > 100 || data[4] < 1 || data[4] > MAX_PRESETS ||
      data[21] != configCRC(data, 21)) return;
  if (data[2] == 1) {
    // One-time preset replacement for older firmware; retain saved brightness.
    brightness = data[3];
    saveSettings();
    return;
  }
  for (uint8_t i = 0; i < data[4]; i++) {
    uint16_t value = (uint16_t)data[5 + i * 2] | ((uint16_t)data[6 + i * 2] << 8);
    if (value < MIN_ML || value > MAX_ML) return;
    if (i && value <= ((uint16_t)data[3 + i * 2] | ((uint16_t)data[4 + i * 2] << 8))) return;
  }
  brightness = data[3];
  PRESET_COUNT = data[4];
  for (uint8_t i = 0; i < MAX_PRESETS; i++)
    PRESETS[i] = (uint16_t)data[5 + i * 2] | ((uint16_t)data[6 + i * 2] << 8);
}

// Optional battery display only. These values assume a conventional 3S pack.
// Divider example: battery+ -> 33k -> A0 -> 10k -> GND.
// Enable ONLY after checking the divider and measuring the Nano's actual VCC.
const bool BATTERY_ENABLED = false;
const float ADC_REFERENCE_V = 5.00;
const float DIVIDER_RATIO = (33000.0 + 10000.0) / 10000.0;
const float BATTERY_EMPTY_V = 9.6, BATTERY_FULL_V = 12.6;
// Linear voltage estimate, not an accurate state-of-charge measurement.

enum Screen : uint8_t { MENU, CUSTOM, READY, FILLING, PAUSED, COMPLETE, STOPPED,
                        SETTINGS, BRIGHTNESS, ADD_PRESET, ABOUT, RESET_PRESETS };
// Explicit prototype prevents Arduino from generating one before Screen exists.
void finishFill(Screen result, unsigned long now);
Screen screen = MENU;
uint8_t selected = 0;
uint16_t targetML = 10, customML = 45, editML = 45;
uint32_t deliveredUL = 0;
volatile uint32_t flowPulseCount = 0;
uint32_t lastObservedFlowPulses = 0;
uint32_t targetPulseCount = 0;
unsigned long fillStartedAt = 0, lastFlowPulseAt = 0;
unsigned long resultAt = 0, lastFrame = 0;
unsigned long lastTurn = 0, lastBattery = 0;
float batteryV = 0;
int batteryPercent = -1;
bool ignoreStartTrigger = false;
const unsigned long TRIGGER_HOLD_MS = 1000;
const unsigned long BOOT_DURATION_MS = 2200;
unsigned long bootStartedAt = 0;
bool bootActive = true;

volatile int16_t encoderTicks = 0;
volatile uint8_t encoderPrevious = 0;

// D2 is INT0 on the classic Nano. Keep the ISR short; do calculations in loop().
void flowPulseISR() {
  if (flowPulseCount < 0xFFFFFFFFUL) flowPulseCount++;
}

uint32_t readFlowPulses() {
  uint32_t count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = flowPulseCount; }
  return count;
}

void resetFlowPulses() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { flowPulseCount = 0; }
  lastObservedFlowPulses = 0;
}

void pumpOn() {
  digitalWrite(PUMP_PIN, HIGH);
}

void pumpOff() {
  digitalWrite(PUMP_PIN, LOW);
}

// Pin-change interrupts capture rotation even while OLED pages are transmitted.
// This ISR is specifically for the classic ATmega328P Nano, D3 and D4.
ISR(PCINT2_vect) {
  const int8_t table[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t now = (PIND >> 3) & 3;
  int8_t delta = table[(encoderPrevious << 2) | now];
  encoderPrevious = now;
  if (encoderTicks > -2000 && encoderTicks < 2000) encoderTicks += delta;
}

struct Button {
  uint8_t pin, debounceMs;
  bool rawDown, down, armed, longSent;
  bool pressed, clicked, held;
  unsigned long changedAt, downAt;
  void begin(uint8_t p, uint8_t debounce) {
    pin = p;
    debounceMs = debounce;
    pinMode(pin, INPUT_PULLUP);
    rawDown = down = (digitalRead(pin) == LOW);
    armed = !down; // A button held at boot must be released first.
    pressed = clicked = held = longSent = false;
    changedAt = downAt = millis();
  }
  void update(unsigned long now) {
    pressed = clicked = held = false;
    bool reading = digitalRead(pin) == LOW;
    if (reading != rawDown) { rawDown = reading; changedAt = now; }
    if (now - changedAt >= debounceMs && down != rawDown) {
      down = rawDown;
      if (down) {
        downAt = now;
        longSent = false;
        if (armed) pressed = true;
      } else {
        if (armed && !longSent) clicked = true;
        armed = true;
      }
    }
    if (pin == TRIGGER && down && armed && !longSent && now - downAt >= TRIGGER_HOLD_MS) {
      held = true;
      longSent = true;
    }
  }
};
Button knob, trigger;

int16_t readEncoder() {
  static int16_t remainder = 0;
  int16_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ticks = encoderTicks; encoderTicks = 0; }
  remainder += ticks;
  int16_t detents = remainder / COUNTS_PER_DETENT;
  remainder %= COUNTS_PER_DETENT;
  return detents * ENCODER_DIRECTION;
}

void startFill(unsigned long now) {
  pumpOff();
  resetFlowPulses();
  deliveredUL = 0;
  targetPulseCount = (uint32_t)((float)targetML * FLOW_PULSES_PER_ML + 0.5f);
  if (targetPulseCount == 0) targetPulseCount = 1;
  fillStartedAt = now;
  lastFlowPulseAt = now;
  screen = FILLING;
  ignoreStartTrigger = true; // Starting press/release must not pause the new fill.
  Serial.println(F("Fill started"));
  pumpOn();
}

void finishFill(Screen result, unsigned long now) {
  pumpOff(); // Every finish path turns the pump off first.
  uint32_t pulses = readFlowPulses();
  Serial.print(F("Fill ended; result="));
  Serial.print((uint8_t)result);
  Serial.print(F("; pulses="));
  Serial.print(pulses);
  Serial.print(F("; measured_mL="));
  Serial.println((float)pulses / FLOW_PULSES_PER_ML, 2);
  screen = result;
  resultAt = now;
}

void updateBattery(unsigned long now) {
  if (!BATTERY_ENABLED || now - lastBattery < 500) return;
  lastBattery = now;
  float sample = analogRead(A0) * ADC_REFERENCE_V / 1023.0 * DIVIDER_RATIO;
  batteryV = batteryV == 0 ? sample : batteryV * 0.9 + sample * 0.1;
  batteryPercent = constrain((int)((batteryV - BATTERY_EMPTY_V) * 100.0 /
                                  (BATTERY_FULL_V - BATTERY_EMPTY_V)), 0, 100);
}

void centered(const char *s, uint8_t baseline) {
  oled.drawStr((128 - oled.getStrWidth(s)) / 2, baseline, s);
}

void drawHeader(const char *title) {
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(0, 8, title);
  if (BATTERY_ENABLED) {
    oled.drawFrame(108, 1, 18, 8);
    oled.drawBox(126, 3, 2, 4);
    if (batteryPercent < 0) oled.drawStr(114, 8, "?");
    else {
      uint8_t bars = batteryPercent == 0 ? 0 : (batteryPercent + 24) / 25;
      for (uint8_t i = 0; i < bars; i++) oled.drawBox(110 + i * 4, 3, 3, 4);
      if (batteryPercent <= 15) oled.drawStr(98, 8, "!");
    }
  }
  oled.drawHLine(0, 11, 128);
}

void footer(const char *text) {
  oled.setFont(u8g2_font_5x7_tf);
  centered(text, 63);
}

// Numeric-only large font keeps flash use lower than a full alphabet font.
void drawVolume(uint16_t volume, uint8_t baseline) {
  char number[6];
  snprintf(number, sizeof(number), "%u", (unsigned)volume);
  oled.setFont(u8g2_font_helvB24_tn);
  int numberWidth = oled.getStrWidth(number);
  oled.setFont(u8g2_font_6x10_tf);
  int unitWidth = oled.getStrWidth("mL");
  int x = (128 - numberWidth - 5 - unitWidth) / 2;
  oled.setFont(u8g2_font_helvB24_tn);
  oled.drawStr(x, baseline, number);
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(x + numberWidth + 5, baseline, "mL");
}

void drawScreen() {
  char text[28];
  const char *title = screen == MENU ? "FILL VOLUME" :
                      screen == CUSTOM ? "CUSTOM" :
                      screen == SETTINGS ? "SETTINGS" :
                      screen == BRIGHTNESS ? "BRIGHTNESS" :
                      screen == ADD_PRESET ? "ADD PRESET" :
                      screen == ABOUT ? "ABOUT" :
                      screen == RESET_PRESETS ? "RESET PRESETS" :
                      screen == READY ? "READY" :
                      screen == FILLING ? "FILLING" :
                      screen == PAUSED ? "PAUSED" :
                      screen == COMPLETE ? "DONE" : "STOPPED";
  oled.firstPage();
  do {
    drawHeader(title);
    if (screen == SETTINGS) {
      const char *items[] = {"Brightness", "Add preset", "Reset presets", "About", "Back"};
      uint8_t first = constrain((int)settingsSelected - 1, 0, 2);
      oled.setFont(u8g2_font_6x10_tf);
      for (uint8_t row = 0; row < 3; row++) {
        uint8_t index = first + row, y = 14 + row * 13;
        if (index == settingsSelected) { oled.drawRBox(2, y, 124, 12, 2); oled.setDrawColor(0); }
        oled.drawStr(9, y + 9, items[index]);
        oled.setDrawColor(1);
      }
      footer(notice[0] ? notice : "Turn:Browse Press:Open");
    } else if (screen == RESET_PRESETS) {
      oled.setFont(u8g2_font_6x10_tf);
      centered("Remove added presets?", 26);
      oled.setFont(u8g2_font_5x7_tf);
      centered("Restore 8,10,20,50,100 mL", 39);
      centered("Brightness stays unchanged", 49);
      footer("Trigger:Reset Knob:Cancel");
    } else if (screen == BRIGHTNESS) {
      oled.setFont(u8g2_font_helvB24_tn);
      snprintf(text, sizeof(text), "%u", (unsigned)editBrightness);
      centered(text, 39);
      oled.setFont(u8g2_font_5x7_tf);
      centered("percent", 50);
      footer("Turn:Adjust Press:Save");
    } else if (screen == ABOUT) {
      oled.setFont(u8g2_font_6x10_tf);
      centered("PERFUME FILLER", 25);
      oled.setFont(u8g2_font_5x7_tf);
      centered("Hardware mode", 37);
      centered("Settings saved onboard", 48);
      footer("Press knob:Back");
    } else if (screen == MENU) {
      uint8_t menuCount = PRESET_COUNT + 2;
      int first = constrain((int)selected - 1, 0, (int)menuCount - 3);
      oled.setFont(u8g2_font_6x10_tf);
      for (uint8_t row = 0; row < 3; row++) {
        uint8_t index = first + row;
        uint8_t y = 14 + row * 13;
        if (index == selected) { oled.drawRBox(2, y, 116, 12, 2); oled.setDrawColor(0); }
        if (index < PRESET_COUNT) snprintf(text, sizeof(text), "%u mL", (unsigned)PRESETS[index]);
        else snprintf(text, sizeof(text), "%s", index == PRESET_COUNT ? "Custom..." : "Settings...");
        oled.drawStr(9, y + 9, text);
        if (index == selected) oled.drawStr(107, y + 9, ">");
        oled.setDrawColor(1);
      }
      oled.drawVLine(124, 15, 35);
      oled.drawBox(122, 15 + (uint16_t)selected * 30 / (menuCount - 1), 5, 5);
      footer("Turn:Browse  Press:Set");
    } else if (screen == CUSTOM || screen == READY || screen == ADD_PRESET) {
      drawVolume(screen == READY ? targetML : editML, 40);
      if (screen != READY) {
        oled.drawTriangle(4, 30, 9, 26, 9, 34);
        oled.drawTriangle(123, 30, 118, 26, 118, 34);
      }
      oled.setFont(u8g2_font_5x7_tf);
      centered(screen == ADD_PRESET ? (notice[0] ? notice : "Trigger: Save preset") :
               screen == CUSTOM ? "Trigger: Set + fill" : "Press trigger to fill", 51);
      footer(screen == READY ? "Press knob:Change volume" : "Turn:Adjust  Knob:Back");
    } else if (screen == FILLING || screen == PAUSED) {
      drawVolume((uint16_t)(deliveredUL / 1000UL), 38);
      oled.setFont(u8g2_font_5x7_tf);
      snprintf(text, sizeof(text), "of %u mL", (unsigned)targetML);
      centered(text, 48);
      oled.drawRFrame(7, 51, 114, 5, 2);
      uint8_t width = (uint8_t)min(112UL, (unsigned long)deliveredUL * 112UL / ((uint32_t)targetML * 1000UL));
      if (width) oled.drawBox(8, 52, width, 3);
      footer(screen == PAUSED ? "Tap:Resume Hold:Reset" : "Tap:Pause Hold:Reset");
    } else {
      if (screen == COMPLETE) {
        // Tick drawn with lines, independent of font symbol coverage.
        oled.drawLine(54, 20, 61, 27);
        oled.drawLine(61, 27, 74, 15);
        oled.drawLine(54, 21, 61, 28);
        oled.drawLine(61, 28, 74, 16);
      } else {
        oled.drawBox(57, 18, 5, 10);
        oled.drawBox(66, 18, 5, 10);
      }
      oled.setFont(u8g2_font_6x10_tf);
      snprintf(text, sizeof(text), "%lu.%lu / %u mL",
               (unsigned long)(deliveredUL / 1000),
               (unsigned long)((deliveredUL % 1000) / 100), (unsigned)targetML);
      centered(text, 44);
      footer(notice[0] ? notice : "Release buttons to reset");
    }
  } while (oled.nextPage());
}

// Animated boot logo: the logo is revealed left to right over 640 ms, then an
// underline draws across over the next 720 ms. Total fits inside BOOT_DURATION_MS.
void drawBoot(unsigned long elapsed) {
  uint8_t visible = min((unsigned long)BOOT_LOGO_W, elapsed * BOOT_LOGO_W / 640UL);
  oled.firstPage();
  do {
    oled.drawXBMP(4, 18, BOOT_LOGO_W, BOOT_LOGO_H, bootLogo);
    if (visible < BOOT_LOGO_W) {
      oled.setDrawColor(0);
      oled.drawBox(4 + visible, 18, BOOT_LOGO_W - visible, BOOT_LOGO_H);
      oled.setDrawColor(1);
    }
    if (elapsed >= 640) {
      uint8_t length = min(72UL, (elapsed - 640) * 72UL / 720UL);
      if (length) oled.drawHLine(28, 54, length);
    }
  } while (oled.nextPage());
}

void setup() {
  Serial.begin(115200); // Pulse/volume diagnostics for calibration over USB serial.
  digitalWrite(PUMP_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT);
  pumpOff();
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseISR, FALLING);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  knob.begin(ENC_SW, 15);
  trigger.begin(TRIGGER, 25);
  loadSettings();
  targetML = PRESETS[0];
  encoderPrevious = (PIND >> 3) & 3;
  PCIFR |= _BV(PCIF2);
  PCMSK2 |= _BV(PCINT19) | _BV(PCINT20);
  PCICR |= _BV(PCIE2);
  oled.setI2CAddress(OLED_ADDRESS << 1);
  oled.begin();
  oled.setContrast((uint16_t)brightness * 255 / 100);
  bootStartedAt = millis();
}

void loop() {
  unsigned long now = millis();
  knob.update(now);
  trigger.update(now);
  int16_t movement = readEncoder();
  if (bootActive) {
    unsigned long elapsed = now - bootStartedAt;
    if (elapsed >= BOOT_DURATION_MS) {
      bootActive = false;
      // Discard boot-time input; held buttons must be released before use.
      knob.begin(ENC_SW, 15);
      trigger.begin(TRIGGER, 25);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { encoderTicks = 0; }
      drawScreen();
      lastFrame = millis();
    } else if (now - lastFrame >= 60) {
      lastFrame = now;
      drawBoot(elapsed);
    }
    return;
  }
  updateBattery(now);
  if (notice[0] && (long)(now - noticeUntil) >= 0) notice = "";

  // Consume the entire gesture that started a fill, including its release.
  bool triggerTap = trigger.clicked && !ignoreStartTrigger;
  bool triggerReset = trigger.held && !ignoreStartTrigger;
  if (!trigger.down && !trigger.rawDown) ignoreStartTrigger = false;
  if (screen == FILLING || screen == PAUSED) {
    if (triggerReset) {
      pumpOff();
      deliveredUL = 0;
      resetFlowPulses();
      screen = READY; // Keep selected target; a fresh press starts a new fill.
    } else if (knob.pressed) {
      finishFill(STOPPED, now);
    } else if (triggerTap) {
      if (screen == FILLING) {
        pumpOff();
        screen = PAUSED;
      } else {
        lastFlowPulseAt = now; // Give the flow sensor time to resume after pause.
        lastObservedFlowPulses = readFlowPulses();
        screen = FILLING;
        pumpOn();
      }
    }
  }

  switch (screen) {
    case MENU:
      selected = constrain((int)selected + movement, 0, (int)PRESET_COUNT + 1);
      if (knob.pressed) {
        if (selected == PRESET_COUNT) { editML = customML; lastTurn = 0; screen = CUSTOM; }
        else if (selected == PRESET_COUNT + 1) { settingsSelected = 0; screen = SETTINGS; }
        else { targetML = PRESETS[selected]; screen = READY; }
      }
      break;
    case CUSTOM:
    case ADD_PRESET:
      if (movement) {
        int step = (lastTurn != 0 && now - lastTurn < 70) ? 5 : 1;
        editML = constrain((long)editML + (long)movement * step, (long)MIN_ML, (long)MAX_ML);
        lastTurn = now;
      }
      if (knob.pressed) {
        screen = screen == ADD_PRESET ? SETTINGS : MENU;
      }
      else if (trigger.pressed) {
        if (screen == ADD_PRESET) {
          bool duplicate = false;
          for (uint8_t i = 0; i < PRESET_COUNT; i++) if (PRESETS[i] == editML) duplicate = true;
          if (duplicate) { notice = "Already in presets"; noticeUntil = now + 1800; }
          else if (PRESET_COUNT < MAX_PRESETS) {
            uint8_t pos = PRESET_COUNT;
            while (pos && PRESETS[pos - 1] > editML) { PRESETS[pos] = PRESETS[pos - 1]; pos--; }
            PRESETS[pos] = editML;
            PRESET_COUNT++;
            selected = PRESET_COUNT + 1;
            saveSettings();
            notice = "Preset saved"; noticeUntil = now + 1800;
            screen = SETTINGS;
          }
        } else {
          customML = editML;
          targetML = editML;
          startFill(now);
        }
      }
      break;
    case SETTINGS:
      settingsSelected = constrain((int)settingsSelected + movement, 0, 4);
      if (knob.pressed) {
        notice = "";
        if (settingsSelected == 0) { editBrightness = brightness; screen = BRIGHTNESS; }
        else if (settingsSelected == 1) {
          if (PRESET_COUNT >= MAX_PRESETS) { notice = "Preset limit: 8"; noticeUntil = now + 1800; }
          else { editML = customML; lastTurn = 0; screen = ADD_PRESET; }
        } else if (settingsSelected == 2) screen = RESET_PRESETS;
        else if (settingsSelected == 3) screen = ABOUT;
        else { selected = PRESET_COUNT + 1; screen = MENU; }
      }
      break;
    case RESET_PRESETS:
      if (knob.pressed) screen = SETTINGS;
      else if (trigger.pressed) {
        const uint16_t defaults[] = {8, 10, 20, 50, 100};
        PRESET_COUNT = 5;
        for (uint8_t i = 0; i < MAX_PRESETS; i++) PRESETS[i] = i < 5 ? defaults[i] : 0;
        selected = PRESET_COUNT + 1;
        saveSettings();
        notice = "Default presets restored";
        noticeUntil = now + 1800;
        screen = SETTINGS;
      }
      break;
    case BRIGHTNESS:
      if (movement) {
        editBrightness = constrain((int)editBrightness + movement * 5, 10, 100);
        oled.setContrast((uint16_t)editBrightness * 255 / 100);
      }
      if (knob.pressed) {
        if (brightness != editBrightness) { brightness = editBrightness; saveSettings(); }
        screen = SETTINGS;
      }
      break;
    case ABOUT:
      if (knob.pressed) screen = SETTINGS;
      break;
    case READY:
      if (knob.pressed) screen = MENU;
      else if (trigger.pressed) startFill(now);
      break;
    case FILLING: {
      // Real closed-loop fill: stop at the calibrated pulse count, or fail safe
      // if the sensor reports no flow. The timeout is ignored while PAUSED.
      uint32_t pulses = readFlowPulses();
      if (pulses != lastObservedFlowPulses) {
        lastObservedFlowPulses = pulses;
        lastFlowPulseAt = now;
      }
      deliveredUL = (uint32_t)((float)pulses * 1000.0f / FLOW_PULSES_PER_ML);

      if (pulses >= targetPulseCount) {
        deliveredUL = (uint32_t)targetML * 1000UL;
        finishFill(COMPLETE, now);
      } else if ((pulses == 0 && now - fillStartedAt >= FLOW_START_GRACE_MS) ||
                 (pulses > 0 && now - lastFlowPulseAt >= FLOW_STALL_TIMEOUT_MS)) {
        finishFill(STOPPED, now);
        notice = "NO FLOW - CHECK LINE";
        noticeUntil = now + 3000UL;
      }
      break;
    }
    case COMPLETE:
    case STOPPED:
      if (now - resultAt >= 1500 && !trigger.down && !trigger.rawDown && !knob.down)
        screen = READY;
      break;
    case PAUSED:
      // Keep delivered volume and target unchanged until resume or reset.
      break;
  }
  if (now - lastFrame >= 80) { lastFrame = now; drawScreen(); }
}
