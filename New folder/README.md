# Perfume dispenser UI prototype

Open `Perfume_UI.ino` in Arduino IDE. Keep the sketch in its `Perfume_UI` folder.
Keep `BootLogo.h` in that same folder; it contains the Superior logo bitmap.

## Boot screen

The supplied Superior wordmark is encoded as a 120x29 monochrome bitmap, using 435 bytes of flash. On boot it reveals from left to right, draws an underline, and holds briefly before opening the menu (2.2 seconds total). The saved brightness setting applies to the boot screen. Buttons and rotation during boot are discarded; held buttons must be released before operating the menu. The pump stays off.

## Hardware assumptions

- Classic 5 V ATmega328P Arduino Nano.
- 128x64 I2C OLED. Default controller: SH1106; address: 0x3C.
- Mechanical quadrature rotary encoder with normally open push button.
- Normally open trigger microswitch.

## Connections

| Part | Nano connection |
|---|---|
| OLED SDA | A4 |
| OLED SCL | A5 |
| OLED GND | GND |
| OLED VCC | Module-rated supply; check its voltage and logic compatibility |
| Encoder CLK/A | D3 |
| Encoder DT/B | D4 |
| Encoder SW | D5 |
| Encoder common/GND | GND |
| Encoder module VCC, if needed | Module-rated supply |
| Trigger NO | D6 |
| Trigger COM | GND |

D2 is reserved for the future flow sensor. D9 is held LOW throughout this prototype. No pump or flow sensor is needed to test the interface.

## Upload

1. Install **U8g2 by oliver** through Arduino IDE's Library Manager.
2. Open the sketch and select Arduino Nano, ATmega328P, and the correct port. Some older clones require the Old Bootloader processor option.
3. If your OLED is SSD1306, change `OLED_SH1106` from `1` to `0`.
4. Set `OLED_ADDRESS` to your module's actual I2C address (usually 0x3C or 0x3D).
5. Upload.

## Controls

- Menu: turn to browse 8, 10, 20, 50, 100 mL and Custom; press to select.
- Custom: turn slowly for 1 mL increments; faster turns use 5 mL increments. Press the trigger to confirm and immediately start the simulated fill. Press the encoder to cancel the edit and return to the menu. Range: 1–500 mL.
- Ready: press encoder to change volume; press trigger once to begin a simulated fill.
- Filling: tap and release the trigger to pause; tap again to resume. Hold the trigger for 1 second while filling or paused to reset delivered volume to zero and return to Ready, keeping the selected target. The starting trigger gesture is ignored until released, preventing an immediate pause. Pressing the encoder stops early.
- Paused: volume and progress remain visible and unchanged. Trigger short presses take effect on release to distinguish them from a reset hold; the encoder still responds on press with 15 ms debounce.
- Complete/Stopped: after 1.5 seconds, return to Ready once both buttons are released. A new trigger press is required for another fill.
- If the encoder runs backwards, set `ENCODER_DIRECTION = -1`.
- If it takes two detents to move one menu item, try `COUNTS_PER_DETENT = 2` instead of 4.

The custom value is remembered in RAM until power is removed. There is no EEPROM persistence in this version.

## Settings and persistence

Scroll past Custom to Settings. Press the encoder to open a settings item.

- Brightness: turn to preview 10–100% in 5% steps; press the encoder to save and return. This adjusts OLED contrast, so perceived brightness depends on the display module.
- Add preset: turn to choose 1–500 mL; press the trigger to save without dispensing, or press the encoder to cancel. Presets are sorted and duplicates are rejected. Maximum eight total presets, including the five defaults. Editing/deleting presets is not implemented.
- About: shows product name, UI version and demo status; press encoder to return.
- Reset presets: opens a confirmation screen. Press the trigger to restore 8, 10, 20, 50 and 100 mL and remove added presets, or press the encoder to cancel. Brightness is retained and the restored list is saved to EEPROM.
- Back: returns to the volume menu.

Brightness and the preset list persist in onboard EEPROM after restart or power removal. The custom edit value and in-progress fill do not persist. Saves occur only on confirmation and use EEPROM.update; invalid or incomplete saved data falls back to defaults. EEPROM bytes 0–21 are reserved by this sketch. Firmware that uses the same addresses may replace these settings. An interrupted save may restore defaults rather than the prior configuration.

## Battery measurement

Battery measurement is disabled by default; the header shows a battery outline with a question mark.

The optional configuration assumes a conventional 3S lithium-ion pack, with battery positive through 33 kOhm to A0 and 10 kOhm from A0 to ground. All grounds must be common. Never connect pack voltage directly to A0. Verify the divider, pack maximum voltage, and actual Nano 5 V supply before enabling `BATTERY_ENABLED`. A 100 nF capacitor from A0 to ground can help filter noise.

The displayed bars use a smoothed, linear 9.6–12.6 V estimate. This is only a rough indication, not a calibrated state of charge, and provides no battery cutoff or charging protection.

## Scope and verification

Filling is simulated at 5 mL/s and marked DEMO. The sketch does not measure liquid or energize the pump. Real dispensing needs calibrated flow counting, pump control and fault handling before use.

Source was reviewed, but not compiled or tested on hardware in this environment; Arduino CLI and the AVR compiler were unavailable. Confirm encoder behavior, OLED controller/address, and trigger behavior on the actual hardware.

