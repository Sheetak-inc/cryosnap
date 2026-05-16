#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// Pin Definitions (Arduino Nano — bench prototype)
// ============================================

// UART — hardware defaults
// RX 0
// TX 1

// Fan Control
#define PIN_FAN_TACH 2
#define PIN_FAN_PWM  3

// WS2812B LED chain
#define PIN_LED_DAT  7

// H-Bridge direction (DRV8701E PH pin, single GPIO)
#define PIN_DIR      6

// UI
#define PIN_BUTTON_ENABLE 8  // needs internal pullup (active low, press = GND)

// SPI — hardware defaults (OLED display, not in firmware scope)
// MOSI 11
// MISO 12
// SCK  13

// Error detection
#define PIN_nFAULT_TPS55288 A0
#define PIN_INA_ALERT       10   // INA226 alert output (polled, not ISR)

// NTCs (op-amp conditioned ADC inputs)
#define NTC_1 A1
#define NTC_2 A2
#define NTC_3 A3

// I2C — hardware defaults
// SDA: A4
// SCL: A5

// UI
#define PIN_MODE_SWITCH A6
#define PIN_POT         A7

// ============================================
// System Constants
// ============================================
#define SERIAL_BAUD_RATE 115200
#define LOOP_INTERVAL_MS 100  // Main control loop frequency (10 Hz)

// Serial plotter stream interval (in 100 ms ticks).
// Only active when streaming is enabled via the "stream" command.
//   1  = every 100 ms (full 10 Hz, for Arduino Serial Plotter GUI)
//   10 = every 1 s   (good default for serial monitor)
//   100 = every 10 s (minimal output)
#define PLOTTER_INTERVAL 10

// ============================================
// Driver Hardware Constants
// ============================================
#define TPS_RSENSE   0.010f  // TPS55288 sense resistor, Ohms (10 mOhm)
#define TPS_I2C_ADDR 0x75    // 0x75 on prototype, 0x74 on Rev A

// INA226 shunt resistor.
// With 10 mOhm: max measurable current = 81.92 mV / 10 mOhm = 8.19 A.
// (With a 100 mOhm shunt this would cap at 0.82 A — readings above
// that saturate.) CAL value recomputes automatically in INA226.h.
// To change the shunt: replace the SMD resistor and update this value.
#define INA_RSENSE   0.010f  // Ohms (10 mOhm)

// ============================================
// Debug Options
// ============================================

// Set to 1 to use the Adafruit_HUSB238 library for PD negotiation
// instead of our native driver. Useful for A/B testing when the
// native driver can't negotiate. Requires installing the
// Adafruit_HUSB238 library (not stock Arduino IDE — debug only).
#define USE_ADAFRUIT_HUSB  0

// ============================================
// Feature Flags  —  set any to 0 to reclaim flash/RAM
// ============================================
//
// Each flag controls a self-contained piece of functionality.
// Approximate flash + RAM savings (relative to a fully-enabled build):
//
//   ENABLE_DEBUG_DUMP_REGS    ~2400 B flash, 28 B RAM
//       Verbose I2C register dump in `status` and at boot. Useful
//       for hardware bring-up; turn off once silicon is verified.
//
//   ENABLE_EEPROM_SETTINGS    ~1100 B flash
//       save / load / defaults serial commands and Settings.h.
//       Without it the firmware always boots with Config.h defaults.
//
//   ENABLE_I2C_BOOT_SCAN      ~300 B flash
//       Walk every I2C address at boot and print devices found.
//       Disable once the board is known-good.
//
//   ENABLE_NTC_CALIBRATION    ~600 B flash
//       cal / cal1 / cal2 / calshow serial commands. Disable after
//       the NTC scale + offset are stored to EEPROM.
//
//   ENABLE_SERIAL_PLOTTER     ~700 B flash
//       Tab-separated plotter output in the 100 ms task plus the
//       `stream` command. Disable in production / when no host PC
//       is reading the line.
//
//   ENABLE_LED_FADE           ~150 B flash
//       Fractional-brightness fade on the topmost LED of the temp
//       and setpoint bars. Cosmetic — disabling gives a hard step.
//
//   ENABLE_FAULT_LED_FLASH    ~100 B flash
//       Slow red flash on all 23 LEDs when a fault latches. Per
//       spec §24, on by default. Disabling skips the override.
//
//   ENABLE_PD_BUDGET_CLAMP    ~250 B flash
//       Software clamp that locks the TPS V/I limits when the INA
//       sees the negotiated USB-PD wattage. Belt-and-braces
//       protection on top of the TPS hardware current limiter.
//
//   ENABLE_VERBOSE_BOOT       ~250 B flash
//       Boot banner, EEPROM blank/loaded message, PD reinit
//       chatter, init warnings. Off = silent boot, ready prompt.
//
//   ENABLE_SAFETY_FAULTS      ~600 B flash
//       Master switch for the FOUR fault sources beyond OVERTEMP
//       (INA alert, TPS PG, HUSB drop, fan tach). OVERTEMP stays
//       on — it's the minimum protection. Useful for stripped
//       reference builds.
//
//   ENABLE_USB_PD             ~2500 B flash, 80 B RAM
//       Alias for HAS_HUSB238 — drops the entire HUSB238 driver,
//       PD budget, and fault when set to 0. Use for screw-terminal
//       builds that bypass USB-C entirely.
//
//   ENABLE_OLED_DISPLAY       ~1300 B flash, ~10 B RAM
//       SSD1306 status display refreshed once per second from the
//       slow task. Auto-disables at runtime if the panel doesn't
//       ACK at boot, so leaving this on with no display fitted is
//       safe — it just costs the flash for the unused driver.
//
//   ENABLE_DIAGNOSTICS        ~500 B flash, 6 B RAM
//       Cross-chip consistency checks in the slow task. Fires
//       advisory DIAG: ... messages on state changes — TPS missing
//       while PD is ok, INA offline, TPS reports SCP, TPS driving
//       but no current visible at INA. None of these latch a fault;
//       the TPS handles its own catastrophic shutdowns.
//
// Display refresh cadence:
//   OLED_REFRESH_TICKS  number of slow-task ticks between full
//                       redraws. 1 = every 1 s. Higher = less I2C
//                       traffic but staler readout.
//
// COMPACT_FAULT_MSGS  =1  ~120 B flash savings
//       Print "FLT[1]"…"FLT[5]" instead of "FAULT[OVERTEMP]" etc.
//
// MINIMAL_BUILD       =1  one-line override that turns OFF every
//                          optional feature above (still keeps
//                          OVERTEMP fault and core control loop).
//                          Useful baseline for shipping firmware.
//
// ----------------------------------------------------------------
// HOW TO ADD A NEW FEATURE FLAG
// ----------------------------------------------------------------
//   1. Document the flag in this comment block (purpose + rough
//      flash/RAM cost from a comparative ablation build).
//   2. Add it to the MINIMAL_BUILD section below with the value
//      that minimises footprint (usually 0).
//   3. Add an `#ifndef ENABLE_FOO / #define ENABLE_FOO 1 / #endif`
//      pair so command-line / IDE overrides work and the default
//      is "on" for normal builds.
//   4. In the consumer code, gate with `#if ENABLE_FOO` and
//      provide compile-out behavior so the flag actually saves
//      flash/RAM when set to 0 (otherwise the flag is decorative).
//
#ifndef MINIMAL_BUILD
#define MINIMAL_BUILD 0
#endif

#if MINIMAL_BUILD
  #define ENABLE_DEBUG_DUMP_REGS    0
  #define ENABLE_EEPROM_SETTINGS    0
  #define ENABLE_I2C_BOOT_SCAN      0
  #define ENABLE_NTC_CALIBRATION    0
  #define ENABLE_SERIAL_PLOTTER     0
  #define ENABLE_LED_FADE           0
  #define ENABLE_FAULT_LED_FLASH    1
  #define ENABLE_PD_BUDGET_CLAMP    0
  #define ENABLE_VERBOSE_BOOT       0
  #define ENABLE_SAFETY_FAULTS      0
  #define ENABLE_OLED_DISPLAY       0
  #define ENABLE_DIAGNOSTICS        0
  #define COMPACT_FAULT_MSGS        1
#endif

#ifndef ENABLE_DEBUG_DUMP_REGS
#define ENABLE_DEBUG_DUMP_REGS    1
#endif
#ifndef ENABLE_EEPROM_SETTINGS
#define ENABLE_EEPROM_SETTINGS    1
#endif
#ifndef ENABLE_I2C_BOOT_SCAN
#define ENABLE_I2C_BOOT_SCAN      1
#endif
#ifndef ENABLE_NTC_CALIBRATION
#define ENABLE_NTC_CALIBRATION    1
#endif
#ifndef ENABLE_SERIAL_PLOTTER
#define ENABLE_SERIAL_PLOTTER     1
#endif
#ifndef ENABLE_LED_FADE
#define ENABLE_LED_FADE           1
#endif
#ifndef ENABLE_FAULT_LED_FLASH
#define ENABLE_FAULT_LED_FLASH    1
#endif
#ifndef ENABLE_PD_BUDGET_CLAMP
#define ENABLE_PD_BUDGET_CLAMP    1
#endif
#ifndef ENABLE_VERBOSE_BOOT
#define ENABLE_VERBOSE_BOOT       1
#endif
#ifndef ENABLE_SAFETY_FAULTS
#define ENABLE_SAFETY_FAULTS      1
#endif
#ifndef ENABLE_OLED_DISPLAY
#define ENABLE_OLED_DISPLAY       1
#endif
#ifndef OLED_REFRESH_TICKS
#define OLED_REFRESH_TICKS        1   // slow-task ticks (1 = each second)
#endif
// Physical SSD1306 panel height in pixels — 32 or 64. The dev kit
// ships with a 128x32 module (4 page rows of 5x7 text); 128x64
// modules (8 rows) are also common. The init sequence and the
// render layout adapt based on this value.
#ifndef OLED_HEIGHT
#define OLED_HEIGHT               32
#endif
#ifndef ENABLE_DIAGNOSTICS
#define ENABLE_DIAGNOSTICS        1
#endif
#ifndef COMPACT_FAULT_MSGS
#define COMPACT_FAULT_MSGS        0
#endif

// Backward-compat alias. Older code still references DEBUG_DUMP_REGISTERS;
// keep the name working as a synonym for ENABLE_DEBUG_DUMP_REGS.
#undef DEBUG_DUMP_REGISTERS
#define DEBUG_DUMP_REGISTERS  ENABLE_DEBUG_DUMP_REGS

// ============================================
// Default Safety Limits
// ============================================
#define DEFAULT_MAX_TEMP_HOT 60.0f  // Celsius
#define DEFAULT_MAX_CURRENT   2.0f  // Amps (TPS55288 I_limit at boot)
#define DEFAULT_MAX_VOLTAGE  12.0f  // Volts (TPS55288 V_limit at boot)
#define DEFAULT_MAX_POWER    20.0f  // Watts

// ============================================
// Default Control Settings
// ============================================
#define DEFAULT_SETPOINT     25.0f  // Celsius — target cold-side temp
#define DEFAULT_FAN_SPEED    180    // 0..255 PWM duty (180 ~ 70%)
// LED brightness scale, 0..255 (0 = off, 255 = full). 64 (~25%)
// is comfortable indoors — the full-scale WS2812 chain is bright
// enough to be uncomfortable on a bench desk. User-adjustable at
// runtime via the `bright` serial command, persisted to EEPROM
// via `save`.
//
// To change the default: edit the value here; existing EEPROM
// settings retain their stored value until `defaults` is run.
#define DEFAULT_LED_BRIGHTNESS  64
#define FAN_DISABLED_SPEED   127    // fan runs at ~50% even when TEC is off (passive cooling)
// Deadband — the window around the setpoint where the TEC is
// completely off. Prevents the control loop from chattering (rapidly
// switching on/off) when the measured temperature oscillates within
// the noise floor of the NTC. Inside this band, the TEC output is
// disabled entirely (not just reduced).
//
// DO NOT set this to zero. With a zero deadband, any millisecond of
// noise on the NTC will cause the H-bridge to flip direction or the
// TPS output to toggle, which:
//   - wears out the TEC (thermal cycling shortens lifetime)
//   - stresses the H-bridge MOSFETs on every transition
//   - creates audible/electrical noise
//   - can oscillate the control loop indefinitely
// Keep this >= 0.1 C. Larger values (0.5-1.0 C) are quieter but give
// less precise temperature tracking.
#define DEFAULT_DEADBAND     0.2f   // +/- degrees C — TEC off inside band

#define DEFAULT_DAMPING_BAND 3.0f   // +/- degrees C — reduce current in band
#define DEFAULT_DAMPING_PCT  50     // % of max current inside damping band

// PID controller tuning. Output is in milliamps; setpoint and
// measured temperature are in degrees Celsius. dt is fixed at
// LOOP_INTERVAL_MS / 1000 = 0.1 s.
//
// Conservative starter values — bench-tune from here. Kp = 200
// means a 1 °C error commands 200 mA of TEC drive. Ki adds slow
// integral correction. Kd starts at 0 because derivative kick on
// setpoint changes is rarely worth the noise on an NTC-driven
// loop until the rest of the loop is well-behaved.
#define DEFAULT_KP   200.0f
#define DEFAULT_KI     5.0f
#define DEFAULT_KD     0.0f

// Default control mode for first boot / EEPROM reset.
//   true  → PID controller
//   false → bang-bang + damping (simpler fallback law, reachable
//           via the `pid` console command for A/B comparison or
//           when PID tuning is going sideways)
#define DEFAULT_USE_PID  true

// Mode switch analog thresholds (10-bit ADC on A6).
// Full scale = Heat, Mid = Auto, Ground = Cool.
#define MODE_THRESH_COOL     256    // below this = Cool
#define MODE_THRESH_AUTO     768    // below this = Auto, above = Heat

// NTC temperature conversion — placeholder linear model.
// T(C) = raw_ADC * NTC_SCALE + NTC_OFFSET
//
// Calibrate by measuring two known temperatures (e.g. ice water
// and body temp), reading the ADC values, and solving for SCALE
// and OFFSET. Replace with Steinhart-Hart once the op-amp
// conditioning circuit values are confirmed on production hardware.
//
// The defaults below are empirical for the bench prototype's NTC +
// op-amp network. Recalibrate if the NTC part or conditioning
// circuit changes — the `cal1` / `cal2` two-point serial commands
// compute fresh constants and persist them via `save`.
#define NTC_SCALE    0.1023f  // degrees C per ADC count
#define NTC_OFFSET  -27.6f   // degrees C at ADC = 0

// Enable button debounce time (milliseconds).
#define BUTTON_DEBOUNCE_MS   50

// Minimum fan PWM duty — below this the fan stalls on the prototype.
#define FAN_MIN_DUTY         30

// Pot setpoint range — the analog pot maps linearly across this
// span. Adjust to taste; widening the range trades resolution for
// reach.
#define POT_TEMP_MIN        -10.0f  // degrees C at pot full CCW
#define POT_TEMP_MAX         40.0f  // degrees C at pot full CW

// NTC running average length (number of samples to smooth).
// Higher = smoother but more lag. At the 100 ms tick, 10 samples
// = a 1 second window.
#define NTC_AVG_LEN          10

// Set to 1 if the board routes a precision reference (typically
// 3.3 V) to the AREF pin. The op-amp NTC network is designed
// around this; with USE_EXTERNAL_AREF=0 the ADC uses the 5 V
// rail as its reference and NTC readings will be ~50% off.
//
// To check on your board: probe AREF — if it's tied to a stable
// 3.3 V (or other) source, keep this at 1. If AREF floats, set
// to 0 and the firmware uses the default 5 V AVCC reference.
#define USE_EXTERNAL_AREF    1

#endif // CONFIG_H
