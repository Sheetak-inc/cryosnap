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
//   ENABLE_SOFT_START         ~150 B flash, 9 B RAM
//       Linear I_limit ramp on the rising edge of every actuating
//       window, so neither PID nor bang-bang slams the TEC with full
//       commanded current the instant OE asserts. Lives above both
//       controllers — direction is decided by the controller and
//       applied unchanged. Window stays strictly inside
//       FAULT_GRACE_MS so the supply-Vlim and fan-tach grace checks
//       cover the ramp. Ramp re-arms on three triggers: operator
//       enable, chip recovery via _tps_only_recover, and any actuate
//       gap longer than SOFT_START_RESET_GAP_MS (deadband stay).
//       Addresses BUG-003 Phase-5 enable-edge inrush. NOT a fix for
//       the chip-lockup ceiling — see Version.h v0.7.10 notes.
//       Tuning: SOFT_START_MS, SOFT_START_I_MA,
//       SOFT_START_RESET_GAP_MS below. V_limit is NOT ramped (a TEC
//       is current-limited; V_limit is just a ceiling).
//
//   ENABLE_VERBOSE_BOOT       ~380 B flash
//       EEPROM blank/loaded message, PD reinit chatter, init warnings,
//       PD-budget echo. Off = silent boot. The version banner ("CryoSnap
//       vX.Y.Z REVA") is emitted regardless of this flag because
//       operator and log-parsing tools depend on it.
//
//   SEEBECK_HB_OFF_MAX_MS     ~290 B flash, 7 B RAM (when > 0)
//       Non-blocking Seebeck wait state machine at the actuate layer:
//       on detected polarity flip, drops OE, reads bus voltage as
//       Seebeck-EMF proxy, waits proportional to measured EMF, then
//       performs the H-bridge direction change. Mitigates the
//       polarity-flip transient that couples Seebeck EMF + H-bridge
//       inductive kickback into the TPS55288 EN/VCC pin at high ΔT.
//       Tuning: SEEBECK_HB_OFF_BASE_MS / SETTLE_MS / PER_VOLT_MS /
//       MAX_MS below. Setting MAX_MS=0 compiles the entire feature out.
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
  #define ENABLE_SOFT_START         0
  #define ENABLE_VERBOSE_BOOT       0
  #define ENABLE_SAFETY_FAULTS      0
  #define ENABLE_OLED_DISPLAY       0
  #define ENABLE_DIAGNOSTICS        0
  #define COMPACT_FAULT_MSGS        1
  // SEEBECK_HB_OFF_MAX_MS=0 compiles the Seebeck wait state machine
  // out entirely (~290 B flash, 7 B RAM). MINIMAL_BUILD targets ship-
  // worthy firmware that has neither soft-start nor the polarity-flip
  // mitigation; both Phase-5-class chip-wedge avoidance and inrush
  // protection are sacrificed for footprint.
  #define SEEBECK_HB_OFF_MAX_MS     0
  #define ENABLE_SEEBECK_TRACE      0
#endif

#ifndef ENABLE_DEBUG_DUMP_REGS
#define ENABLE_DEBUG_DUMP_REGS    1
#endif
#ifndef ENABLE_EEPROM_SETTINGS
#define ENABLE_EEPROM_SETTINGS    1
#endif
#ifndef ENABLE_I2C_BOOT_SCAN
// Default OFF. Driver-specific WARN messages (WARN TPS, WARN INA,
// WARN HUSB, WARN OLED) at boot already report missing chips by
// name. The full bus walk is a bring-up convenience that costs
// ~300 B flash. Override with -DENABLE_I2C_BOOT_SCAN=1 when an
// unexpected device may be on the bus.
#define ENABLE_I2C_BOOT_SCAN      0
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
#ifndef ENABLE_SOFT_START
#define ENABLE_SOFT_START         1
#endif
// Soft-start ramp tuning (only used when ENABLE_SOFT_START=1).
//
// SOFT_START_MS — ramp duration (linear interpolation from
//   SOFT_START_I_MA up to g_imax_mA). Must stay strictly below
//   FAULT_GRACE_MS (3000) so the supply-Vlim and fan-tach grace
//   windows cover the full ramp. Compile-time guard in
//   CryoSnap.ino enforces this (placed there because FAULT_GRACE_MS
//   lives in CryoSnap.ino, included after this header). Default
//   600 ms: chosen as a "comfortably under the 3 s grace window"
//   value; not yet derived from a scope measurement of actual
//   inrush envelope — see Version.h v0.7.10 risk register.
//
// SOFT_START_I_MA — starting current at ramp t=0. 200 mA picked to
//   stay clear of the SUPPLY_VLIM_FLOOR-driven supply check trigger
//   (which keys on V_limit, not I_limit, but a tiny non-zero current
//   write makes chip-side telemetry less ambiguous).
//
// SOFT_START_RESET_GAP_MS — gap (ms) of "no actuating tick" that
//   triggers a fresh ramp on the next actuate. Default 500 ms:
//   comfortably above the 100 ms scheduler period (deadband entry
//   alone won't re-arm) but well below typical thermostat deadband
//   stays (so a multi-second deadband followed by a re-drive gets a
//   fresh ramp). Compile-time constant only; no runtime knob.
//
// V_limit is NOT ramped: for a TEC (low-impedance resistive + Seebeck
// EMF) V_limit is a ceiling that never engages during drive, so
// ramping it would cost ~50 B flash for no inrush benefit.
#ifndef SOFT_START_MS
#define SOFT_START_MS             600
#endif
#ifndef SOFT_START_I_MA
#define SOFT_START_I_MA           200
#endif
#ifndef SOFT_START_RESET_GAP_MS
#define SOFT_START_RESET_GAP_MS   500
#endif

// Direction-change Seebeck mitigation (non-blocking state machine).
// When the controller flips drive_dir, the actuate layer drops OE,
// settles briefly so the output cap equilibrates with the TEC's
// Seebeck EMF through the H-bridge body diodes, READS the residual
// bus voltage (= Seebeck EMF on the de-energised load), then waits
// a measurement-scaled period before re-asserting OE in the new
// direction. The wait length tracks actual heat-removal conditions:
// a chilled-water-loop installation keeps ΔT (and therefore Seebeck)
// small and gets the BASE wait only; a poorly-heatsunk one accumulates
// large ΔT and gets up to MAX_MS. Does not constrain fast temperature
// movements when the customer's heat path can keep ΔT low.
//   wait_ms = BASE_MS + V_seebeck * PER_VOLT_MS  (clamped to MAX_MS)
//
// SEEBECK_HB_OFF_BASE_MS — minimum wait per flip, applied even when
//   measured Seebeck is ~0 V. 100 ms covers the chip's internal
//   recovery period observed across the Phase-4 toggle sequence.
//
// SEEBECK_HB_OFF_SETTLE_MS — quiet period between OE-off and the
//   INA bus voltage sample, so the cap equilibrates with Seebeck
//   through the body diodes before measurement.
//
// SEEBECK_HB_OFF_PER_VOLT_MS — scaling factor from measured Seebeck
//   voltage to additional wait. 1000 = 1 second of additional wait
//   per volt of measured Seebeck.
//
// SEEBECK_HB_OFF_MAX_MS — clamp ceiling on the computed wait. Also
//   acts as the feature kill switch: setting MAX_MS=0 compiles the
//   state machine out entirely (polarity flips proceed without any
//   mitigation, as in v0.7.10 and earlier).
#ifndef SEEBECK_HB_OFF_BASE_MS
#define SEEBECK_HB_OFF_BASE_MS    100
#endif
#ifndef SEEBECK_HB_OFF_SETTLE_MS
#define SEEBECK_HB_OFF_SETTLE_MS   20
#endif
#ifndef SEEBECK_HB_OFF_PER_VOLT_MS
#define SEEBECK_HB_OFF_PER_VOLT_MS 1000
#endif
#ifndef SEEBECK_HB_OFF_MAX_MS
#define SEEBECK_HB_OFF_MAX_MS    3000
#endif

// Hysteresis floor: after a Seebeck wait completes, suppress new wait
// triggers for this many ms. Aimed at the limit-cycle case (bang-bang
// + Mode Auto, EEPROM-stale 0-deadband, etc.) where the controller
// oscillates around setpoint and the SM would otherwise fire on every
// crossing. Within the floor, any prior wait has just protected the
// chip from polarity-flip stress, so a subsequent flip can proceed
// without a fresh wait. Set to 0 to disable hysteresis.
#ifndef SEEBECK_MIN_SAME_DIR_MS
#define SEEBECK_MIN_SAME_DIR_MS   1000
#endif

#ifndef ENABLE_VERBOSE_BOOT
// Default OFF. EEPROM blank/loaded status, PD-reinit chatter, and
// PD-budget echo are useful during bring-up but not in steady-state
// operation. The version banner stays emitted regardless (see the
// banner block in setup()) so operator and log-parsing tools can
// still identify the running build. Override with
// -DENABLE_VERBOSE_BOOT=1 on a flash-budget-rich build.
#define ENABLE_VERBOSE_BOOT       0
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

#ifndef ENABLE_SEEBECK_TRACE
// Default ON when diagnostics are on. Emits a `DIAG: Seebeck V=x.xx
// wait=NNNNms` line at each polarity-flip wait calculation, then a
// `DIAG: Seebeck t V=x.xx` line every 100 ms tick during the wait
// so the EMF decay curve is visible in the serial log. ~150 B flash.
// Set to 0 in flash-tight builds; auto-zeroed under MINIMAL_BUILD.
#define ENABLE_SEEBECK_TRACE      ENABLE_DIAGNOSTICS
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
