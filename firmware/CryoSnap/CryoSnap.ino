/*
  CryoSnap Firmware — Rev A
  =========================

  Reference / teaching firmware for makers. Reads top to bottom.

  Hardware
  --------
  Arduino Nano (ATmega328P). The board drives a thermoelectric cooler
  (TEC) through these peripherals:

    TPS55288  I2C buck-boost — sets the voltage and current that drive
              the TEC. Output can be enabled/disabled in software.
    INA226    I2C current/voltage/power monitor — reads what the TEC
              is actually drawing (feeds the Serial Plotter line).
    HUSB238   I2C USB-PD sink controller — negotiates 20 V from a
              USB-C PD power supply. Also supports >20 V via screw
              terminal (bypasses PD entirely).
    DRV8701E  H-bridge gate driver — direction pin (PIN_DIR) selects
              heat vs cool polarity. No SPI, just one GPIO.
    WS2812B   23-LED chain — temperature bar, setpoint bar, mode /
              enable / H-bridge indicators. Bit-banged, zero deps.
    NTCs      3 thermistor channels (cold/hot/ambient) via op-amp
              conditioned ADC inputs. Only cold-side drives control.
    Fan       4-wire fan — hardware PWM speed + tach pulse ISR.

  See Pins.h for pin assignments (TARGET_PROTO vs TARGET_REVA) and
  Config.h for tunables (baud rate, safety limits, sense resistors).

  Code organization notes
  -----------------------
  All driver code lives in header-only .h files with inline functions.
  No .cpp files — the sketch compiles as a single translation unit so
  the "open and compile" story stays trivial in the stock Arduino IDE.

  Every peripheral has its own header, including the H-bridge
  (HBridge.h) even though it's currently just one GPIO pin. This
  makes it easy to swap in a more capable driver later without
  touching the main sketch.

  Execution model
  ---------------
  Pure cooperative scheduling with millis(). Three task buckets:

    Fast tasks   (every loop iteration)
      serial command parser, enable button debounce, mode switch, pot

    100 ms task  (LOOP_INTERVAL_MS from Config.h)
      read NTCs + INA226, check faults, resolve mode + setpoint,
      deadband / damping decision, update TPS V/I + PIN_DIR, update
      LED frame, emit Serial Plotter line

    1 s slow task
      status summary, HUSB238 PD re-poll, optional diagnostics

  Example boot output
  -------------------
    CryoSnap -- boot
    Build target: PROTO (bench prototype)
    I2C scan starting...
      found 0x22  (HUSB238)
      found 0x3C  (OLED SSD1306)
      found 0x40  (INA226)
      found 0x75  (TPS55288)
    I2C scan done: 4 device(s) responded.
    HUSB238: tried 20V, got 20V
    --- status ---
    PD     : 20 V / 3250 mA
    INA226 : V=0.000 V  I=0.000 A  P=0.000 W
    --------------
    --- registers ---
    TPS55288 (0x75):
      VREF_L     = 0xBF
      VREF_H     = 0x3
      IOUT_LIMIT = 0x84  (limiter ON, steps=4)
      VOUT_FS    = 0x0  (FS range code 0)
      MODE       = 0x20  (OE=off)
      STATUS     = 0x1
      V_limit    = 5.00 V
    INA226 (0x40):
      CONFIG     = 0x4127  (expected 0x4127)
      CAL        = 0x66  (expected 0x66 = 102, OK)
      MFG_ID     = 0x5449  ('TI' — expected 0x5449)
      DIE_ID     = 0x2260  (expected 0x2260)
    HUSB238 (0x22):
      PD_STATUS0 = 0x6B  (V=20V, I=3250 mA)
      PDO_20V    = 0xA3  (DETECTED)
      SRC_PDO    = 0xA0  (requesting 20V)
    --------------
    Boot complete. Type 'help' for commands.
*/

#include <Wire.h>

#include "Config.h"
#include "Version.h"
#include "Pins.h"
#include "I2CScan.h"
#include "TPS55288.h"
#include "INA226.h"
#include "HUSB238.h"
#include "HBridge.h"
#include "WS2812B.h"
#include "FanControl.h"
#include "NTC.h"
#include "OLED.h"
#include "PID.h"

// =========================================================================
// runtime state
// =========================================================================

static unsigned long g_last100  = 0;
static unsigned long g_lastSlow = 0;

// ---- control state (volatile during operation, reset on reboot) ---------

static bool     g_enabled    = false;               // TEC drive enabled
static uint8_t  g_mode       = 0;                   // 0=Cool, 1=Heat, 2=Auto
static float    g_setpoint   = DEFAULT_SETPOINT;    // target cold-side temp (C)
static uint8_t  g_fan_speed  = DEFAULT_FAN_SPEED;   // fan duty 0..255
static uint16_t g_imax_mA    = (uint16_t)(DEFAULT_MAX_CURRENT * 1000); // TPS I limit
static uint16_t g_vmax_mV    = (uint16_t)(DEFAULT_MAX_VOLTAGE * 1000); // TPS V limit
static float    g_maxhot     = DEFAULT_MAX_TEMP_HOT; // hot-side safety limit (C)
static float    g_deadband   = DEFAULT_DEADBAND;     // +/- C, TEC off inside band
static float    g_damping    = DEFAULT_DAMPING_BAND; // +/- C, reduced current inside
static uint8_t  g_damping_pct = DEFAULT_DAMPING_PCT; // % of max current when damping
static bool     g_stream     = false;                // serial plotter stream on/off
static bool     g_console    = false;                // true = serial console controls; false = physical inputs
static uint8_t  g_led_brightness = DEFAULT_LED_BRIGHTNESS; // 0..255 master scale for WS2812 frame
static bool     g_use_pid    = DEFAULT_USE_PID;     // true = PID, false = bang-bang + damping fallback

// Mode enum for readability.
#define MODE_COOL  0
#define MODE_HEAT  1
#define MODE_AUTO  2

// ---- fault state --------------------------------------------------------
// When a fault fires, g_enabled is forced false and g_fault is set to a
// non-zero code. The TEC stays off until the user clears the fault via
// the enable button or "enable 1" command (which resets g_fault to 0).
#define FAULT_NONE       0
#define FAULT_HOT_SIDE   1   // NTC hot-side > g_maxhot
#define FAULT_INA_ALERT  2   // INA226 alert function flag (AFF)
#define FAULT_TPS_PG     3   // TPS55288 power-good / fault pin (A0)
#define FAULT_HUSB_20V   4   // HUSB238 lost 20V PD contract
#define FAULT_FAN_TACH   5   // fan tach reports 0 RPM while enabled

static uint8_t  g_fault = FAULT_NONE;

// HUSB PD fault debounce: require N consecutive reads below 20V
// before latching. A single-tick I2C glitch or momentary PD
// renegotiation shouldn't trip the fault.
#define HUSB_FAULT_DEBOUNCE  5   // ticks (500 ms at 100 ms cadence)
static uint8_t _husb_fault_count = 0;

// Sticky: set true the first tick HUSB238 reports any negotiated
// voltage. Stays false on a direct-supply boot (DC input into the
// barrel jack with no USB-C source), which lets us skip the PD
// fault chain in that case — losing a contract you never had is
// not a fault. If a PD source IS plugged in and later disconnects,
// the flag is already true and the fault chain still catches it.
static bool _husb_was_attached = false;

// ---- button debounce state ----------------------------------------------

static bool     _btn_last_raw   = HIGH;  // active-low button
static bool     _btn_stable     = HIGH;
static unsigned long _btn_last_change = 0;

// ---- enable timestamp (for fault grace periods) -------------------------
// When g_enabled transitions to true, we record the time. Fault checks
// that depend on slow-polled sensors (like fan tach, which only updates
// every 1 s) skip for a grace period so the sensor has time to produce
// a valid reading before we judge it.
#define FAULT_GRACE_MS  3000  // 3 seconds after enable before checking fan tach

static unsigned long _enable_time = 0;

// ---- USB-PD power budget ------------------------------------------------
// Computed at boot from the HUSB238 negotiated voltage and current.
// The control task must not allow INA226 measured power to exceed this.
// 95% leaves enough headroom for the TPS55288's own input losses
// without tripping the clamp before the supply is actually loaded.
// Drop this if you're seeing premature PD-clamp events in normal
// operation; raise it if you want a wider safety margin.
#define PD_EFFICIENCY_PCT  95
static uint32_t g_pd_budget_mW = 0;
static bool     g_pd_clamped   = false;

// =========================================================================
// forward declarations for the scheduled tasks
// =========================================================================

static void task_100ms();
static void task_slow();

// Forward declaration — defined below, called from SerialCmd.h and button handler.
static void _pd_reinit();

// Settings.h and SerialCmd.h both reference runtime state declared
// above, so they must be included here — after the state block,
// before setup(). Settings.h must come before SerialCmd.h because
// the save/load/reset commands call into it.
#if ENABLE_EEPROM_SETTINGS
#include "Settings.h"
#endif
#include "Controller.h"
#include "Diagnostics.h"
#include "LedRender.h"
#include "OledRender.h"
#include "SerialCmd.h"

// =========================================================================
// _pd_reinit() — called on every enable transition. If PD isn't at
// 20V, re-run the HUSB238 init (which does HARD_RESET or PDO walk)
// and recompute the power budget. Also re-init TPS55288 in case it
// lost its settings during a PD brownout.
// =========================================================================

static void _pd_reinit() {
#if HAS_HUSB238
  if (husb_negotiatedV() < 20) {
#if ENABLE_VERBOSE_BOOT
    Serial.println(F("PD reinit..."));
#endif
    husb_init();
    delay(200);
#if ENABLE_PD_BUDGET_CLAMP
    uint8_t pdv = husb_negotiatedV();
    uint16_t mA = husb_maxCurrentMA();
#if ENABLE_VERBOSE_BOOT
    Serial.print(F("PD=")); Serial.print(pdv); Serial.print('/');
    Serial.println(mA);
#endif
    g_pd_budget_mW = (uint32_t)pdv * mA * PD_EFFICIENCY_PCT / 100;
#if ENABLE_VERBOSE_BOOT
    Serial.print(F("budget=")); Serial.println(g_pd_budget_mW);
#endif
#endif
  }
#endif
  tps_init();
  _husb_fault_count = 0;
}

// =========================================================================
// setup() — runs once at power-on. Bring up hardware in a safe order:
//   serial -> pins -> LEDs -> I2C bus -> drivers -> runtime state -> banner.
// =========================================================================

// =========================================================================
// setup() — runs once at power-on. Bring up hardware in a safe order:
//   pin defaults → serial → LEDs → I2C bus → drivers → runtime state →
//   banner. Each step is gated so failures (a missing chip, a blank
//   EEPROM) log a warning but don't halt the firmware.
//
// To add a new I2C peripheral:
//   1. Define I2C_ADDR_FOO in Pins.h.
//   2. Create a Foo.h header in the project's inline-driver style
//      (see TPS55288.h / INA226.h / HUSB238.h as templates).
//   3. Include it at the top of this file.
//   4. Call foo_init() in the "Driver init" block below; mirror the
//      WARN-on-fail pattern so the operator sees absence at boot.
//   5. If it should appear in the I2C scan annotation, add a case
//      to i2cScan() in I2CScan.h.
//
// To add a new fault source:
//   1. Add a FAULT_<name> constant in the runtime-state block above.
//   2. Add an else-if arm to the fault chain in task_100ms().
//   3. Add a case to the switch statements in serial_print_status()
//      (SerialCmd.h) and the OLED render (OledRender.h) so the
//      operator sees the human-readable name.
// =========================================================================
void setup() {
  // FIRST THING: force the DC/DC converter and H-bridge into a safe
  // state before any other initialization runs. The TPS55288 OE pin
  // is active-high and defaults to off, but the H-bridge direction
  // pin floats until configured — set it to a known safe level
  // immediately. This runs before Serial, before I2C, before
  // anything else touches hardware.
  pinMode(HW_HB_DIR, OUTPUT);
  digitalWrite(HW_HB_DIR, HB_COOL);

  Serial.begin(SERIAL_BAUD_RATE);
  // Wait briefly so a USB-CDC host (if any) has time to enumerate.
  // Skip the wait if Serial isn't going to be available within ~2 s.
  while (!Serial && millis() < 2000) { /* spin */ }

#if ENABLE_VERBOSE_BOOT
  Serial.println();
  Serial.print(F("CryoSnap v"));
  Serial.print(F(FW_VERSION_STR));
#if BUILD_TARGET == TARGET_PROTO
  Serial.println(F(" PROTO"));
#elif BUILD_TARGET == TARGET_REVB
  Serial.println(F(" REVB"));
#else
  Serial.println(F(" REVA"));
#endif
#endif

  // LEDs first so the maker has a visible "I'm alive" cue even if a
  // bus init below hangs.
  leds_init();

  // I2C bus comes up next — used by TPS55288, INA226, HUSB238.
  Wire.begin();

  // 25 ms per-transaction timeout, with bus reset on timeout. Without
  // this, a missing or unpowered I2C device that holds the bus low
  // makes the firmware hang for the entire duration of the scan or
  // init. 25 ms is generous enough for the longest legitimate
  // transaction (HUSB238 PDO walk waits 2 s elsewhere with explicit
  // delay()) but short enough that a 117-address scan with NO chips
  // present completes in ~3 s.
  //
  // To tune: raise the value if you add a chip that needs longer
  // clock-stretching; lower it (carefully) for a more responsive
  // scan on a sparse bus.
  Wire.setWireTimeout(25000UL, true);

#if ENABLE_I2C_BOOT_SCAN
  // Walk the bus and report what's there. Easy to disable in
  // Config.h once the board is known-good.
  i2cScan(Serial);
#endif

  // Driver init. Each call returns true on success; failures are
  // logged but do not halt the firmware so the maker can still see
  // serial output and react.
#if ENABLE_VERBOSE_BOOT
  if (!tps_init())  Serial.println(F("WARN TPS"));
  if (!ina_init())  Serial.println(F("WARN INA"));
  if (!husb_init()) Serial.println(F("WARN HUSB"));
#else
  tps_init(); ina_init(); husb_init();
#endif

  hb_init();
  fan_init();
  ntc_init();

  // OLED probe — same fail-soft pattern as the other I2C drivers.
  // If the panel isn't on the bus, oled_init() returns false and
  // every subsequent oled_*() call becomes a no-op. The control
  // loop, faults, and serial commands all work normally without
  // a display fitted; this is just a heads-up that no status
  // screen will appear.
#if ENABLE_VERBOSE_BOOT
  if (!oled_init()) Serial.println(F("WARN OLED"));
#else
  oled_init();
#endif

  // Enable button (D8, active-low). Needs internal pullup — the
  // prototype doesn't have an external pullup so the pin floats
  // otherwise and the button never registers.
  pinMode(HW_BUTTON_EN, INPUT_PULLUP);

  // The TPS55288 FB/INT pin (wired to A0 on Rev A) is intentionally
  // NOT polled here as a fault input — the chip's own internal
  // protections (OVP / OTP / SCP) handle catastrophic events
  // autonomously, so reacting to FB/INT in firmware would just
  // duplicate the work and (with OCP unmasked) trip on normal
  // current-limit operation. TPS health is monitored over I2C in
  // task_diag() instead.

#if ENABLE_EEPROM_SETTINGS
  // Try to load persisted settings from EEPROM. If the EEPROM is blank,
  // corrupted, or from an older firmware version, keep the Config.h
  // defaults that were applied at variable declaration.
  if (settings_load()) {
#if ENABLE_VERBOSE_BOOT
    Serial.println(F("EEPROM loaded"));
#endif
    tps_setCurrentLimit(g_imax_mA);
    tps_setVoltageLimit(g_vmax_mV);
  }
#if ENABLE_VERBOSE_BOOT
  else { Serial.println(F("EEPROM blank")); }
#endif
#endif

#if ENABLE_VERBOSE_BOOT
  // Print status + register dump (if ENABLE_DEBUG_DUMP_REGS is enabled).
  serial_print_status();
#endif

  // Compute the USB-PD power budget from the negotiated contract.
  // The control task uses this to prevent the INA226 measured power
  // from exceeding what the PD source can deliver.
#if HAS_HUSB238 && ENABLE_PD_BUDGET_CLAMP
  {
    uint8_t  v  = husb_negotiatedV();      // 0/5/9/12/15/18/20
    uint16_t mA = husb_maxCurrentMA();     // 0..5000
    // Budget in mW = V * mA * efficiency / 100
    g_pd_budget_mW = (uint32_t)v * mA * PD_EFFICIENCY_PCT / 100;
#if ENABLE_VERBOSE_BOOT
    Serial.print(F("budget=")); Serial.println(g_pd_budget_mW);
#endif
  }
#endif

  // Initialize the enable timestamp so the fan tach grace period
  // works correctly even on the very first enable press. Without
  // this, _enable_time = 0 and any enable press > 3 s after boot
  // sees (millis() - 0) > 3000 → grace expired before it started.
  _enable_time = millis();

#if ENABLE_VERBOSE_BOOT
  Serial.println(F("ready. 'help' for commands"));
#endif
}

// =========================================================================
// loop() — cooperative scheduler. Three buckets:
//   fast tasks   every iteration   (serial, button, mode/pot reads)
//   100 ms task  control pipeline  (sense, decide, act)   — LOOP_INTERVAL_MS
//   1 s task     status + diagnostics
//
// To change a task period: edit LOOP_INTERVAL_MS in Config.h for the
// 100 ms task, or the literal `1000` below for the slow task. Keep
// the 100 ms cadence in mind if you tune the PID — pid_compute()
// hard-codes dt = LOOP_INTERVAL_MS / 1000 inside task_100ms().
//
// To add a new periodic task:
//   1. Add a static `unsigned long g_lastFoo = 0;` near the other
//      schedulers above.
//   2. Add an `if (now - g_lastFoo >= <period_ms>) { g_lastFoo = now;
//      task_foo(); }` block in the scheduler below.
//   3. Implement `static void task_foo() { ... }` — keep it short and
//      non-blocking. Anything that may block longer than its period
//      belongs on the slow task or in its own thread of execution
//      (e.g. fan_pollRPM(), which blocks ~100 ms).
// =========================================================================

void loop() {
  // -- fast tasks: every iteration --------------------------------------

  // --- Physical inputs (button, mode switch, pot) ---
  // When g_console is false (default): physical inputs drive the state.
  // When g_console is true: physical inputs are ignored, serial
  //   commands are the only way to change enable/mode/setpoint.
  // Toggle with the "console" serial command.
  if (!g_console) {
    // Enable button debounce (D8, active-low with pullup).
    {
      bool raw = digitalRead(HW_BUTTON_EN);
      if (raw != _btn_last_raw) {
        _btn_last_change = millis();
        _btn_last_raw = raw;
      }
      if ((millis() - _btn_last_change) > BUTTON_DEBOUNCE_MS) {
        if (raw != _btn_stable) {
          _btn_stable = raw;
          if (_btn_stable == LOW) {
            g_fault = FAULT_NONE;  // clear any latched fault
            g_enabled = !g_enabled;
            if (g_enabled) {
              _enable_time = millis();  // start grace period
              _pd_reinit();            // re-negotiate PD if needed
              pid_reset();             // clear stale integral/derivative
              ctrl_on_enable();        // kick off armed schedule, if any
            }
            Serial.print(F("Enable: "));
            Serial.println(g_enabled ? F("ON") : F("OFF"));
          }
        }
      }
    }

    // Mode switch (A6 analog).
    {
      uint16_t mode_raw = analogRead(HW_MODE_SWITCH);
      if      (mode_raw < MODE_THRESH_COOL) g_mode = MODE_COOL;
      else if (mode_raw < MODE_THRESH_AUTO) g_mode = MODE_AUTO;
      else                                  g_mode = MODE_HEAT;
    }

    // Pot -> setpoint (A7).
    {
      uint16_t pot_raw = analogRead(HW_POT);
      g_setpoint = POT_TEMP_MIN + (float)pot_raw * (POT_TEMP_MAX - POT_TEMP_MIN) / 1023.0f;
    }
  }

  // --- Serial command parser (see SerialCmd.h) ---
  serial_poll();

  unsigned long now = millis();

  // -- 100 ms control task ----------------------------------------------
  if (now - g_last100 >= LOOP_INTERVAL_MS) {
    g_last100 = now;
    task_100ms();
  }

  // -- 1 s slow task ----------------------------------------------------
  if (now - g_lastSlow >= 1000) {
    g_lastSlow = now;
    task_slow();
  }
}

// =========================================================================
// 100 ms control task — sense, decide, act.
// Detailed flow is in docs/20260411 - Firmware Flowchart Rev A.md page 3.
// =========================================================================

static void task_100ms() {
  // ---- 0. SCHEDULED CONTROLLER PROFILE ---------------------------------
  // If the operator armed/started a `controller` schedule, fire the
  // user_controller() waypoint once its scheduled time has elapsed.
  // user_controller() may mutate setpoint, PID gains, mode, etc.;
  // the changes propagate to the control decision below. No-op (one
  // bool check) when the schedule is idle.
  ctrl_tick();

  // ---- 1. SENSE ---------------------------------------------------------

  // Read all three NTC channels. t_cold is the one we control on.
  float t_cold = ntc_readAll();
  float t_hot  = ntc_getC(2);
  float t_amb  = ntc_getC(3);

  // Read INA226. Call current FIRST (sets the clipped flag), then
  // voltage and power. If the shunt register is saturated, estimate
  // power from V_bus × TPS I_limit instead of the clipped chip value.
  float ina_i = ina_readCurrentA();
  float ina_v = ina_readVoltageV();
  float ina_p = ina_readPowerW();
  bool  ina_clip = ina_isClipped();
  // When clipped, estimate power from V_bus × TPS current CEILING
  // for the PD budget clamp (worst-case, before we know drive_mA).
  float ina_p_ceiling = ina_clip
      ? ina_estimatePowerW(ina_v, (float)g_imax_mA / 1000.0f)
      : ina_p;

  // ---- 2. FAULT GATE ----------------------------------------------------
  // Check all fault sources. If any fires, latch g_fault and disable
  // the TEC. The fault stays latched until cleared by the enable
  // button or "enable 1" command (which resets g_fault to FAULT_NONE).
  // Only check when enabled — no point faulting when already off.

  // Update HUSB debounce counter every tick (before the fault chain).
  // If voltage drops below 20V, attempt to re-negotiate before counting
  // toward the fault. This handles momentary PD renegotiation events
  // and the bench brownout issue — the chip may recover if we re-request.
#if HAS_HUSB238 && ENABLE_SAFETY_FAULTS
  {
    uint8_t pdv = husb_negotiatedV();
    if (pdv >= 5) {
      // PD source has appeared at least once. From here on we treat
      // PD loss as a real fault. The flag is sticky — unplugging the
      // PD source later does not clear it.
      _husb_was_attached = true;
    }
    if (pdv < 20) {
      if (!_husb_was_attached) {
        // No PD source has ever been seen — the device is running
        // off a direct DC supply (e.g. 24 V into the barrel jack).
        // Losing a contract we never had is not a fault.
        _husb_fault_count = 0;
      } else {
        if (_husb_fault_count == 0) {
          // First tick below 20V — try to re-negotiate immediately.
          _husb_write(HUSB_REG_SRC_PDO,    HUSB_PDO_SEL_20V);
          _husb_write(HUSB_REG_GO_COMMAND, HUSB_CMD_REQUEST_PDO);
        }
        _husb_fault_count++;
      }
    } else {
      _husb_fault_count = 0;
    }
  }
#endif

  // Remember whether we entered this tick with no fault; if a fault
  // latches below, we'll dump full status afterwards so the operator
  // sees the surrounding context (sensor readings, mode, PD state)
  // without having to type `status` manually.
  bool prev_no_fault = (g_fault == FAULT_NONE);

  if (g_enabled && g_fault == FAULT_NONE) {

    // (a) Over-temp safety: trip if ANY connected NTC exceeds g_maxhot.
    //     This is the minimum-protection fault — always compiled in.
    //     NaN comparisons are always false, so unwired channels are
    //     skipped naturally without needing explicit isnan() guards.
    if ((t_cold > g_maxhot) || (t_hot > g_maxhot) || (t_amb > g_maxhot)) {
      g_fault = FAULT_HOT_SIDE;
      g_enabled = false;
#if COMPACT_FAULT_MSGS
      Serial.print(F("FLT[1] ")); Serial.println(g_maxhot, 1);
#else
      Serial.print(F("FAULT[OVERTEMP]: "));
      if      (t_cold > g_maxhot) { Serial.print(F("COLD "));  Serial.print(t_cold, 1); }
      else if (t_hot  > g_maxhot) { Serial.print(F("HOT "));   Serial.print(t_hot, 1); }
      else                        { Serial.print(F("AMB "));   Serial.print(t_amb, 1); }
      Serial.print(F(" C > "));
      Serial.print(g_maxhot, 1); Serial.println(F(" C"));
#endif
    }

#if ENABLE_SAFETY_FAULTS
    // (b) INA226 alert function flag (over-current / over-voltage).
#if HAS_INA226
    else if (ina_alertLatched()) {
      g_fault = FAULT_INA_ALERT;
      g_enabled = false;
#if COMPACT_FAULT_MSGS
      Serial.println(F("FLT[2]"));
#else
      Serial.println(F("FAULT[INA]: alert flag fired"));
#endif
    }
#endif

    // (c) TPS55288 health is monitored via the I2C STATUS register
    //     in task_diag() rather than a discrete fault pin — the chip
    //     handles its own catastrophic shutdowns and current-limit
    //     during normal TEC drive looks identical to a fault on the
    //     PG line, which makes the discrete pin worse than useless.

    // (d) HUSB238 lost 20V PD contract for HUSB_FAULT_DEBOUNCE ticks.
#if HAS_HUSB238
    else if (_husb_fault_count >= HUSB_FAULT_DEBOUNCE) {
      g_fault = FAULT_HUSB_20V;
      g_enabled = false;
#if COMPACT_FAULT_MSGS
      Serial.println(F("FLT[4]"));
#else
      Serial.println(F("FAULT[PD]: lost 20V for 500 ms"));
#endif
    }
#endif

    // (e) Fan tach failure: RPM = 0 while fan should be spinning.
    //     Only check if fan speed is set above the minimum duty.
    //     Skip during the grace period after enable — the tach polling
    //     runs every 1 s and needs time to produce a first valid reading.
    else if (g_fan_speed >= FAN_MIN_DUTY
             && fan_lastRPM() == 0
             && (millis() - _enable_time) > FAULT_GRACE_MS) {
      g_fault = FAULT_FAN_TACH;
      g_enabled = false;
#if COMPACT_FAULT_MSGS
      Serial.println(F("FLT[5]"));
#else
      Serial.println(F("FAULT[FAN]: tach reports 0 RPM"));
#endif
    }
#endif // ENABLE_SAFETY_FAULTS
  }

  // If a fault just latched this tick, dump full status so the
  // operator can see the sensor readings + control state at the
  // moment of the trip without typing `status`.
  if (prev_no_fault && g_fault != FAULT_NONE) {
    serial_print_status();
  }

  // ---- 3. PD POWER-BUDGET CLAMP ----------------------------------------
#if HAS_INA226 && ENABLE_PD_BUDGET_CLAMP
  if (g_pd_budget_mW > 0) {
    float power_mW = ina_p_ceiling * 1000.0f;
    if (power_mW >= (float)g_pd_budget_mW) {
      if (!g_pd_clamped) {
        uint16_t clamp_mV = (uint16_t)(ina_v * 1000.0f);
        uint16_t clamp_mA = (uint16_t)(ina_i * 1000.0f);
        tps_setVoltageLimit(clamp_mV);
        tps_setCurrentLimit(clamp_mA);
        g_pd_clamped = true;
        Serial.print(F("PD CLAMP: "));
        Serial.print(clamp_mV); Serial.print(F(" mV / "));
        Serial.print(clamp_mA); Serial.println(F(" mA"));
      }
    } else {
      g_pd_clamped = false;
    }
  }
#endif

#if ENABLE_DIAGNOSTICS
  // ---- 3b. FAST TPS STATUS POLL (SCP detection) ------------------------
  // Poll the TPS STATUS register at the 100 ms task cadence so a
  // short-circuit event surfaces promptly. The bit is latched in
  // the chip until we read the register, so we don't miss events
  // even if the i2c bus is busy briefly. Heavier presence pings
  // stay in task_diag() (slow task) because their 25 ms timeout-
  // on-missing-chip would blow the 100 ms budget.
  //
  // SCP also LATCHES a firmware fault so we don't re-enable on the
  // next tick after the chip clears its bit. If the underlying
  // short persists, naively re-enabling would just trigger SCP
  // again forever. The operator clears the latch explicitly via
  // the enable button or `enable 1`.
  {
    static bool _scp_last = false;
    uint8_t status = _tps_read(TPS_REG_STATUS);
    bool scp = (status & 0x80) != 0;       // bit 7 = SCP
    if (scp && !_scp_last) {
      g_enabled = false;
      g_fault   = FAULT_TPS_PG;
      Serial.println(F("DIAG: TPS reports SCP -- check TEC or circuit for short"));
      serial_print_status();
    }
    _scp_last = scp;
  }
#endif

  // ---- 4. CONTROL DECISION ---------------------------------------------
  // Two controllers live side-by-side; the user toggles between them
  // via the `pid` console command (also persisted to EEPROM). Both
  // share the same deadband (chatter prevention near setpoint) and
  // both honour the Mode (Cool/Heat/Auto) constraint that gates which
  // direction the TEC may be driven.
  //
  //   PID      → closed-loop, output is PID equation in signed mA.
  //              Anti-windup + saturation handled inside pid_compute.
  //   Bang-bang→ simpler fallback: full current outside damping band,
  //              damping_pct of max inside, off in deadband.

  uint8_t  drive_dir = HB_COOL;          // default direction
  uint16_t drive_mA  = 0;                // 0 = TEC off
  bool     tec_on    = false;

  if (g_enabled && !isnan(t_cold)) {
    float error_pos = t_cold - g_setpoint;     // positive = too hot
    float abs_err   = fabs(error_pos);

    if (abs_err <= g_deadband) {
      // Inside deadband — both controllers agree: TEC off.
      drive_mA = 0;
    }
    else if (g_use_pid) {
      // ---- PID controller ----
      // pid_compute uses the convention error = setpoint - measured
      // → output > 0 means HEAT, output < 0 means COOL.
      const float dt = LOOP_INTERVAL_MS / 1000.0f;
      float out = pid_compute(g_setpoint, t_cold, dt, (float)g_imax_mA);

      // Apply Mode constraint — clamp away the disallowed direction.
      if      (g_mode == MODE_COOL && out > 0) out = 0;
      else if (g_mode == MODE_HEAT && out < 0) out = 0;

      if (out > 0.5f) {
        drive_dir = HB_HEAT;
        drive_mA  = (uint16_t)out;
        tec_on    = true;
      } else if (out < -0.5f) {
        drive_dir = HB_COOL;
        drive_mA  = (uint16_t)(-out);
        tec_on    = true;
      }
      // else: |output| < 0.5 mA → effectively zero → TEC stays off.
    }
    else {
      // ---- Bang-bang + damping (fallback controller) ----
      bool should_drive = false;

      if (g_mode == MODE_COOL) {
        drive_dir = HB_COOL;
        should_drive = (error_pos > 0);   // only cool if too hot
      } else if (g_mode == MODE_HEAT) {
        drive_dir = HB_HEAT;
        should_drive = (error_pos < 0);   // only heat if too cold
      } else {  // MODE_AUTO
        if (error_pos > 0) {
          drive_dir = HB_COOL;
          should_drive = true;
        } else if (error_pos < 0) {
          drive_dir = HB_HEAT;
          should_drive = true;
        }
      }

      if (should_drive) {
        if (abs_err <= g_damping) {
          // Inside damping band — reduce current to percentage of max.
          drive_mA = (uint16_t)((uint32_t)g_imax_mA * g_damping_pct / 100);
          tec_on = true;
        } else {
          // Outside damping band — full current.
          drive_mA = g_imax_mA;
          tec_on = true;
        }
      }
      // else: temp on the wrong side for this mode, or at setpoint.
    }
  }

  // ---- 5. ACTUATE -------------------------------------------------------

  if (tec_on && !g_pd_clamped) {
    tps_setCurrentLimit(drive_mA);
    tps_setVoltageLimit(g_vmax_mV);
    hb_safeDirectionChange(drive_dir);
    tps_setOutput(true);
  } else {
    tps_setOutput(false);
    hb_setDirection(HB_COOL);  // safe default when off
  }

  // Fan runs at the configured speed when enabled. When disabled, it
  // still spins at FAN_DISABLED_SPEED for passive cooling — this
  // prevents heat buildup on a recently-active TEC.
  fan_setSpeed(g_enabled ? g_fan_speed : FAN_DISABLED_SPEED);

  // ---- 6. LED UPDATE ----------------------------------------------------
  // See LedRender.h for the per-LED mapping and override hooks.
  leds_render(t_cold, tec_on, drive_dir);

  // ---- 7. SERIAL PLOTTER ------------------------------------------------
#if ENABLE_SERIAL_PLOTTER
  // Controlled by g_stream (toggled via "stream" command).
  // When streaming: print every PLOTTER_INTERVAL ticks.
  // When not streaming: silent (use "status" for on-demand readout).
  if (g_stream) {
    static uint8_t _plot_count = 0;
    if (++_plot_count >= PLOTTER_INTERVAL) {
      _plot_count = 0;
      Serial.print(F("Setpoint_C:"));  Serial.print(g_setpoint, 1);
      Serial.print(F("\tTcold_C:"));   Serial.print(t_cold, 1);
      Serial.print(F("\tThot_C:"));    Serial.print(t_hot, 1);
      Serial.print(F("\tTamb_C:"));    Serial.print(t_amb, 1);
      Serial.print(F("\tDrive_mA:"));
      // Negative = cooling, positive = heating, 0 = off.
      if (tec_on) {
        int16_t signed_drive = (drive_dir == HB_HEAT)
            ? (int16_t)drive_mA : -(int16_t)drive_mA;
        Serial.print(signed_drive);
      } else {
        Serial.print(0);
      }
      Serial.print(F("\tI_A:"));
      if (ina_clip) Serial.print('>');
      Serial.print(ina_i, 3);
      Serial.print(F("\tV_V:"));       Serial.print(ina_v, 3);
      // Tight power estimate: use drive_mA (actual control output)
      // instead of g_imax_mA (ceiling). More accurate in damping band.
      float ina_p_tight = ina_clip
          ? ina_estimatePowerW(ina_v, (float)drive_mA / 1000.0f)
          : ina_p;
      Serial.print(F("\tP_W:"));
      if (ina_clip) Serial.print('~');
      Serial.print(ina_p_tight, 3);
      Serial.print(F("\tFan_duty:"));  Serial.print(g_fan_speed);
      Serial.print(F("\tFan_RPM:"));   Serial.println(fan_lastRPM());
    }
  }
#endif // ENABLE_SERIAL_PLOTTER
}

// =========================================================================
// 1 s slow task — status summary, HUSB238 PD re-poll, light diagnostics.
// =========================================================================

static void task_slow() {
  // --- Fan tach polling ---
  // fan_pollRPM() blocks for up to 100 ms while it times edges.
  // Running it here (1 s cadence) keeps the 100 ms control task
  // non-blocking. The result is cached in fan_lastRPM() for anyone
  // who needs it between polls.
  fan_pollRPM();

  // OLED status display — see OledRender.h for the layout + edit hooks.
  oled_render();

  // Cross-chip consistency diagnostics — see Diagnostics.h.
  task_diag();
}
