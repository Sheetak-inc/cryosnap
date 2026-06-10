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

  See Pins.h for pin assignments (TARGET_REVA vs TARGET_REVB) and
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

  Example boot output (default build — ENABLE_VERBOSE_BOOT and
  ENABLE_I2C_BOOT_SCAN both default to 0; the version banner is
  emitted regardless of those flags)
  ---------------------------------
    CryoSnap v0.7.11 REVA
    HUSB238: 20V but only 3250 mA, renegotiating...
    HUSB238: after renegotiation: 20V / 3250 mA

  Send "status" over serial to see the full register / sensor /
  control-state snapshot at any time. With
  -DENABLE_VERBOSE_BOOT=1 -DENABLE_I2C_BOOT_SCAN=1 the boot also
  prints an I2C bus walk:
    I2C scan starting...
      found 0x08  (HUSB238)
      found 0x3C  (OLED SSD1306)
      found 0x40  (INA226)
      found 0x75  (TPS55288)
    I2C scan done: 4 device(s) responded.
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
#define FAULT_NO_SUPPLY  6   // TPS auto-reset Vlim under drive — no PD and Vin < ~12 V

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

// Supply-sufficiency check state — used by the periodic V_limit
// readback in task_100ms. See the long comment near the check
// itself for the rationale; declared up here so _pd_reinit() can
// reset everything on every enable rising edge.
#define SUPPLY_VLIM_FLOOR    5500  // mV — V_limit at or below this after a drive attempt = chip auto-reset
#define SUPPLY_FAULT_DEBOUNCE   2  // bad reads anywhere in the drive session before latching
static uint8_t  _supply_fault_count       = 0;
static bool     _tried_drive_last_tick    = false;
// Captured for the FAULT[NOPSU] trip print so the operator can see
// the V_limit value that drove the counter (BUG-003 addendum Fix 4).
// TPS_VLIM_READ_FAIL (0xFFFF) here means the last bad event was a
// NACK during the read burst, not a real low value.
static uint16_t _supply_fault_last_bad_mv = 0;

// Write-NACK debounce. The original BUG-003 addendum fixes the
// SILENT case (writes don't land but the firmware thinks they did)
// by gating _tried_drive_last_tick on write success. That gate
// alone has a secondary failure mode: a chip alive-but-NACKing
// every write leaves _tried_drive_last_tick=false forever, which
// makes the supply check go to its reset branch every tick, which
// means FAULT_NO_SUPPLY can NEVER latch — operator sees enable on,
// fan running, no fault, no drive, no diagnostic.
//
// _tps_write_nack_count tracks consecutive ticks where the actuate
// stage tried to drive but its writes NACK'd. Past
// TPS_NACK_FAULT_DEBOUNCE, the fault chain latches FAULT_NO_SUPPLY
// with a distinct "TPS NACKing" trigger string so the operator
// sees that the supply (not the chip presence, not the V_limit
// reset) is the issue. Also incremented when _tps_only_recover
// fails both attempts — same root cause from a different code
// path. Reset to 0 on any successful actuate-write tick or
// successful _tps_only_recover.
#define TPS_NACK_FAULT_DEBOUNCE 5  // ticks (500 ms) of consecutive NACKs before latch
static uint8_t _tps_write_nack_count = 0;
// Edge-trigger for "TPS dropped off I2C" — distinct failure mode
// from "TPS is alive but supply is insufficient." Set false the
// first tick tps_isPresent() returns false during a drive session;
// re-armed when the chip comes back or _pd_reinit() runs. Keeps the
// DIAG message and recovery attempt one-shot per disappearance
// event instead of spamming once per tick.
static bool    _tps_was_present       = true;

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

#if ENABLE_SOFT_START
// Compile-time guards for the soft-start ramp. Placed here (not in
// Config.h) because the constants they validate — SUPPLY_VLIM_FLOOR
// and FAULT_GRACE_MS — are defined in this file AFTER Config.h is
// included. Putting these checks in Config.h's fallback section would
// see those symbols as undefined (substituted to 0 by the preprocessor)
// and either silently pass or always fire — neither is what we want.
#if SOFT_START_MS == 0
  #error "SOFT_START_MS must be > 0; set ENABLE_SOFT_START=0 to disable the ramp entirely"
#endif
#if SOFT_START_MS >= FAULT_GRACE_MS
  #error "SOFT_START_MS must be < FAULT_GRACE_MS so the supply/fan grace windows cover the full ramp"
#endif

// Soft-start ramp state.
//
//   _ramp_armed     = one-shot "start a new ramp on the next actuating
//                     tick". Set by every site that needs the ramp to
//                     re-fire (operator enable rising edge, chip
//                     re-init via _tps_only_recover, long actuate gap
//                     from deadband). Consumed inside the actuate
//                     write block.
//   _ramp_start_ms  = wall-clock zero of the current ramp, captured
//                     when _ramp_armed is consumed.
//   _last_actuate_ms= timestamp of the previous actuating tick; the
//                     actuate block re-arms the ramp if too much time
//                     has elapsed since (deadband-exit protection).
//
// The flag-based design replaces a v0.7.10 first-pass that compared
// _enable_time != _ramp_for_enable_time as a session key. That
// pattern was vulnerable to millis() rollover, boot-time 0/0
// collision (any auto-enable feature), and silent rename typos. An
// explicit one-shot flag is dumber and safer; the only contract is
// "anywhere you want a ramp, set _ramp_armed=true."
static bool          _ramp_armed       = false;
static unsigned long _ramp_start_ms    = 0;
static unsigned long _last_actuate_ms  = 0;
#endif

// Non-blocking Seebeck wait state machine. Tunables live in Config.h
// (SEEBECK_HB_OFF_BASE_MS / SETTLE_MS / PER_VOLT_MS / MAX_MS); the
// state machine logic lives in task_100ms below.
//
// State machine lives across task_100ms ticks so the scheduler stays
// cooperative: operator commands, fault polls, fan tach, OLED, and
// INA/HUSB monitors all keep their cadence during the up-to-MAX_MS
// wait. Compare to a blocking delay(), which would stall every other
// task until the wait completes.
//
//   SB_IDLE          — no wait active; actuate runs normally.
//   SB_OE_OFF_SETTLE — OE off, waiting SETTLE_MS for the TPS output
//                      cap to equilibrate with the TEC's Seebeck EMF
//                      via the always-enabled DRV8701E H-bridge
//                      MOSFETs (in linear region, low-Rds path
//                      between INA-side rail and TEC terminals)
//                      before we sample INA.
//   SB_OE_OFF_WAIT   — sampled Seebeck once, deadline pushed forward
//                      by the measured-EMF-scaled wait; the polarity
//                      flip happens when this expires.
//
// _have_last_dir is an explicit bool rather than a sentinel-value
// inside _last_drive_dir_exp, so the HB direction-value space stays
// free for future Rev B pin remapping.
#if SEEBECK_HB_OFF_MAX_MS > 0
#define SB_IDLE           0
#define SB_OE_OFF_SETTLE  1
#define SB_OE_OFF_WAIT    2
static uint8_t        _seebeck_state      = SB_IDLE;
static unsigned long  _seebeck_deadline   = 0;
static uint8_t        _last_drive_dir_exp = 0;
static bool           _have_last_dir      = false;
#if SEEBECK_MIN_SAME_DIR_MS > 0
// Timestamp of the previous WAIT->IDLE transition. The SM trigger
// (in SB_IDLE) refuses to re-arm within SEEBECK_MIN_SAME_DIR_MS of
// this timestamp; suppresses limit-cycle flip noise from a controller
// that oscillates around setpoint (bang-bang + Mode Auto being the
// pathological case — engagement-internal bench evidence in the
// seebeck_mode_controller report). _have_last_dir gates the first-
// flip-after-boot case independently, so the initial 0 value here
// has no observable effect at boot.
static unsigned long  _seebeck_last_done_ms = 0;
#endif
#endif

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

// ---- PID dt tracking ----------------------------------------------------
// Wall-clock timestamp of the previous pid_compute() / deadband-observe
// tick. Used by the task_100ms PID branch to derive actual elapsed dt
// instead of trusting LOOP_INTERVAL_MS (BUG-008 fix).
//
// File-static (not inside the PID branch) so it can be reset alongside
// pid_reset() — if pid_compute hasn't run for a while (disable cycle,
// fault, prolonged deadband stay), the next call would otherwise see
// a multi-second raw delta that the [0.05, 0.5] s dt clamp saturates
// to 0.5 s, producing a 5x oversized first integrator step.
// Sentinel 0 means "no prior tick, use the LOOP_INTERVAL_MS nominal."
static unsigned long _pid_last_ms = 0;

// Wrap pid_reset() so we also re-arm the dt sentinel — every site that
// clears PID state should also clear the dt history so the next compute
// starts with a fresh, sane dt.
//
// IMPORTANT: this does NOT clear _tps_write_nack_count. The NACK
// counter is a hardware-health signal, not controller state — it
// must NOT be wiped by operator-controller gestures (pid toggle,
// mode change, even enable rising edge). If we cleared it here:
//   - mode-switch pot noise would call _set_mode -> _pid_full_reset
//     every loop iteration, indefinitely deferring FAULT[NOPSU] latch
//     under a real brown-out.
//   - the enable handler's _pd_reinit -> _tps_only_recover failure
//     leg increment would be wiped on the very next line by this
//     reset, defeating the documented "half second to latch" path.
// Natural clears happen at the right places: actuate-success and
// _tps_only_recover-success both zero the counter; a chip that has
// genuinely recovered will see writes_ok=true on the next actuate
// tick and the counter drops to 0 with no help needed here.
static inline void _pid_full_reset() {
  pid_reset();
  _pid_last_ms = 0;
}

// Mode-change helper. With the new conditional-integration in 0.7.8,
// a non-zero integrator accumulated under one Mode's [out_min, out_max]
// bounds does NOT get drained when the operator switches Modes — it
// just unwinds at err*dt per tick. For the typical bench scenarios
// (~1000 mA·s residual + 10 °C step) this can take a minute or more
// to release, defeating the BUG-002 anti-windup story for cross-mode
// transitions. So: any mode change drops the integrator. The change-
// detection guard avoids resetting every 50 ms when the analog mode
// switch pot sits steady on one threshold.
static inline void _set_mode(uint8_t new_mode) {
  if (new_mode != g_mode) {
    g_mode = new_mode;
    _pid_full_reset();
  }
}

// =========================================================================
// forward declarations for the scheduled tasks
// =========================================================================

static void task_100ms();
static void task_slow();

// Forward declarations — defined below.
// _pd_reinit:        called from SerialCmd.h enable handler + button enable.
// _tps_only_recover: called from Diagnostics.h CDC-drift branch + the
//                    supply check's TPS-absent branch (also from _pd_reinit
//                    itself). Without this forward decl the include of
//                    Diagnostics.h further down would only resolve via
//                    Arduino IDE's auto-prototyping, which is brittle.
static void _pd_reinit();
static void _tps_only_recover();

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

// Lightweight TPS-only recovery — safe to call from inside the 100 ms
// task. Resets the supply-check state and re-runs tps_init() (a few
// I2C writes, no blocking delays). Crucially, does NOT touch HUSB
// state: that stays under the explicit enable-cycle path so a
// concurrent PD drop is not silently swallowed. Caller manages the
// `_tps_was_present` edge flag.
//
// State-clear gating (BUG-003 addendum 2026-06-05): on the SUCCESS
// leg we clear g_pd_clamped, the supply counters, last-bad-mV, and
// the NACK counter — the chip is now at safe defaults via tps_init.
// On the FAILURE leg we touch ONLY two things:
//   (a) increment _tps_write_nack_count so the NOPSU fault chain
//       eventually latches via the NACK path (the operator must
//       see SOMETHING when recovery is broken)
//   (b) clear g_pd_clamped — leaving it set would gate actuate
//       out of writes (gated on !g_pd_clamped) and starve the
//       NACK counter of fresh failed writes, producing the
//       stuck-clamp state the second-round adversarial review
//       flagged.
// _supply_fault_count, _supply_fault_last_bad_mv, and
// _tried_drive_last_tick are preserved on failure so a persistent
// supply problem can re-latch the e1 trigger after FAULT_GRACE_MS.
// Retry once after a 20 ms settle (TPS UVLO de-glitch ~ ms).
static void _tps_only_recover() {
  bool ok = tps_init();
  if (!ok) {
    // 20 ms gives a brown-out / UVLO de-glitch room to settle — the
    // chip's soft-start is several ms and 2 ms wasn't enough on
    // bench. Still well inside any task budget we'd call this from
    // (worst case slow task at 1 s cadence).
    delay(20);
    ok = tps_init();
  }
  if (!ok) {
    // No print here — the chip is broken, increments below propagate
    // to FAULT[NOPSU] "trg NACKx N" with the same information.
    // Increment the persistent-NACK counter so the operator-visible
    // FAULT[NOPSU] latch eventually fires even when the chip is
    // alive-but-NACK. Without this, repeated recovery failures stay
    // silent (the supply-check counter resets every tick because
    // _tried_drive_last_tick is false). NOTE: combined with actuate
    // NACKs on the next tick, total latch latency to FAULT[NOPSU] is
    // a few hundred ms past the FAULT_GRACE_MS window.
    if (_tps_write_nack_count < 0xFF) _tps_write_nack_count++;
    // Also clear g_pd_clamped on the failure leg. If we leave it
    // set, the actuate stage will skip its V/I writes (gated on
    // !g_pd_clamped), starving the NACK counter of fresh failed
    // writes and leaving the operator with a stuck-clamp state
    // they can only clear by power-cycling. Clearing it allows
    // actuate to try fresh writes which either succeed (chip is
    // back, good) or NACK (count keeps climbing to the latch).
    // Print a release message so the operator sees the clamp went
    // away — without this, a CDC-drift recovery at 1 Hz can
    // silently disarm a deliberately-latched clamp.
    // Clear clamp silently — operator sees the trip via FAULT[NOPSU]
    // (NACK counter increments above ensure it fires).
    g_pd_clamped = false;
    // CAVEAT for future maintainers: task_diag's CDC drift check
    // (1 s cadence) only catches the alive-and-CDC-readable case
    // — if the chip is alive enough to ACK its address but also
    // NACKing reads, even that diagnostic path can't recover. The
    // NACK count above is the only path that will surface this
    // state to the operator. Don't tighten the failure leg to
    // skip the counter increment "because task_diag will catch it"
    // — task_diag won't.
    return;
  }
  _supply_fault_count       = 0;
  _supply_fault_last_bad_mv = 0;
  _tried_drive_last_tick    = false;
  _tps_write_nack_count     = 0;
  // Also clear the PD-budget clamp — tps_init() reset the chip's V/I
  // registers, so any clamp the firmware had asserted is stale.
  // Print a release message so the operator sees a deliberately-
  // latched clamp going away (CDC drift in task_diag at 1 Hz can
  // get here without an operator gesture).
  // Clear clamp silently — recovery success is observable via the
  // resumed actuate writes and the absence of further NACK trips.
  g_pd_clamped = false;
#if ENABLE_SOFT_START
  // Re-arm the I_limit ramp: tps_init() just took the chip back to
  // safe defaults and its output cap was likely discharged by the
  // brown-out that triggered recovery. Without this, the next
  // actuate tick would slam the (still-stale) ramp_elapsed-past-
  // window value straight into a freshly-reset chip — the exact
  // BUG-003 Phase-5 inrush at the moment soft-start matters most.
  _ramp_armed = true;
#endif
#if SEEBECK_HB_OFF_MAX_MS > 0
  // Reset Seebeck SM bookkeeping so the next post-recovery polarity
  // flip gets a fresh wait. Without this, _have_last_dir would
  // still hold the pre-recovery direction (matched against current
  // drive_dir as if nothing happened), and the SEEBECK_MIN_SAME_DIR_MS
  // floor would suppress the very flip the wait was designed to
  // protect (the first drive after a chip reset is exactly when the
  // EN/VCC pin is most vulnerable).
  _have_last_dir = false;
#if SEEBECK_MIN_SAME_DIR_MS > 0
  _seebeck_last_done_ms = 0;
#endif
#endif
}

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
  // tps_init + supply-state reset + g_pd_clamped clear. On the
  // failure leg, only g_pd_clamped and _tps_write_nack_count are
  // touched — supply counters and last-bad-mV are preserved so a
  // persistent supply problem correctly re-latches NOPSU after
  // FAULT_GRACE_MS.
  _tps_only_recover();
  _husb_fault_count = 0;
  // Re-arm the sticky "PD ever attached" detector. If the operator
  // is re-enabling after a PD fault (cable knocked loose, source
  // switched out for a direct DC supply), this lets the firmware
  // reassess the supply from scratch instead of carrying the stale
  // "was attached" verdict — which would otherwise re-fault on the
  // next tick if PD is genuinely gone now.
  _husb_was_attached = false;
  // Re-arm the "TPS dropped off I2C" edge trigger. The next
  // disappearance event re-prints DIAG and re-attempts recovery.
  // (_supply_fault_count, _tried_drive_last_tick, g_pd_clamped
  //  are already reset by _tps_only_recover() above.)
  _tps_was_present   = true;
}

// =========================================================================
// Supply-sufficiency check — runs every 100 ms tick whenever the
// firmware just attempted to drive the TEC.
//
// Why a periodic check instead of a synchronous probe at enable: the
// TPS55288 only resets its V_limit to the 5 V safety default once it
// has actually attempted to regulate at the commanded voltage AND
// can't sustain it under real load. A low-current probe at enable
// time doesn't trigger the protection (the chip can hold 12 V into
// any load if there's nothing actually being drawn), so the only
// reliable signal is to observe Vlim while the chip is genuinely
// driving the TEC at the user-configured current.
//
// Mechanism: each tick where we wrote `tps_setVoltageLimit(g_vmax_mV)`
// and turned OE on, the next tick reads V_limit back. If g_vmax_mV
// asked for > 5 V but the chip reset itself to ~5 V, the upstream
// supply is insufficient (no USB-PD AND Vin is missing or below
// ~12 V). The counter is sticky on bad reads while driving (the
// chip oscillates as the firmware re-writes V_limit each tick, so a
// symmetric debounce never accumulates); SUPPLY_FAULT_DEBOUNCE bad
// reads anywhere in the drive session latch FAULT_NO_SUPPLY.
//
// State (SUPPLY_VLIM_FLOOR, SUPPLY_FAULT_DEBOUNCE,
// _supply_fault_count, _tried_drive_last_tick) is declared up in the
// runtime-state block near the top of the file so _pd_reinit() can
// reset both counters on every enable rising edge.
// =========================================================================

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

  // Version banner — always emitted (NOT gated by ENABLE_VERBOSE_BOOT)
  // because operator + log-parsing tools (e.g. cryosnap_monitor) need
  // it to identify the running build.
  Serial.println();
  Serial.print(F("CryoSnap v"));
  Serial.print(F(FW_VERSION_STR));
#if BUILD_TARGET == TARGET_REVB
  Serial.println(F(" REVB"));
#else
  Serial.println(F(" REVA"));
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
              _pid_full_reset();       // clear stale integral + derivative + dt history
              ctrl_on_enable();        // kick off armed schedule, if any
#if ENABLE_SOFT_START
              _ramp_armed = true;      // arm I_limit ramp; consumed at first actuating tick
#endif
#if SEEBECK_HB_OFF_MAX_MS > 0
              // Clear Seebeck direction history + hysteresis floor.
              // The FAULT_GRACE_MS gate on the state machine also
              // covers _have_last_dir during the first 3 s, but the
              // explicit clear keeps the invariant "every state-loss
              // event (operator enable, chip recovery, etc.) resets
              // all SM bookkeeping" symmetric and easy to reason
              // about. Zero flash for the immediate-zero stores
              // (avr-gcc folds them).
              _have_last_dir = false;
#if SEEBECK_MIN_SAME_DIR_MS > 0
              _seebeck_last_done_ms = 0;
#endif
#endif
            }
            Serial.print(F("Enable: "));
            Serial.println(g_enabled ? F("ON") : F("OFF"));
          }
        }
      }
    }

    // Mode switch (A6 analog). Wrapped in _set_mode() so an actual
    // mode change drains the PID integrator (see BUG-002 follow-up
    // in 0.7.8 changelog).
    {
      uint16_t mode_raw = analogRead(HW_MODE_SWITCH);
      uint8_t  new_mode = (mode_raw < MODE_THRESH_COOL) ? MODE_COOL
                       : (mode_raw < MODE_THRESH_AUTO) ? MODE_AUTO
                                                       : MODE_HEAT;
      _set_mode(new_mode);
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

  // Supply-sufficiency tracker. Only meaningful when we actually
  // attempted to drive in the previous tick — that's when the TPS
  // either holds the commanded Vlim or auto-resets to its 5 V
  // safety default (the signal Jon identified on bench 2026-05-22).
  // Skip entirely when the user has configured Vmax at or below
  // the floor; in that range the chip will hold the request
  // regardless of supply, so the "reset to 5 V" signal is moot.
  //
  // Also skip while the PD-budget clamp is latched: the clamp
  // writes its own reduced V_limit (often below 5500 mV when the
  // supply rail had drooped at trip), and reading that back would
  // false-trigger NOPSU within ~SUPPLY_FAULT_DEBOUNCE ticks of
  // the clamp firing. The reduced V_limit during clamp is the
  // firmware's deliberate choice, not a chip-side protection.
#if ENABLE_SAFETY_FAULTS
  // Three exclusions before we even read V_limit back:
  //   - not driving last tick: chip's V_limit is whatever it was
  //     last commanded, not a fresh probe response
  //   - g_vmax_mV <= floor: operator asked for ~5 V which is below
  //     the detection threshold anyway
  //   - g_pd_clamped: the latching PD clamp writes its own reduced
  //     V_limit which can land below the floor when the supply rail
  //     drooped at trip time. Reading that back would false-trigger
  //     FAULT_NO_SUPPLY for the wrong reason. (BUG-004 ↔ NOPSU
  //     interaction caught by the PR-A adversarial review.)
  if (_tried_drive_last_tick && g_vmax_mV > SUPPLY_VLIM_FLOOR && !g_pd_clamped) {
    if (!tps_isPresent()) {
      // TPS chip dropped off the I2C bus — Vin has likely browned
      // out below the chip's UVLO threshold. A direct V_limit read
      // would return 0 mV (Wire.read() on a NACK'd device) and
      // falsely look like "supply insufficient to hold Vmax,"
      // which would latch FAULT_NO_SUPPLY for the wrong reason.
      // Edge-trigger a DIAG message and one recovery attempt per
      // disappearance event; skip the supply-fault counter so the
      // NOPSU detection stays specific to the "chip is alive but
      // says Vlim=5 V" case.
      //
      // IMPORTANT: use the lightweight _tps_only_recover() rather
      // than the full _pd_reinit(). _pd_reinit clears
      // _husb_fault_count and _husb_was_attached, and can block on
      // husb_init() for up to 2 s — both of which are unsafe from
      // the 100 ms task. _tps_only_recover keeps HUSB state intact
      // so a concurrent PD drop is still caught by the regular
      // FAULT_HUSB_20V chain.
      if (_tps_was_present) {
        // Silent reinit — _tps_only_recover increments NACK counter
        // on failure, surfacing eventually via FAULT[NOPSU].
        _tps_only_recover();
        _tps_was_present = false;  // suppress re-fire until chip recovers
      }
    } else {
      // Chip is alive — re-arm the edge so a future disappearance
      // triggers a fresh DIAG + recovery, then check V_limit.
      _tps_was_present = true;
      uint16_t vlim = tps_getVoltageLimitMV();
      if (vlim == TPS_VLIM_READ_FAIL) {
        // One or more of the three reads NACK'd between the
        // tps_isPresent() probe and the data fetch. This is the
        // marginal-Vin transient pattern documented in BUG-003
        // addendum 2026-06-05 — chip is alive but a read failed
        // mid-burst. Without this sentinel we'd decode the zeros
        // as ~200 mV and increment the counter for a non-event.
        // Skip this tick. Do NOT overwrite _supply_fault_last_bad_mv:
        // if the counter was already accumulating from real low
        // readings, the trip print would mislabel the trigger as
        // "(NACK during read burst)" when the actual cause was a
        // legitimate Vlim drop. The per-tick DIAG below still
        // captures NACK occurrences in the serial log for
        // correlation.
      } else if (vlim <= SUPPLY_VLIM_FLOOR) {
        _supply_fault_count++;
        _supply_fault_last_bad_mv = vlim;
        // Log every increment so future audits don't have to guess
        // what V_limit value drove the latch. Bounded by
        // SUPPLY_FAULT_DEBOUNCE (2) outside the post-enable grace
        // window, and by the grace window itself inside it (~30
        // lines worst case at LOOP_INTERVAL_MS=100,
        // FAULT_GRACE_MS=3000). (BUG-003 addendum Fix 4.)
        // No per-tick print — trip-time "trg V=" line records the
        // last bad reading at the latch point, which is enough for
        // field debugging without burning flash on every increment.
      }
      // No reset on a good read while we're driving: the TPS oscillates
      // briefly between "trying to drive at Vmax" and "protected at 5 V"
      // because the firmware re-writes V_limit every tick. A symmetric
      // debounce would never trip — we'd alternate 5 V / 12 V / 5 V /
      // 12 V. Sticky accumulation guarantees that any two bad reads
      // during the drive session (consecutive or not) latch the fault.
    }
  } else {
    // Not driving (or clamp latched, or operator dropped Vmax to the
    // floor) — chip's V_limit reading is meaningless for the supply
    // check. Reset so a fresh enable starts the assessment from
    // zero. Also re-arm the TPS-present edge so a future
    // disappearance during real drive fires a fresh DIAG.
    _supply_fault_count       = 0;
    _supply_fault_last_bad_mv = 0;
    _tps_was_present          = true;
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

    // (c) TPS55288 SCP is monitored via the I2C STATUS register
    //     earlier in this same task (section 3b above) rather than
    //     a discrete fault pin — the chip handles its own
    //     catastrophic shutdowns and current-limit during normal
    //     TEC drive looks identical to a fault on the PG line,
    //     which makes the discrete pin worse than useless. A
    //     latched SCP shows up as FAULT_TPS_PG in the rest of the
    //     fault chain even though it isn't latched here.

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

    // (e) Supply insufficient — two possible triggers, both folded
    //     into FAULT_NO_SUPPLY because the operator-facing remedy is
    //     the same ("check Vin / PD supply"):
    //
    //       (e1) TPS auto-reset V_limit to its 5 V safety default
    //            for SUPPLY_FAULT_DEBOUNCE consecutive ticks of
    //            attempted drive. The Nano-on-PC-USB-only case (no
    //            Vin, no USB-PD) — chip is alive but can't sustain
    //            the commanded voltage.
    //
    //       (e2) TPS write transactions NACK for
    //            TPS_NACK_FAULT_DEBOUNCE consecutive ticks — chip
    //            is alive enough to ACK its address but the
    //            register writes don't land (BUG-003 addendum
    //            follow-up). Catches marginal-Vin brownouts that
    //            the (e1) signal would miss because
    //            _tried_drive_last_tick stays false and the V_limit
    //            counter never advances.
    //
    //     Both check the post-enable grace period (same window the
    //     fan tach check uses) so soft-start + fan spin-up have
    //     room to complete before judging the supply.
    else if (((_supply_fault_count >= SUPPLY_FAULT_DEBOUNCE)
              || (_tps_write_nack_count >= TPS_NACK_FAULT_DEBOUNCE))
             && (millis() - _enable_time) > FAULT_GRACE_MS) {
      g_fault = FAULT_NO_SUPPLY;
      g_enabled = false;
      bool nack_trigger = (_tps_write_nack_count >= TPS_NACK_FAULT_DEBOUNCE);
#if COMPACT_FAULT_MSGS
      if (nack_trigger) {
        Serial.print(F("FLT[6] NACKx")); Serial.println(_tps_write_nack_count);
      } else {
        Serial.print(F("FLT[6] V=")); Serial.println(_supply_fault_last_bad_mv);
      }
#else
      Serial.println(F("FAULT[NOPSU]: no USB-PD or Vin<12V"));
      // BUG-003 addendum Fix 4: print what actually drove the latch.
      // _supply_fault_last_bad_mv is never set to TPS_VLIM_READ_FAIL
      // (the NACK path deliberately leaves it untouched per the
      // comment in section 3a) so we only handle two cases here.
      if (nack_trigger) {
        Serial.print(F("  trg NACKx")); Serial.println(_tps_write_nack_count);
      } else {
        Serial.print(F("  trg V="));   Serial.println(_supply_fault_last_bad_mv);
      }
#endif
    }

    // (f) Fan tach failure: RPM = 0 while fan should be spinning.
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
  // When measured power crosses the PD budget, write reduced V/I
  // limits and LATCH the clamp until the next enable cycle. The
  // latch prevents the off/on oscillation the previous design had:
  // the old code cleared g_pd_clamped as soon as power dipped
  // below budget, which let the actuate block re-write the full
  // drive_mA / g_vmax_mV on the next tick, which pushed power
  // back over budget, which re-fired the clamp. Up to ~10 Hz
  // toggling under the prior design; bench evidence in the
  // 2026-06-03 audit (BUG-004 / C-1).
  //
  // The actuate block in section 5 now respects g_pd_clamped and
  // skips its V/I writes so the reduced limits stick. The clamp
  // releases when the operator re-enables (see _pd_reinit()).
#if HAS_INA226 && ENABLE_PD_BUDGET_CLAMP
  // Also gate on g_enabled so a noisy INA reading during the
  // disabled window can't spuriously latch the clamp. Released
  // automatically on the next _pd_reinit either way.
  if (g_enabled && g_pd_budget_mW > 0 && !g_pd_clamped) {
    float power_mW = ina_p_ceiling * 1000.0f;
    if (power_mW >= (float)g_pd_budget_mW) {
      uint16_t clamp_mV = (uint16_t)(ina_v * 1000.0f);
      uint16_t clamp_mA = (uint16_t)(ina_i * 1000.0f);
      // Check write success before latching. The earlier code
      // discarded both returns and unconditionally set
      // g_pd_clamped=true; if either write NACK'd (the same
      // marginal-Vin brown-out that drives most over-budget
      // events in the first place), the chip stayed at its
      // prior higher V/I limits while the firmware believed it
      // was clamped. With g_pd_clamped=true the actuate stage
      // skips ALL its V/I writes, the supply-Vlim NOPSU branch
      // is gated off (!g_pd_clamped), the NACK counter doesn't
      // climb (no actuate writes happening), and the chip stays
      // at full drive over budget indefinitely. Only latch the
      // clamp when the writes actually landed.
      bool clamp_ok = tps_setVoltageLimit(clamp_mV);
      clamp_ok = tps_setCurrentLimit(clamp_mA) && clamp_ok;
      Serial.print(F("PD CLAMP: "));
      Serial.print(clamp_mV); Serial.print(F(" mV / "));
      Serial.print(clamp_mA); Serial.println(F(" mA"));
      // Latch silently on success; on NACK, leave g_pd_clamped false
      // so actuate keeps re-writing and the NACK counter advances
      // toward the TPS_NACK_FAULT_DEBOUNCE latch.
      if (clamp_ok) g_pd_clamped = true;
    }
  }
#endif

#if ENABLE_SAFETY_FAULTS
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
  //
  // Gated on ENABLE_SAFETY_FAULTS — NOT ENABLE_DIAGNOSTICS — because
  // this is a safety latch, not advisory logging. A build that turns
  // diagnostics off to reclaim flash must still get short-circuit
  // protection. (BUG-001 / S-2 in the 2026-06-03 audit log.)
  //
  // The latch fires only when no other fault is currently active —
  // if OVERTEMP or INA-alert latched earlier in this tick, leave
  // the original cause in g_fault for the operator. _scp_last still
  // updates unconditionally so the chip-level latch state stays
  // tracked across ticks.
  {
    static bool _scp_last = false;
    uint8_t status = _tps_read(TPS_REG_STATUS);
    bool scp = (status & 0x80) != 0;       // bit 7 = SCP
    if (scp && !_scp_last && g_fault == FAULT_NONE) {
      g_enabled = false;
      g_fault   = FAULT_TPS_PG;
      Serial.println(F("FAULT[SCP]: TPS reports short -- check TEC or wiring"));
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
      // Keep the PID derivative-on-measurement history fresh
      // even though we skipped pid_compute() — otherwise leaving
      // the deadband would see a stale _pid_last_measured and
      // produce a one-tick spike proportional to drift. (BUG-007
      // fix is only useful if observe-while-deadband is wired.)
      pid_observe(t_cold);
      // Also advance the dt timestamp so the first compute after
      // a multi-tick deadband stay sees one nominal tick of dt,
      // not the full deadband duration clamped to 0.5 s. Without
      // this the integrator would absorb up to 5x its usual step
      // on the deadband-exit tick.
      _pid_last_ms = millis();
    }
    else if (g_use_pid) {
      // ---- PID controller ----
      // pid_compute uses the convention error = setpoint - measured
      // → output > 0 means HEAT, output < 0 means COOL.
      //
      // dt is measured per-tick (BUG-008): cooperative scheduling
      // means real ticks stretch past LOOP_INTERVAL_MS when other
      // work runs long (~100 ms fan tach poll on the slow task,
      // hb_safeDirectionChange's 5 ms wait, long fault dumps).
      // Stretched ticks would under-integrate and over-differentiate
      // against a fixed dt = 0.1. _pid_last_ms is file-static so
      // _pid_full_reset() clears it alongside the integrator —
      // post-disable, post-deadband, post-fault recovery all start
      // with a fresh dt sentinel rather than a stale wall-clock
      // delta.
      //
      // out_min / out_max express the Mode constraint to
      // pid_compute, so the controller itself knows which
      // direction it can drive (BUG-002): integration is frozen
      // the moment the proposed total output would push past
      // the forbidden bound, leaving no residual that would
      // delay the correct direction once the actuator is free
      // again.
      unsigned long now = millis();
      // _pid_last_ms == 0 sentinel means "no prior tick" (after
      // boot, after _pid_full_reset, after entering this branch
      // for the first time since a deadband/disabled stretch).
      // Fall back to the nominal LOOP_INTERVAL_MS in that case.
      float dt = (_pid_last_ms == 0) ? (LOOP_INTERVAL_MS / 1000.0f)
                                     : (float)(now - _pid_last_ms) / 1000.0f;
      _pid_last_ms = now;
      // Sane bounds against occasional scheduler stall or a long
      // fault dump — the first-call fallback above already protects
      // the initial tick separately.
      if (dt < 0.05f) dt = 0.05f;
      if (dt > 0.5f)  dt = 0.5f;

      float out_min, out_max;
      switch (g_mode) {
        case MODE_COOL: out_min = -(float)g_imax_mA; out_max = 0.0f;            break;
        case MODE_HEAT: out_min = 0.0f;              out_max = (float)g_imax_mA; break;
        default:        out_min = -(float)g_imax_mA; out_max = (float)g_imax_mA; break;  // AUTO
      }
      float out = pid_compute(g_setpoint, t_cold, dt, out_min, out_max);

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
  //
  // Under PD-budget clamp (g_pd_clamped == true), keep OE on but
  // do NOT overwrite the V/I limits — the clamp block above wrote
  // reduced values and we want them to stick. Previously this
  // block always re-wrote the full drive_mA / g_vmax_mV, which
  // defeated the clamp entirely (BUG-004 / C-1 in the 2026-06-03
  // audit log). hb_safeDirectionChange still runs because a
  // direction flip under clamp is still safer with the standard
  // sequence (TPS off -> settle -> toggle -> TPS on).

  // Seebeck mitigation, non-blocking state machine.
  //   (1) on detected direction-change, tps_setOutput(false), enter SETTLE
  //   (2) SETTLE_MS later: read INA bus voltage. The DRV8701E
  //       H-bridge has EN tied high (always enabled), so the commanded
  //       MOSFETs stay in low-Rds linear region and present a near-
  //       zero-resistance bidirectional path between the INA-side rail
  //       and the TEC terminals. With OE off the TPS output cap
  //       equilibrates to |Seebeck EMF| across that path (only a few
  //       mV of MOSFET Rds drop), so V_meas is a direct proxy for
  //       |Seebeck| down to the INA226 ~1.25 mV LSB.
  //   (3) WAIT for BASE + V_meas * PER_V (clamped to MAX_MS) so the
  //       polarity flip happens after the chip has had time to recover
  //       internally from the prior drive — the wait IS the recovery
  //       window, scaled to the prior-drive stress depth via V_meas;
  //       voltage decay itself is not the binding signal (the bus is
  //       pinned near Seebeck EMF by slow thermal-mass relaxation).
  // Does NOT key on setpoint magnitude — heat-removal quality varies
  // (chilled-water loop vs poor heatsink); a customer with great heat
  // removal sees only the BASE_MS blip even on a large setpoint swing.
  // Cooperative across ticks (was a blocking delay() in the v0.7.11
  // first-draft, which broke BUG-008 PID dt accounting, burned
  // FAULT_GRACE_MS, blocked operator-emergency-off for up to 3 s,
  // and starved task_slow). Gated outside FAULT_GRACE_MS so a first-
  // tick flip cannot consume the grace window inside the wait.
#if SEEBECK_HB_OFF_MAX_MS > 0
  {
    unsigned long now_ms = millis();
    switch (_seebeck_state) {
      case SB_OE_OFF_SETTLE:
        if ((long)(now_ms - _seebeck_deadline) >= 0) {
          // ina_readVoltageV scales an unsigned LSB — always >= 0,
          // no abs() needed (verified in INA226.h:168-172).
          float v_meas = ina_readVoltageV();
          uint32_t scaled_ms = (uint32_t)(v_meas * SEEBECK_HB_OFF_PER_VOLT_MS);
          uint32_t wait_ms   = SEEBECK_HB_OFF_BASE_MS + scaled_ms;
          if (wait_ms > SEEBECK_HB_OFF_MAX_MS) wait_ms = SEEBECK_HB_OFF_MAX_MS;
          _seebeck_deadline = now_ms + wait_ms;
          _seebeck_state    = SB_OE_OFF_WAIT;
#if ENABLE_SEEBECK_TRACE
          Serial.print(F("DIAG: Seebeck V="));
          Serial.print(v_meas, 2);
          Serial.print(F(" wait="));
          Serial.print(wait_ms);
          Serial.println(F("ms"));
#endif
        }
        break;
      case SB_OE_OFF_WAIT:
#if ENABLE_SEEBECK_TRACE
        {
          // Per-tick EMF trace so the decay curve is visible in the
          // serial log. INA is on a different I2C address from TPS;
          // sampling it during the wait does not perturb the recovery.
          Serial.print(F("DIAG: Seebeck t V="));
          Serial.println(ina_readVoltageV(), 2);
        }
#endif
        if ((long)(now_ms - _seebeck_deadline) >= 0) {
#if SEEBECK_MIN_SAME_DIR_MS > 0
          _seebeck_last_done_ms = now_ms;      // start hysteresis floor
#endif
          _seebeck_state   = SB_IDLE;          // actuate runs this tick
          // BUG-008 protection: next pid_compute would otherwise see
          // ~wait_ms as one giant dt, get clamped to 0.5 s, and 5x-
          // oversize its integrator step. Sentinel = 0 → first-call
          // fallback at line ~1287 uses LOOP_INTERVAL_MS for one tick.
          _pid_last_ms = 0;
#if ENABLE_SOFT_START
          // Suppress soft-start gap-based re-arm. A polarity flip on
          // the same TEC is not an inrush event (same load, opposite
          // polarity), so the ramp should NOT fire here. Without this
          // the gap check would re-arm and the controller would see
          // a sluggish step response post-flip.
          _last_actuate_ms = now_ms;
#endif
          // Record new direction so the SB_IDLE history-update below
          // does not re-trigger the state machine on this same tick.
          if (tec_on) {
            _last_drive_dir_exp = drive_dir;
            _have_last_dir      = true;
          }
        }
        break;
      case SB_IDLE:
      default:
        // Detect direction-change → enter SETTLE. Triggers only when:
        //   - controller actually wants to drive this tick (tec_on),
        //   - we have a recorded prior driving direction,
        //   - new direction differs from the recorded one,
        //   - we are past FAULT_GRACE_MS — without this a first-tick
        //     flip burns the grace window inside the wait and a
        //     spurious FAULT[FAN] / FAULT[NOPSU] latches next tick,
        //   - we are past the SEEBECK_MIN_SAME_DIR_MS hysteresis
        //     floor since the last wait completed — suppresses
        //     limit-cycle flip noise (a controller oscillating around
        //     setpoint after the previous wait has already protected
        //     against the polarity-flip transient; firing again
        //     within ~1 s adds OE-off duty cycle without safety
        //     benefit).
        if (tec_on
            && _have_last_dir
            && drive_dir != _last_drive_dir_exp
            && (now_ms - _enable_time) > FAULT_GRACE_MS
#if SEEBECK_MIN_SAME_DIR_MS > 0
            && (now_ms - _seebeck_last_done_ms) > SEEBECK_MIN_SAME_DIR_MS
#endif
            ) {
          // OE-off NACK: chip is still driving in the OLD direction.
          // Sampling INA at +20 ms would give the regulated output
          // (5-15 V), not Seebeck — wait_ms would saturate to MAX
          // and the TEC would be driven the wrong way for the full
          // freeze. Skip the dance on NACK; normal actuate retries
          // and its NACKs accumulate _tps_write_nack_count.
          if (tps_setOutput(false)) {
            _seebeck_deadline      = now_ms + SEEBECK_HB_OFF_SETTLE_MS;
            _seebeck_state         = SB_OE_OFF_SETTLE;
            _tried_drive_last_tick = false;  // chip is OE-off during wait
          }
        }
        // Record current direction only on active-drive ticks.
        // A deadband tick leaves drive_dir at its default (HB_COOL),
        // which would corrupt history and trigger a spurious wait on
        // the next active drive (dominant user-visible misbehaviour
        // of the v0.7.11 first-draft near setpoint).
        if (tec_on) {
          _last_drive_dir_exp = drive_dir;
          _have_last_dir      = true;
        }
        break;
    }
    // Safety: if g_enabled went false (operator off or fault latch)
    // while we were waiting, drop the state so the else-branch below
    // can run normal disable cleanup. The chip is already OE-off from
    // the SETTLE entry, so no additional write is needed here.
    if (!g_enabled) _seebeck_state = SB_IDLE;
  }
  if (_seebeck_state == SB_IDLE)
#endif
  {
  if (tec_on) {
    // Track whether the V/I writes actually landed. If they NACK
    // (marginal Vin during high-current drive), the chip is still
    // at whatever it was previously holding — leaving
    // _tried_drive_last_tick true here would let the next supply
    // check mis-interpret a stale-but-not-our-fault V_limit reading
    // as "we asked the chip to hold Vmax." (BUG-003 addendum Fix 1
    // recommendation.) hb_safeDirectionChange is internal to
    // HBridge.h and doesn't report its internal TPS off/on success
    // — that's a deliberate contract gap; the actuate writes here
    // are the ones that matter for the supply check.
    bool writes_ok = true;
    if (!g_pd_clamped) {
#if ENABLE_SOFT_START
      // Soft-start I_limit ramp. Lives above the controllers so PID
      // and bang-bang get identical inrush protection without either
      // needing knowledge of the ramp. V_limit is NOT ramped — for a
      // TEC (low-impedance + Seebeck EMF) the I_limit is the binding
      // constraint; V_limit is a ceiling that never engages during
      // drive on this load type.
      //
      // Ramp re-arm triggers (any sets _ramp_armed=true):
      //   (a) operator enable rising edge (button/serial enable
      //       handlers — see CryoSnap.ino:733 / SerialCmd.h:258)
      //   (b) _tps_only_recover success leg — chip just took fresh
      //       defaults, output cap probably discharged by the
      //       brown-out that triggered recovery
      //   (c) actuate gap >= SOFT_START_RESET_GAP_MS — deadband-exit
      //       protection. Brief inter-tick jitter passes; multi-tick
      //       deadband stays during normal thermostat operation
      //       re-arm the ramp so the next OE off->on doesn't slam.
      //
      // Monotonic ceiling formula (NOT a conditional scale on
      // drive_mA): ramp_cap_mA grows linearly from SOFT_START_I_MA up
      // to g_imax_mA over SOFT_START_MS, then sticks at g_imax_mA.
      // ramp_mA = min(drive_mA, ramp_cap_mA). Avoids the dithering
      // hazard where a controller oscillating across the
      // SOFT_START_I_MA threshold (small PID near steady-state, or BB
      // damping band) would produce non-monotonic I_limit register
      // writes — chip's IOUT_LIMIT must be monotonically increasing
      // during the ramp regardless of controller activity.
      //
      // uint32_t cast prevents 16-bit overflow at full scale
      // (worst: ~5800 mA * 600 ms = 3.5e6, fits in uint32).
      unsigned long now_ms = millis();
      if (_ramp_armed ||
          (now_ms - _last_actuate_ms) > SOFT_START_RESET_GAP_MS) {
        _ramp_start_ms = now_ms;
        _ramp_armed    = false;
      }
      uint16_t ramp_mA = drive_mA;
      unsigned long ramp_elapsed = now_ms - _ramp_start_ms;
      if (ramp_elapsed < SOFT_START_MS && g_imax_mA > SOFT_START_I_MA) {
        uint16_t ramp_cap_mA = SOFT_START_I_MA +
          (uint16_t)(((uint32_t)(g_imax_mA - SOFT_START_I_MA) * ramp_elapsed) / SOFT_START_MS);
        if (ramp_mA > ramp_cap_mA) ramp_mA = ramp_cap_mA;
      }
      _last_actuate_ms = now_ms;
      writes_ok &= tps_setCurrentLimit(ramp_mA);
      writes_ok &= tps_setVoltageLimit(g_vmax_mV);
      // Mutate drive_mA so the stream telemetry below prints the
      // value actually written (not the controller's commanded
      // value). Operator-visible plotter then shows the ramp shape;
      // without this the plotter shows drive_mA=full while INA shows
      // ramped current — diagnostic-confusing.
      drive_mA = ramp_mA;
#else
      writes_ok &= tps_setCurrentLimit(drive_mA);
      writes_ok &= tps_setVoltageLimit(g_vmax_mV);
#endif
    }
    hb_safeDirectionChange(drive_dir);
    writes_ok &= tps_setOutput(true);
    _tried_drive_last_tick = writes_ok;   // arm the supply-Vlim check for next tick
    // Persistent-NACK tracking (BUG-003 addendum follow-up): if
    // writes_ok stayed false for TPS_NACK_FAULT_DEBOUNCE consecutive
    // ticks, the fault chain latches NOPSU below. Without this, the
    // supply check (gated on _tried_drive_last_tick) goes to its
    // reset branch every tick and the counter never accumulates —
    // an alive-but-NACK chip would stay silent forever.
    if (writes_ok) {
      _tps_write_nack_count = 0;
    } else if (_tps_write_nack_count < 0xFF) {
      _tps_write_nack_count++;
    }
  } else {
    tps_setOutput(false);
    hb_setDirection(HB_COOL);        // safe default when off
    _tried_drive_last_tick = false;  // chip's Vlim reading is meaningless when not driving
    // NOTE: _tps_write_nack_count is NOT cleared here. Earlier
    // revisions did, on the theory "no drive intent → no NACK
    // pressure to track". But the deadband path enters this else
    // branch every tick the |pid output| is below the 0.5 mA
    // threshold or |error| <= deadband — and a controller that
    // oscillates in and out of deadband with a chip alive-but-
    // persistently-NACKing would wipe the watchdog every other
    // tick, indefinitely deferring the FAULT[NOPSU] latch the
    // counter was added to surface. The counter is hardware-
    // health state, not controller-intent state; only natural
    // success paths (a writes_ok=true actuate tick, or a
    // _tps_only_recover success leg) drop it back to zero.
  }
  }  // _seebeck_state == SB_IDLE — when non-IDLE, the chip is OE-off
     // and actuate writes are suppressed until the wait deadline elapses.

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
