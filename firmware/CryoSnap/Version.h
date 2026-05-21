#ifndef VERSION_H
#define VERSION_H

// Firmware version — update on each meaningful change.
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  7
#define FW_VERSION_PATCH  4
#define FW_VERSION_STR    "0.7.4"

/*
  Changelog (newest first):

  0.7.4  2026-05-21  Direct-supply boot no longer trips PD fault
    - On a bench rig powered from a direct DC supply (e.g. 24 V
      into the barrel jack with no USB-C source attached), the
      HUSB238 reports unattached / 0 V. The FAULT_HUSB_20V chain
      was counting every tick toward the fault and latching
      within 500 ms of the first enable.
    - New sticky `_husb_was_attached` flag flips true the first
      tick HUSB reports any negotiated voltage (>= 5 V). The
      FAULT_HUSB_20V chain now only counts when the flag is set,
      so "never had PD" cannot fault. PD-loss after attach (the
      real failure case) still trips normally.
    - No new config or console command; behaviour is automatic.

  0.7.3  2026-05-11  `controller` one-shot user-function hook
    - Scaffolded "controller" console command that fires
      user_controller(elapsed_s) ONCE at a chosen elapsed time.
      Useful as a demo-prep trigger that adjusts setpoint / PID
      gains / mode at a known time.
    - New Controller.h:
        ctrl_arm()              fire one-shot at t=0 on next enable
        ctrl_start(fire_s)      fire NOW once at elapsed_s = fire_s
        ctrl_stop()             cancel a pending / armed fire
        ctrl_tick()             called every 100 ms from task_100ms;
                                fires when scheduled time reached
        user_controller(t_s)    the customisation point — empty by
                                default, edit in place for demo logic
    - Console:
        controller         arm; fire on next enable button press
        controller <s>     fire NOW at elapsed_s = s
        controller -1      cancel
    - Enable rising edge (button + serial cmd) calls ctrl_on_enable()
      to kick off any armed schedule.
    - One-shot semantics: user_controller runs exactly once per
      command. For multi-step profiles, the body can chain by
      calling ctrl_start(next_s) — example shown in Controller.h.

  0.7.2  2026-05-09  Align temp LED bar with setpoint bar range
    - The temperature bar (LEDs 1-10) used a fixed 0..50 C
      mapping while the setpoint bar already used the pot's
      POT_TEMP_MIN..POT_TEMP_MAX range (-10..+40 C). The temp
      column no longer lined up with NTC1 once the setpoint was
      below zero.
    - Map the temp bar through the same range as the setpoint
      bar so the two bars share a scale. Now you can eyeball
      the offset between cold-side and setpoint directly.

  0.7.1  2026-05-08  OLED: 128x32 panel support
    - OLED_HEIGHT config knob (Config.h) with 32 / 64 branches
      in the SSD1306 init sequence — picks the right MUX (0x1F
      vs 0x3F) and COM-pin config (0x02 vs 0x12). Default is 32
      to match the supplied panel. The previous build sent the
      64-line init to a 32-line panel, which mostly worked but
      drove the lower half of the framebuffer off-screen.
    - OLED_PAGES now derived from OLED_HEIGHT (4 pages on 32px,
      8 on 64px).
    - Render redesigned to a 4-row layout that fits both panels:
        row 0: "v0.7.1 ON Cool"
        row 1: "Set:25.0 T:23.4C"
        row 2: "1.23A 12.0V 14.8W"
        row 3: "Flt:- H:30.1C"
      On a 128x64 panel the bottom 4 rows stay blank.

  0.7.0  2026-05-06  PID controller (selectable at runtime)
    - New PID.h: minimal closed-loop controller, output in signed
      milliamps, anti-windup integrator clamp, no library deps.
      Sign convention: +mA = heat, -mA = cool, |out| = drive_mA.
    - task_100ms branches on g_use_pid: PID path or bang-bang +
      damping. Common deadband + Mode-direction guard apply to
      both. Bang-bang stays available for A/B comparison and as a
      fallback if PID tuning goes sideways.
    - Serial cmds:
        pid           toggle controller (or `pid 0|1` to set)
        kp / ki / kd  show or set tuning constants
      All persisted in EEPROM (SETTINGS_VERSION 2 -> 3, four new
      fields: use_pid, kp, ki, kd). Existing saved settings are
      rejected once on first boot.
    - PID state (integrator + last_err) resets on enable rising
      edge, on Ki change, and on `pid` toggle so a stale residual
      can't kick the TEC at restart.
    - status print now includes "Ctrl:PID/BB" and Kp/Ki/Kd row.
    - Defaults from Config.h: DEFAULT_USE_PID=true, Kp=200,
      Ki=5, Kd=0 (conservative starters; bench tune from there).

  0.6.2  2026-05-06  Latch fault on TPS SCP
    - When the TPS reports SCP (short
      circuit) the firmware now also drops g_enabled and latches
      FAULT_TPS_PG, instead of just printing the DIAG message.
      Reading the STATUS register clears the chip's SCP bit, so
      without latching we'd re-enable on the next tick and the
      chip would re-trip SCP if the short persists — looping
      forever. User has to clear the latch explicitly via the
      enable button or `enable 1`.
    - Status snapshot also auto-prints on the trip, matching the
      other latched-fault paths.

  0.6.1  2026-05-06  Move TPS SCP poll to 100 ms task
    - The lightweight TPS STATUS read for SCP detection moved
      into task_100ms right after the PD-clamp section so a short
      surfaces within 100 ms (was 1 s after the 0.6.0 split).
      Heavier presence pings (HUSB / TPS / INA) stay in task_diag
      at 1 s — their 25 ms timeout-on-missing-chip would blow the
      100 ms task budget if a chip ever drops.

  0.6.0  2026-05-06  Cross-chip diagnostics + serial command UX
    - Serial UX: typing a command without a number now reports the
      current value instead of silently writing zero. Affected:
      set, fan, mode, imax, vmax, maxhot, deadband, damping,
      damping_pct, bright. The `enable` command is special-cased
      to TOGGLE on no-arg (still requires console mode).
    - Read forms are allowed regardless of console mode; only
      writes to enable / set / mode are console-gated.
    - A0 (HW_TPS_FAULT) polling removed entirely —
      the chip handles its own catastrophic shutdowns. The fault
      chain no longer has a TPS arm; OCP_MASK + FB/INT are now
      a no-op for our control flow.
    - New ENABLE_DIAGNOSTICS subsystem (default on) — task_diag()
      runs in the slow task and emits state-change messages:
        * PD ok but TPS not responding (power switch / cable)
        * INA226 offline while TPS responds
        * TPS reports SCP (short circuit on TEC)
        * TPS driving but INA reads ~0 A (disconnected TEC)
      None latch a fault — purely advisory.
    - OLED probe failure now logs `WARN OLED` at boot (matches
      tps/ina/husb pattern). Firmware continues normally without
      a display fitted; every oled_*() call is gated by a runtime
      _oled_present flag set by oled_init().

  0.5.3  2026-05-05  Mask TPS55288 OCP indication on FB/INT
    - Per TPS55288 datasheet §7.6.5, OCP_MASK in the CDC register
      must be 0 around any OE 0->1 transition or the chip pulls
      FB/INT (= HW_TPS_FAULT on Rev A) LOW during the soft-start
      ramp into a load. We use current limit as a *normal*
      operating mode for TEC drive, so OCP_MASK stays cleared
      permanently. SC_MASK and OVP_MASK remain enabled — those
      are real catastrophes that should still surface as
      FAULT[TPS]. CDC register dump now decodes the mask bits.
    - Explains why the FAULT[TPS] trip showed up only on Rev A
      (FB/INT actually wired to A0) and not on the prototype
      (pin floated, internal pullup masked the signal entirely).

  0.5.2  2026-05-04  TPS PG grace + auto status print on fault
    - TPS55288 PG check now skips the first 200 ms after enable so
      the chip's soft-start ramp doesn't latch FAULT[TPS] on the
      very first post-enable tick. Real regulation faults still
      trip within 2-3 ticks once the grace expires.
    - When any fault latches, the full `status` snapshot is now
      printed automatically (sensor readings, mode, PD state).
      Saves typing `status` after every trip during bring-up.

  0.5.1  2026-05-01  OLED + Apr 30 hardware-test fixes merged
    - Brings the 0.4.1 dev fixes onto the OLED branch:
        - Adjustable LED brightness (default 64/255) — `bright`
          serial command, persisted via EEPROM (Settings v2)
        - Setpoint bar now maps the pot's POT_TEMP_MIN..POT_TEMP_MAX
          range to the 10-LED span; full-scale pot lights all 10
        - Wire.setWireTimeout(25 ms, reset) so a missing or stuck
          I2C device no longer freezes the boot scan or driver init
        - PD power-budget headroom raised from 90% to 95%
        - HW_TPS_FAULT now uses INPUT_PULLUP (was floating INPUT)

  0.5.0  2026-04-29  SSD1306 OLED status display
    - Custom framebuffer-free SSD1306 driver in OLED.h (text only,
      5x7 font in PROGMEM, ~150 B RAM total — vs ~1 KB for a full
      framebuffer driver).
    - 8-line status readout refreshed in the 1 s slow task: version,
      mode, setpoint, T_cold, T_hot, I, V, P, fan RPM, fault.
    - HAS_OLED set in both target pin maps (PROTO and Rev A).
    - ENABLE_OLED_DISPLAY flag (default 1) — auto-disables at runtime
      if the panel doesn't ACK at boot, so safe to leave on.
    - OLED_REFRESH_TICKS knob to slow the redraw cadence.
    - Cost when enabled: ~3.3 KB flash, ~10 B RAM.

  0.4.0  2026-04-27  Build-time feature flags + RAM trim + Rev A pin fix
    - ENABLE_* flags in Config.h gate every optional feature
      (debug regs, EEPROM, I2C scan, NTC cal, plotter, LED fade,
      fault LED, PD clamp, verbose boot, safety faults beyond OT)
    - MINIMAL_BUILD master switch: 14.7 KB flash / 664 B RAM
    - COMPACT_FAULT_MSGS prints "FLT[n]" instead of "FAULT[NAME]"
    - SerialCmd strncmp keys moved to PROGMEM (saves 134 B RAM)
    - Rev A: HW_LED_DAT corrected from D7 to D5 per the schematic
    - WS2812B.h derives port bit from HW_LED_DAT (was hard-coded)
    - Per-component memory report in MEMORY_REPORT.md

  0.3.0  2026-04-23  Bench feedback round
    - EEPROM save/load/defaults for persistent settings
    - Over-temp safety now checks all 3 NTCs (was hot-side only)
    - Deadband / damping / damping_pct user-modifiable via serial
    - D8 enable button INPUT_PULLUP (was floating on prototype)

  0.2.0  2026-04-22  Spec-complete + fault recovery
    - Fault LED slow-flash red per spec §24
    - Zero-deadband warning in Config.h per spec §19
    - README.md with implementation note per spec §30
    - PD re-initialization on every enable transition
    - HUSB238 PD fault debounced (5 ticks / 500 ms)
    - HUSB238 auto-renegotiate on first voltage drop
    - Fan tach fault grace period (3 s) after enable
    - INA226 clipping detection with ">" prefix and V*I_limit estimate
    - Plotter fields now include units + signed Drive_mA (cool/heat)
    - NTC calibration commands (cal, cal1, cal2, calshow)
    - Console mode to disable physical inputs
    - H-bridge polarity target-aware (inverted on PROTO)
    - Safe startup: H-bridge pin forced low before anything else
    - TPS I_limit decoded in register dump

  0.1.0  2026-04-21  First functional control pipeline
    - Bang-bang control with deadband (0.2 C) and damping (3.0 C / 50%)
    - NTC 3-channel read with 10-sample running average
    - Serial commands: help, status, enable, set, fan, imax, vmax, mode, maxhot, stream
    - Enable button debounce (D8 active-low) + mode switch (A6 analog)
    - Pot -> setpoint (-10 to 40 C range, always live)
    - WS2812B 23-LED bit-bang driver (AVR inline asm, zero deps)
    - Full LED mapping: temp bar (1-10), setpoint bar (11-20), mode/enable/HB indicators (21-23)
    - Fan 25 kHz hardware PWM (Timer2 OC2B) + polled tach with consistency filter
    - Fan passive cooling at 50% when TEC disabled
    - Hot-side over-temp safety (NTC2 > maxhot -> TEC off)
    - TPS55288 V/I/output control, current limiter ON
    - INA226 current/voltage/power read (100 mOhm shunt, 16A range, CAL=102)
    - HUSB238 USB-PD negotiation (20V/5A, PDO ladder walk, renegotiation on low current)
    - USB-PD power budget protection (90% clamp via INA226)
    - H-bridge direction control with safe direction-change sequence
    - I2C bus scanner with device annotation at boot
    - Register dump for TPS/INA/HUSB (behind DEBUG_DUMP_REGISTERS flag)
    - Dual build target: TARGET_PROTO (bench breadboard) / TARGET_REVA (production PCB)
    - Adafruit HUSB238 debug path (USE_ADAFRUIT_HUSB flag)

  0.0.1  2026-04-16  Skeleton + I2C bring-up
    - Project skeleton: Config.h, Pins.h, driver header stubs
    - I2C scanner confirming TPS55288 @ 0x75, INA226 @ 0x40, HUSB238 @ 0x08
*/

#endif // VERSION_H
