#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Config.h"

/*
  SerialCmd.h — serial command parser.

  Fixed 32-byte buffer, no String class, memory-safe. Drains Serial
  RX each loop iteration; dispatches on newline or CR.

  Built-in commands (use `help` at the prompt for the live list):
    help          show available commands
    status        print runtime state + sensor readings
    console       toggle console mode (gates writes to enable/set/mode)
    enable [0|1]  toggle or set TEC drive (mirrors enable button)
    set <C>       set target cold-side temp (e.g. "set 25.0")
    fan <0-255>   set fan duty (applied immediately)
    imax <mA>     set TPS55288 current limit (applied immediately)
    vmax <mV>     set TPS55288 voltage limit (applied immediately)
    mode <0-2>    force mode (0=Cool 1=Heat 2=Auto)
    bright <0-255> WS2812 master brightness
    pid [0|1]     toggle / set controller (PID vs bang-bang)
    kp/ki/kd <v>  PID tuning constants
    controller [s] one-shot user_controller(elapsed_s) trigger
    save / load / defaults    EEPROM persistence

  Argument convention (since 0.4.x): a numeric command without an
  argument REPORTS the current value instead of writing zero. So
  `fan` alone is a read; `fan 200` is a write. The exception is
  `enable`, which toggles on no-arg.

  Per-write values are volatile until `save` writes them to EEPROM;
  `defaults` wipes the EEPROM and restores Config.h values.

  ---------------------------------------------------------------
  HOW TO ADD A NEW COMMAND
  ---------------------------------------------------------------
  1. Pick a unique prefix (no other command starts with the same
     letters). Order matters — longer prefixes must be checked
     BEFORE shorter ones (see how `damping_pct` is matched before
     `damping`, and `cal1`/`cal2`/`calshow` before `cal`).
  2. Add an else-if branch to _cmd_execute() following the existing
     pattern:
       else if (MATCH("myname", N)) {
         if (_has_arg(cmd + N)) {
           // parse + write — atoi / atof for numeric args
         }
         Serial.print(F("Mine=")); Serial.println(<current value>);
       }
     The MATCH macro is strncmp_P-backed so the string lives in
     PROGMEM and costs zero RAM.
  3. If the command writes a piece of state, add the new global to
     CryoSnap.ino's runtime-state block AND to SavedSettings
     in Settings.h (see the recipe at the top of that file).
  4. Add a one-line description to the `help` listing below.

  IMPORTANT: this header references runtime state variables
  (g_enabled, g_mode, g_setpoint, etc.) declared in the main .ino.
  It must be #included AFTER those declarations — see the comment at
  the #include site in CryoSnap.ino.
*/

// ---- buffer state -------------------------------------------------------
#define CMD_BUF_SIZE 32
static char    _cmd_buf[CMD_BUF_SIZE];
static uint8_t _cmd_len = 0;

#if ENABLE_NTC_CALIBRATION
// Two-point NTC calibration state (persists between cal1 and cal2 calls).
static float _cal1_raw  = 0;
static float _cal1_temp = 0;
#endif

static void _cmd_execute(char* cmd);

// ---- public API ---------------------------------------------------------

// Print a human-readable status snapshot — PD, INA readings, and all
// runtime control state. Called at boot and by the "status" command.
inline void serial_print_status() {
  Serial.println(F("--- status ---"));

#if HAS_HUSB238
  uint8_t  pd_v  = husb_negotiatedV();
  uint16_t pd_mA = husb_maxCurrentMA();
  Serial.print(F("PD:"));
  if (pd_v == 0) {
    Serial.println(F(" none (check cable/source or using screw terminal)"));
  } else {
    Serial.print(' '); Serial.print(pd_v); Serial.print(F("V/"));
    Serial.print(pd_mA); Serial.print(F("mA"));
    if (pd_v < 20)   Serial.print(F(" [want 20V]"));
    if (pd_mA < 5000) Serial.print(F(" [want 5A]"));
    Serial.println();
  }
#endif

#if HAS_INA226
  {
    float i = ina_readCurrentA();
    float v = ina_readVoltageV();
    float p = ina_readPowerW();
    bool  clip = ina_isClipped();
    Serial.print(F("INA: V=")); Serial.print(v, 3);
    Serial.print(F(" I="));
    if (clip) Serial.print('>');
    Serial.print(i, 3);
    Serial.print(F(" P="));
    if (clip) {
      Serial.print('>');
      Serial.print(ina_estimatePowerW(v, (float)g_imax_mA / 1000.0f), 3);
      Serial.print(F(" [clip@")); Serial.print(ina_maxMeasurable(), 1);
      Serial.println(F("A]"));
    } else {
      Serial.println(p, 3);
    }
  }
#endif

  Serial.print(F("En:")); Serial.print(g_enabled ? F("1") : F("0"));
  Serial.print(F(" Flt:"));
  switch (g_fault) {
    case FAULT_NONE:      Serial.print(F("-"));       break;
    case FAULT_HOT_SIDE:  Serial.print(F("OVERTEMP")); break;
    case FAULT_INA_ALERT: Serial.print(F("INA"));      break;
    case FAULT_TPS_PG:    Serial.print(F("TPS"));      break;
    case FAULT_HUSB_20V:  Serial.print(F("PD"));       break;
    case FAULT_FAN_TACH:  Serial.print(F("FAN"));      break;
    case FAULT_NO_SUPPLY: Serial.print(F("NoPSU"));    break;
    default:              Serial.print(g_fault);       break;
  }
  Serial.print(F(" Mode:"));
  Serial.println(g_mode == MODE_COOL ? F("Cool") :
                 g_mode == MODE_HEAT ? F("Heat") : F("Auto"));

  Serial.print(F("SP=")); Serial.print(g_setpoint, 1);
  Serial.print(F("C Imax=")); Serial.print(g_imax_mA);
  Serial.print(F("mA Vmax=")); Serial.print(g_vmax_mV);
  Serial.print(F("mV Fan=")); Serial.print(g_fan_speed);
  Serial.print(F(" RPM=")); Serial.println(fan_lastRPM());

  Serial.print(F("MaxHot=")); Serial.print(g_maxhot, 1);
  Serial.print(F("C DB=")); Serial.print(g_deadband, 2);
  Serial.print(F("C Damp=")); Serial.print(g_damping, 2);
  Serial.print(F("C/")); Serial.print(g_damping_pct); Serial.print('%');
  Serial.print(F(" Br=")); Serial.println(g_led_brightness);

  Serial.print(F("Ctrl:")); Serial.print(g_use_pid ? F("PID") : F("BB"));
  Serial.print(F(" Kp=")); Serial.print(pid_getKp(), 2);
  Serial.print(F(" Ki=")); Serial.print(pid_getKi(), 2);
  Serial.print(F(" Kd=")); Serial.println(pid_getKd(), 2);

  Serial.print(F("NTC: C=")); Serial.print(ntc_getC(1), 1);
  Serial.print(F(" H=")); Serial.print(ntc_getC(2), 1);
  Serial.print(F(" A=")); Serial.println(ntc_getC(3), 1);

  Serial.print(F("Stream:")); Serial.print(g_stream ? F("1") : F("0"));
  Serial.print(F(" Console:")); Serial.println(g_console ? F("1") : F("0"));
  Serial.println(F("--------------"));

  // Verbose register dump — controlled by DEBUG_DUMP_REGISTERS in
  // Config.h. Set to 0 for production to save ~2.6 KB flash.
#if DEBUG_DUMP_REGISTERS
  Serial.println(F("--- registers ---"));
  tps_dump(Serial);
  ina_dump(Serial);
  husb_dump(Serial);
  Serial.println(F("--------------"));
#endif
}

// Called every loop iteration — drains Serial RX into the buffer.
// On newline (or CR), null-terminates and dispatches.
inline void serial_poll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (_cmd_len > 0) {
        _cmd_buf[_cmd_len] = '\0';
        _cmd_execute(_cmd_buf);
        _cmd_len = 0;
      }
    } else if (_cmd_len < CMD_BUF_SIZE - 1) {
      _cmd_buf[_cmd_len++] = c;
    }
  }
}

// ---- command dispatch ----------------------------------------------------

// MATCH(literal, n) — true if cmd starts with literal (n chars).
// The literal lives in PROGMEM via PSTR; strncmp_P reads it from
// flash. This avoids per-key RAM cost (was ~150 B in .data).
#define MATCH(LIT, N)  (strncmp_P(cmd, PSTR(LIT), (N)) == 0)

// Returns true if there is at least one non-space character after
// the command word (i.e. an argument was supplied). Used so that
// `fan` alone reports the current value, while `fan 200` writes it.
// `enable` is special-cased to toggle on no-arg (handled in the
// dispatcher itself, not here).
static inline bool _has_arg(const char* p) {
  while (*p == ' ') p++;
  return *p != '\0';
}

static void _cmd_execute(char* cmd) {
  // Skip leading whitespace.
  while (*cmd == ' ') cmd++;

  if (MATCH("help", 4)) {
    // Compact help — one-line entries.
    Serial.println(F("Commands:"));
    Serial.println(F("help status console"
#if ENABLE_SERIAL_PLOTTER
                     " stream"
#endif
#if ENABLE_EEPROM_SETTINGS
                     " save load defaults"
#endif
                     ));
    Serial.println(F("enable <0|1>  set <C>  fan <0-255>  mode <0-2>"));
    Serial.println(F("bright <0-255>  imax <mA>  vmax <mV>  maxhot <C>"));
    Serial.println(F("deadband <C>  damping <C>  damping_pct <0-100>"));
    Serial.println(F("pid <0|1>  kp <v>  ki <v>  kd <v>"));
    Serial.println(F("controller [s]    one-shot user fn (-1 cancels)"));
#if ENABLE_NTC_CALIBRATION
    Serial.println(F("cal <C> | cal1 <C> | cal2 <C> | calshow"));
#endif
  }
  else if (MATCH("status", 6)) {
    serial_print_status();
  }
  else if (MATCH("console", 7)) {
    g_console = !g_console;
    Serial.print(F("Console:")); Serial.println(g_console ? '1' : '0');
  }
  // Commands that conflict with physical inputs require console mode
  // ONLY when they're attempting to write. Reads (no-arg form) are
  // always allowed regardless of console mode. `enable` always writes
  // (toggle if no arg, set if arg present), so it's gated unconditionally.
  else if ((MATCH("enable", 6)
         || (MATCH("set", 3) && _has_arg(cmd + 3))
         || (MATCH("mode", 4) && _has_arg(cmd + 4)))
         && !g_console) {
    Serial.println(F("ERR: enable 'console' mode first"));
  }
  else if (MATCH("enable", 6)) {
    g_fault = FAULT_NONE;
    if (_has_arg(cmd + 6)) {
      g_enabled = (atoi(cmd + 6) != 0);
    } else {
      g_enabled = !g_enabled;   // toggle
    }
    if (g_enabled) {
      _enable_time = millis();
      _pd_reinit();
      _pid_full_reset();        // clear stale integral + derivative + dt history
      ctrl_on_enable();         // kick off armed schedule, if any
    }
    Serial.print(F("Enable:")); Serial.println(g_enabled ? '1' : '0');
  }
  else if (MATCH("set", 3)) {
    if (_has_arg(cmd + 3)) g_setpoint = atof(cmd + 3);
    Serial.print(F("SP=")); Serial.println(g_setpoint, 1);
  }
  else if (MATCH("fan", 3)) {
    if (_has_arg(cmd + 3)) {
      int v = atoi(cmd + 3);
      if (v < 0) v = 0; else if (v > 255) v = 255;
      g_fan_speed = (uint8_t)v;
      fan_setSpeed(g_fan_speed);
    }
    Serial.print(F("Fan=")); Serial.println(g_fan_speed);
  }
  else if (MATCH("bright", 6)) {
    if (_has_arg(cmd + 6)) {
      int v = atoi(cmd + 6);
      if (v < 0) v = 0; else if (v > 255) v = 255;
      g_led_brightness = (uint8_t)v;
    }
    Serial.print(F("Bright=")); Serial.println(g_led_brightness);
  }
  else if (MATCH("imax", 4)) {
    if (_has_arg(cmd + 4)) {
      // Clamp BOTH ends before the uint16_t cast. atoi returns int;
      // a bare cast on out-of-range positives wraps silently —
      // imax 70000 became 4464 mA in the old code (BUG-009 / C-2
      // in the 2026-06-03 audit). Upper bound is the TPS55288's
      // encodable ceiling: 0.0635 V / 10 mOhm shunt = 6350 mA.
      long v = atol(cmd + 4);
      if (v < 0)    v = 0;
      if (v > 6350) v = 6350;
      g_imax_mA = (uint16_t)v;
      tps_setCurrentLimit(g_imax_mA);
    }
    Serial.print(F("Imax=")); Serial.println(g_imax_mA);
  }
  else if (MATCH("vmax", 4)) {
    if (_has_arg(cmd + 4)) {
      // Same wrap protection as imax. tps_setVoltageLimit also
      // clamps internally to 20000 mV, but the unclamped value
      // would still land in g_vmax_mV and persist through save.
      long v = atol(cmd + 4);
      if (v < 0)     v = 0;
      if (v > 20000) v = 20000;
      g_vmax_mV = (uint16_t)v;
      tps_setVoltageLimit(g_vmax_mV);
    }
    Serial.print(F("Vmax=")); Serial.println(g_vmax_mV);
  }
  else if (MATCH("mode", 4)) {
    if (_has_arg(cmd + 4)) {
      int v = atoi(cmd + 4);
      if (v >= 0 && v <= 2) _set_mode((uint8_t)v);
    }
    Serial.print(F("Mode:"));
    Serial.println(g_mode == MODE_COOL ? F("Cool") :
                   g_mode == MODE_HEAT ? F("Heat") : F("Auto"));
  }
#if ENABLE_NTC_CALIBRATION
  else if (MATCH("calshow", 7)) {
    Serial.print(F("scale=")); Serial.print(ntc_getScale(), 6);
    Serial.print(F(" off=")); Serial.println(ntc_getOffset(), 2);
    for (uint8_t i = 1; i <= 3; ++i) {
      Serial.print(F("NTC")); Serial.print(i);
      Serial.print(F(" raw=")); Serial.print(ntc_getRawAvg(i), 1);
      Serial.print(F(" T=")); Serial.println(ntc_getC(i), 1);
    }
  }
  else if (MATCH("cal1", 4) && cmd[4] == ' ') {
    _cal1_raw  = ntc_getRawAvg(1);
    _cal1_temp = atof(cmd + 5);
    Serial.print(F("P1 raw=")); Serial.print(_cal1_raw, 1);
    Serial.print(F(" T=")); Serial.println(_cal1_temp, 1);
  }
  else if (MATCH("cal2", 4) && cmd[4] == ' ') {
    float raw2  = ntc_getRawAvg(1);
    float temp2 = atof(cmd + 5);
    float dRaw = raw2 - _cal1_raw;
    if (fabs(dRaw) < 10) {
      Serial.println(F("ERR: points too close"));
    } else {
      float new_scale  = (temp2 - _cal1_temp) / dRaw;
      float new_offset = _cal1_temp - _cal1_raw * new_scale;
      ntc_setScale(new_scale);
      ntc_setOffset(new_offset);
      Serial.print(F("scale=")); Serial.print(new_scale, 6);
      Serial.print(F(" off=")); Serial.println(new_offset, 2);
    }
  }
  else if (MATCH("cal", 3) && cmd[3] == ' ') {
    float actual = atof(cmd + 4);
    float raw    = ntc_getRawAvg(1);
    float new_offset = actual - raw * ntc_getScale();
    ntc_setOffset(new_offset);
    Serial.print(F("off=")); Serial.println(new_offset, 2);
  }
#endif // ENABLE_NTC_CALIBRATION
  else if (MATCH("maxhot", 6)) {
    if (_has_arg(cmd + 6)) g_maxhot = atof(cmd + 6);
    Serial.print(F("MaxHot=")); Serial.println(g_maxhot, 1);
  }
  else if (MATCH("damping_pct", 11)) {
    if (_has_arg(cmd + 11)) {
      int v = atoi(cmd + 11);
      if (v < 0) v = 0; else if (v > 100) v = 100;
      g_damping_pct = (uint8_t)v;
    }
    Serial.print(F("Dpct=")); Serial.println(g_damping_pct);
  }
  else if (MATCH("damping", 7)) {
    if (_has_arg(cmd + 7)) {
      g_damping = atof(cmd + 7);
      if (g_damping < 0) g_damping = 0;
    }
    Serial.print(F("Damp=")); Serial.println(g_damping, 2);
  }
  // ---- PID controller selection + tuning ---------------------------------
  // `pid`            -> toggle between PID and bang-bang+damping
  // `pid 0|1`        -> set explicitly
  // `kp / ki / kd`   -> show or set tuning constants
  else if (MATCH("pid", 3)) {
    if (_has_arg(cmd + 3)) {
      g_use_pid = (atoi(cmd + 3) != 0);
    } else {
      g_use_pid = !g_use_pid;
    }
    _pid_full_reset();   // both directions of toggle benefit from a clean integrator + dt history
    Serial.print(F("PID:")); Serial.println(g_use_pid ? '1' : '0');
  }
  else if (MATCH("kp", 2)) {
    if (_has_arg(cmd + 2)) pid_setKp(atof(cmd + 2));
    Serial.print(F("Kp=")); Serial.println(pid_getKp(), 4);
  }
  else if (MATCH("ki", 2)) {
    if (_has_arg(cmd + 2)) pid_setKi(atof(cmd + 2));   // also resets integrator
    Serial.print(F("Ki=")); Serial.println(pid_getKi(), 4);
  }
  else if (MATCH("kd", 2)) {
    if (_has_arg(cmd + 2)) pid_setKd(atof(cmd + 2));
    Serial.print(F("Kd=")); Serial.println(pid_getKd(), 4);
  }
  // ---- Scheduled controller fire (one-shot) -----------------------------
  // `controller`        arm; fire user_controller(0) on next enable press
  // `controller <s>`    fire NOW at elapsed_s = s
  // `controller -1`     cancel any armed / pending fire
  else if (MATCH("controller", 10)) {
    if (_has_arg(cmd + 10)) {
      float s = atof(cmd + 10);
      if (s < 0) {
        ctrl_stop();
        Serial.println(F("Sched:OFF"));
      } else {
        ctrl_start(s);
        Serial.print(F("Sched:fire @")); Serial.print(s, 1); Serial.println('s');
      }
    } else {
      ctrl_arm();
      Serial.println(F("Sched:armed (fires on enable)"));
    }
  }
#if ENABLE_EEPROM_SETTINGS
  else if (MATCH("save", 4)) {
    settings_save();
    Serial.println(F("Saved"));
  }
  else if (MATCH("load", 4)) {
    if (settings_load()) {
      tps_setCurrentLimit(g_imax_mA);
      tps_setVoltageLimit(g_vmax_mV);
      Serial.println(F("Loaded"));
    } else {
      Serial.println(F("ERR: no EEPROM data"));
    }
  }
  else if (MATCH("defaults", 8)) {
    settings_reset();
    tps_setCurrentLimit(g_imax_mA);
    tps_setVoltageLimit(g_vmax_mV);
    Serial.println(F("Defaults restored"));
  }
#endif // ENABLE_EEPROM_SETTINGS
  else if (MATCH("deadband", 8)) {
    if (_has_arg(cmd + 8)) {
      g_deadband = atof(cmd + 8);
      if (g_deadband < 0) g_deadband = 0;
      if (g_deadband == 0) {
        Serial.println(F("WARN: zero deadband causes TEC chatter"));
      }
    }
    Serial.print(F("DB=")); Serial.println(g_deadband, 2);
  }
#if ENABLE_SERIAL_PLOTTER
  else if (MATCH("stream", 6)) {
    g_stream = !g_stream;
    Serial.print(F("Stream:")); Serial.println(g_stream ? '1' : '0');
  }
#endif
  else {
    Serial.print(F("Unknown: ")); Serial.println(cmd);
  }
}

#endif // SERIAL_CMD_H
