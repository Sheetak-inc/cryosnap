#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h"

/*
  Settings.h — EEPROM persistence for runtime state.

  Lets the operator persist tuned values (setpoint, fan duty,
  PID gains, NTC cal, etc.) across power cycles instead of always
  booting at Config.h defaults. Public API:

    settings_save()     write current runtime state to EEPROM
    settings_load()     read EEPROM, validate, apply to runtime state
    settings_reset()    wipe EEPROM and restore Config.h defaults

  ---------------------------------------------------------------
  HOW TO ADD A NEW PERSISTED FIELD
  ---------------------------------------------------------------
  The struct is wrapped with a magic word + version + CRC. Any
  layout change has to bump SETTINGS_VERSION so existing EEPROMs
  are rejected at boot (the firmware then falls back to Config.h
  defaults until the operator runs `save` once). To add a field:

    1. Append the new member to `struct SavedSettings` BEFORE the
       `crc` field (the CRC must stay last because _settings_crc
       hashes everything except itself).
    2. Bump SETTINGS_VERSION below by 1.
    3. In settings_save(), copy the runtime global into s.<field>.
    4. In settings_load(), copy s.<field> back into the runtime
       global (or call the appropriate setter).
    5. In settings_reset(), set the runtime global to the
       Config.h default for cold-start.
    6. (Optional) Add a serial command to read/write it — see the
       recipe at the top of SerialCmd.h.

  The Arduino Nano has 1 KB of EEPROM; the struct currently uses
  ~60 B, so there's plenty of room.

  Storage layout is a single struct at EEPROM address 0, wrapped with
  a magic number + version byte + simple CRC. If any of the three
  fails at load time, the runtime state stays at Config.h defaults
  (so a blank or corrupted EEPROM doesn't brick the controller).

  Struct version: bump SETTINGS_VERSION when the layout changes.
  settings_load() will treat an older version as invalid and the
  next settings_save() overwrites with the new format.

  The Arduino Nano has 1 KB of EEPROM. This struct uses ~36 bytes.
*/

#define SETTINGS_MAGIC    0xABCD
#define SETTINGS_VERSION  3   // bump on any layout change to invalidate old EEPROMs
#define SETTINGS_ADDR     0

struct SavedSettings {
  uint16_t magic;
  uint8_t  version;
  // control state
  uint8_t  mode;
  uint8_t  fan_speed;
  uint8_t  damping_pct;
  uint8_t  led_brightness;     // v2 — 0..255 master scale for WS2812 frame
  uint8_t  use_pid;            // v3 — 0 = bang-bang, 1 = PID
  uint16_t imax_mA;
  uint16_t vmax_mV;
  float    setpoint;
  float    maxhot;
  float    deadband;
  float    damping;
  // PID tuning (v3)
  float    kp;
  float    ki;
  float    kd;
  // NTC calibration
  float    ntc_scale;
  float    ntc_offset;
  // checksum (last field)
  uint16_t crc;
};

// Simple 16-bit XOR-fold CRC over all bytes except the crc field
// itself. Not cryptographic — just catches EEPROM corruption.
static inline uint16_t _settings_crc(const SavedSettings& s) {
  const uint8_t* p = (const uint8_t*)&s;
  size_t len = sizeof(s) - sizeof(s.crc);
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum = (sum << 1) | (sum >> 15);  // rotate-left-1
    sum ^= p[i];
  }
  return sum;
}

// Write the current runtime state to EEPROM. Returns true on success.
// Uses EEPROM.put which skips unchanged bytes (wear-sparing).
inline bool settings_save() {
  SavedSettings s = {};
  s.magic       = SETTINGS_MAGIC;
  s.version     = SETTINGS_VERSION;
  s.mode           = g_mode;
  s.fan_speed      = g_fan_speed;
  s.damping_pct    = g_damping_pct;
  s.led_brightness = g_led_brightness;
  s.use_pid        = g_use_pid ? 1 : 0;
  s.imax_mA        = g_imax_mA;
  s.vmax_mV     = g_vmax_mV;
  s.setpoint    = g_setpoint;
  s.maxhot      = g_maxhot;
  s.deadband    = g_deadband;
  s.damping     = g_damping;
  s.kp          = pid_getKp();
  s.ki          = pid_getKi();
  s.kd          = pid_getKd();
  s.ntc_scale   = ntc_getScale();
  s.ntc_offset  = ntc_getOffset();
  s.crc         = _settings_crc(s);

  EEPROM.put(SETTINGS_ADDR, s);
  return true;
}

// Read from EEPROM and apply to runtime state. Returns true if the
// EEPROM contents were valid; false if blank/corrupted/wrong version
// (in which case the runtime state is left at Config.h defaults).
inline bool settings_load() {
  SavedSettings s = {};
  EEPROM.get(SETTINGS_ADDR, s);

  if (s.magic != SETTINGS_MAGIC)      return false;
  if (s.version != SETTINGS_VERSION)  return false;
  if (_settings_crc(s) != s.crc)      return false;

  g_mode           = s.mode;
  g_fan_speed      = s.fan_speed;
  g_damping_pct    = s.damping_pct;
  g_led_brightness = s.led_brightness;
  g_use_pid        = (s.use_pid != 0);
  g_imax_mA        = s.imax_mA;
  g_vmax_mV     = s.vmax_mV;
  g_setpoint    = s.setpoint;
  g_maxhot      = s.maxhot;
  g_deadband    = s.deadband;
  g_damping     = s.damping;
  pid_setKp(s.kp);
  pid_setKi(s.ki);  // also resets the integrator
  pid_setKd(s.kd);
  ntc_setScale(s.ntc_scale);
  ntc_setOffset(s.ntc_offset);
  return true;
}

// Wipe the EEPROM header and restore Config.h defaults. Caller may
// want to also push the new defaults to the hardware (e.g. call
// tps_setCurrentLimit(g_imax_mA) after).
inline void settings_reset() {
  SavedSettings blank = {};  // magic = 0 → load() will reject
  EEPROM.put(SETTINGS_ADDR, blank);

  g_mode           = MODE_COOL;
  g_fan_speed      = DEFAULT_FAN_SPEED;
  g_damping_pct    = DEFAULT_DAMPING_PCT;
  g_led_brightness = DEFAULT_LED_BRIGHTNESS;
  g_use_pid        = DEFAULT_USE_PID;
  g_imax_mA        = (uint16_t)(DEFAULT_MAX_CURRENT * 1000);
  g_vmax_mV     = (uint16_t)(DEFAULT_MAX_VOLTAGE * 1000);
  g_setpoint    = DEFAULT_SETPOINT;
  g_maxhot      = DEFAULT_MAX_TEMP_HOT;
  g_deadband    = DEFAULT_DEADBAND;
  g_damping     = DEFAULT_DAMPING_BAND;
  pid_setKp(DEFAULT_KP);
  pid_setKi(DEFAULT_KI);
  pid_setKd(DEFAULT_KD);
  ntc_setScale(NTC_SCALE);
  ntc_setOffset(NTC_OFFSET);
}

#endif // SETTINGS_H
