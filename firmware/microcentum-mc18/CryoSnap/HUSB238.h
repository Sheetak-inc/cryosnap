#ifndef HUSB238_H
#define HUSB238_H

#include <Arduino.h>
#include <Wire.h>
#include "Pins.h"

/*
  HUSB238.h — USB-PD sink controller driver.

  The TEC needs ~20 V from a USB-C PD supply. The HUSB238 negotiates
  with the upstream supply and reports back what was actually
  delivered. Firmware MUST verify husb_has20V() at boot before
  enabling the TPS55288 output — otherwise the TEC sees the default
  5 V from a non-PD host and nothing useful happens.

  No alert / interrupt pin is wired (schematic confirmed only SDA
  and SCL go to the chip), so all status is polled over I2C.

  Wiring: I2C @ 0x08. Datasheet: Adafruit-published HUSB238 reference.

  The chip only powers its I2C interface when VBUS is present, so
  husb_init() returns false if no USB-C cable is attached. The
  caller logs a warning and continues.

  The user can also wire >20 V directly into the screw terminal on
  the PCB, bypassing USB-C PD entirely. In that case HUSB238 will
  report "no contract" and the firmware should still allow operation
  — the >20 V path is valid for bench and production use.

  Public API:
    husb_init()           probe, request 20 V PDO, wait for negotiation
    husb_has20V()         true if the source has agreed to 20 V
    husb_maxCurrentMA()   negotiated max current (mA), 0 if unattached
*/

#if HAS_HUSB238

// ---- register map (HUSB238 datasheet) --------------------------------
#define HUSB_REG_PD_STATUS0   0x00   // [7:4] negotiated voltage, [3:0] current code
#define HUSB_REG_PD_STATUS1   0x01   // CC orientation, attach, response status
#define HUSB_REG_PDO_5V       0x02
#define HUSB_REG_PDO_9V       0x03
#define HUSB_REG_PDO_12V      0x04
#define HUSB_REG_PDO_15V      0x05
#define HUSB_REG_PDO_18V      0x06
#define HUSB_REG_PDO_20V      0x07
#define HUSB_REG_SRC_PDO      0x08   // [7:4] = which PDO to request
#define HUSB_REG_GO_COMMAND   0x09   // [4:0] = command to execute

// PD_STATUS0[7:4] encoding — negotiated source voltage.
#define HUSB_VOLT_UNATTACHED  0x0
#define HUSB_VOLT_5V          0x1
#define HUSB_VOLT_9V          0x2
#define HUSB_VOLT_12V         0x3
#define HUSB_VOLT_15V         0x4
#define HUSB_VOLT_18V         0x5
#define HUSB_VOLT_20V         0x6

// SRC_PDO[7:4] = which PDO to request from the source.
//
// IMPORTANT: the selector nibble does NOT match the PD_STATUS0
// voltage code for 15/18/20 V — this is a quirk of the chip
// confirmed against the Adafruit_HUSB238 library. 5/9/12 V happen
// to use the same code on both sides; 15/18/20 V do not. Trust
// these constants over any "obvious" mapping.
#define HUSB_PDO_SEL_5V       0x10
#define HUSB_PDO_SEL_9V       0x20
#define HUSB_PDO_SEL_12V      0x30
#define HUSB_PDO_SEL_15V      0x80
#define HUSB_PDO_SEL_18V      0x90
#define HUSB_PDO_SEL_20V      0xA0

// GO_COMMAND values.
#define HUSB_CMD_REQUEST_PDO  0x01
#define HUSB_CMD_GET_SRC_CAP  0x04
#define HUSB_CMD_HARD_RESET   0x10

// ---- private I2C helpers (file-local) --------------------------------

static inline void _husb_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_ADDR_HUSB238);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline uint8_t _husb_read(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR_HUSB238);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)I2C_ADDR_HUSB238, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Decode the 4-bit current code in PD_STATUS0[3:0] to milliamps.
// Datasheet table — discrete USB-PD current levels.
static inline uint16_t _husb_decode_current_mA(uint8_t code) {
  switch (code & 0x0F) {
    case 0x0: return  500;
    case 0x1: return  700;
    case 0x2: return 1000;
    case 0x3: return 1250;
    case 0x4: return 1500;
    case 0x5: return 1750;
    case 0x6: return 2000;
    case 0x7: return 2250;
    case 0x8: return 2500;
    case 0x9: return 2750;
    case 0xA: return 3000;
    case 0xB: return 3250;
    case 0xC: return 3500;
    case 0xD: return 4000;
    case 0xE: return 4500;
    case 0xF: return 5000;
    default:  return 0;
  }
}

// Decode a 4-bit voltage code from PD_STATUS0[7:4] to a short label.
// Returns "?" for reserved codes.
static inline const __FlashStringHelper* _husb_volt_name(uint8_t code) {
  switch (code & 0x0F) {
    case HUSB_VOLT_UNATTACHED: return F("unattached");
    case HUSB_VOLT_5V:         return F("5V");
    case HUSB_VOLT_9V:         return F("9V");
    case HUSB_VOLT_12V:        return F("12V");
    case HUSB_VOLT_15V:        return F("15V");
    case HUSB_VOLT_18V:        return F("18V");
    case HUSB_VOLT_20V:        return F("20V");
    default:                   return F("?");
  }
}

// Decode the SRC_PDO selector nibble (separate code space from
// PD_STATUS0 — see the SRC_PDO constants above for why).
static inline const __FlashStringHelper* _husb_sel_name(uint8_t sel) {
  switch (sel & 0xF0) {
    case 0x00: return F("none");
    case HUSB_PDO_SEL_5V:  return F("5V");
    case HUSB_PDO_SEL_9V:  return F("9V");
    case HUSB_PDO_SEL_12V: return F("12V");
    case HUSB_PDO_SEL_15V: return F("15V");
    case HUSB_PDO_SEL_18V: return F("18V");
    case HUSB_PDO_SEL_20V: return F("20V");
    default:               return F("?");
  }
}

// ---- public API ------------------------------------------------------

// ---- husb_init() — two implementations switchable via Config.h -------
//
// USE_ADAFRUIT_HUSB = 1  ->  Adafruit_HUSB238 library (debug mode)
// USE_ADAFRUIT_HUSB = 0  ->  native register-level driver (default)
//
// Everything else (husb_negotiatedV, husb_has20V, husb_maxCurrentMA,
// husb_dump) stays native in both modes — they just read registers.

#if USE_ADAFRUIT_HUSB
// ------------------------------------------------------------------
// Adafruit library path — verbose debug output at every step.
// Requires Adafruit_HUSB238 library installed (not stock Arduino IDE).
// ------------------------------------------------------------------
#include <Adafruit_HUSB238.h>

static Adafruit_HUSB238 _husb_af;

static inline const __FlashStringHelper* _husb_resp_name(HUSB238_ResponseCodes r) {
  switch (r) {
    case NO_RESPONSE:                   return F("NO_RESPONSE");
    case SUCCESS:                       return F("SUCCESS");
    case INVALID_CMD_OR_ARG:            return F("INVALID_CMD_OR_ARG");
    case CMD_NOT_SUPPORTED:             return F("CMD_NOT_SUPPORTED");
    case TRANSACTION_FAIL_NO_GOOD_CRC:  return F("TRANSACTION_FAIL");
    default:                            return F("UNKNOWN");
  }
}

inline bool husb_init() {
  Serial.println(F("HUSB238 init (Adafruit lib debug):"));

  // Probe using Adafruit's begin() — internally uses Adafruit_I2CDevice.
  if (!_husb_af.begin(I2C_ADDR_HUSB238)) {
    Serial.println(F("  begin() FAILED — chip not on bus"));
    return false;
  }
  Serial.println(F("  begin() OK"));

  // Attachment and CC info.
  Serial.print(F("  attached: "));
  Serial.println(_husb_af.isAttached() ? F("yes") : F("no"));
  Serial.print(F("  CC dir:   "));
  Serial.println(_husb_af.getCCdirection() ? F("CC2") : F("CC1"));

  // Ask the chip to refresh its source-cap PDO registers.
  Serial.println(F("  getSourceCapabilities()..."));
  _husb_af.getSourceCapabilities();
  delay(500);

  // Report which PDOs the source advertises.
  static const HUSB238_PDSelection pdoSel[] = {
    PD_SRC_5V, PD_SRC_9V, PD_SRC_12V, PD_SRC_15V, PD_SRC_18V, PD_SRC_20V
  };
  static const char* const pdoName[] = {
    "5V", "9V", "12V", "15V", "18V", "20V"
  };
  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(F("  PDO "));
    Serial.print(pdoName[i]);
    Serial.print(F(": "));
    Serial.println(_husb_af.isVoltageDetected(pdoSel[i]) ? F("DETECTED") : F("not detected"));
  }

  // Walk 20 V -> 5 V. For each voltage, request the PDO and then
  // poll getPDSrcVoltage() for up to 10 seconds, printing progress
  // every second. This gives slow PD sources enough time to respond
  // and lets us watch the negotiation happen in real time.
  Serial.println(F("  negotiation walk (10 s per attempt):"));
  for (int8_t i = 5; i >= 0; --i) {
    Serial.print(F("    selectPD("));
    Serial.print(pdoName[i]);
    Serial.println(F(")..."));

    _husb_af.selectPD(pdoSel[i]);
    _husb_af.requestPD();

    bool accepted = false;
    for (uint8_t sec = 1; sec <= 10; ++sec) {
      delay(1000);
      HUSB238_ResponseCodes resp = _husb_af.getPDResponse();
      HUSB238_VoltageSetting  vv = _husb_af.getPDSrcVoltage();
      HUSB238_CurrentSetting  cc = _husb_af.getPDSrcCurrent();
      bool att = _husb_af.isAttached();

      Serial.print(F("      "));
      Serial.print(sec);
      Serial.print(F("s: resp="));
      Serial.print(_husb_resp_name(resp));
      Serial.print(F(" V="));
      Serial.print(vv);
      Serial.print(F(" I="));
      Serial.print(cc);
      Serial.print(F(" att="));
      Serial.println(att ? F("yes") : F("no"));

      if (resp == SUCCESS) {
        Serial.print(F("    -> accepted at "));
        Serial.print(pdoName[i]);
        Serial.println(F("!"));
        accepted = true;
        break;
      }
    }
    if (accepted) break;
    Serial.print(F("    -> "));
    Serial.print(pdoName[i]);
    Serial.println(F(" not accepted after 10 s, trying next..."));
  }

  // Show raw PD_STATUS0 via our native read for cross-check.
  uint8_t s0 = _husb_read(HUSB_REG_PD_STATUS0);
  Serial.print(F("  raw PD_STATUS0 = 0x"));
  Serial.print(s0, HEX);
  Serial.print(F("  (V="));
  Serial.print(_husb_volt_name((s0 >> 4) & 0x0F));
  Serial.println(F(")"));

  return true;
}

#else
// ------------------------------------------------------------------
// Native driver path (default) — direct register access.
// ------------------------------------------------------------------
inline bool husb_init() {
  // ACK probe at 0x08.
  Wire.beginTransmission(I2C_ADDR_HUSB238);
  if (Wire.endTransmission() != 0) return false;

  // Check if we already have a valid PD contract (e.g. negotiated by
  // the DIP switches at power-up). If 20 V is already there at full
  // current, skip the negotiation walk entirely.
  //
  // Sometimes the chip reports 20 V but with a low current (e.g.
  // 1250 mA) because the source hasn't finalized the contract yet.
  // In that case, re-request 20 V to trigger a fresh negotiation
  // that picks up the source's full current budget.
  uint8_t s0 = _husb_read(HUSB_REG_PD_STATUS0);
  uint8_t cur_v = (s0 >> 4) & 0x0F;
  if (cur_v == HUSB_VOLT_20V) {
    uint16_t cur_mA = _husb_decode_current_mA(s0 & 0x0F);
    if (cur_mA >= 5000) {
      Serial.println(F("HUSB238: 20V / 5A already negotiated"));
      return true;
    }
    // 20 V but under-current — renegotiate to get the full budget.
    Serial.print(F("HUSB238: 20V but only "));
    Serial.print(cur_mA);
    Serial.println(F(" mA, renegotiating..."));
    _husb_write(HUSB_REG_SRC_PDO,    HUSB_PDO_SEL_20V);
    _husb_write(HUSB_REG_GO_COMMAND, HUSB_CMD_REQUEST_PDO);
    delay(2000);
    s0 = _husb_read(HUSB_REG_PD_STATUS0);
    cur_mA = _husb_decode_current_mA(s0 & 0x0F);
    Serial.print(F("HUSB238: after renegotiation: "));
    Serial.print(_husb_volt_name((s0 >> 4) & 0x0F));
    Serial.print(F(" / "));
    Serial.print(cur_mA);
    Serial.println(F(" mA"));
    return true;
  }
  Serial.print(F("HUSB238: current state = "));
  Serial.println(_husb_volt_name(cur_v));

  // Walk requested voltages from highest to lowest. Highest accepted
  // wins. Each step: write SRC_PDO selector, kick GO_COMMAND, give
  // the source time to respond, then read PD_STATUS0. Adafruit's
  // TryAllVoltages example uses 2 s per attempt; we use the same.
  struct PdoStep { uint8_t sel; uint8_t expected; };
  static const PdoStep steps[] = {
    { HUSB_PDO_SEL_20V, HUSB_VOLT_20V },
    { HUSB_PDO_SEL_18V, HUSB_VOLT_18V },
    { HUSB_PDO_SEL_15V, HUSB_VOLT_15V },
    { HUSB_PDO_SEL_12V, HUSB_VOLT_12V },
    { HUSB_PDO_SEL_9V,  HUSB_VOLT_9V  },
    { HUSB_PDO_SEL_5V,  HUSB_VOLT_5V  },
  };
  for (uint8_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
    _husb_write(HUSB_REG_SRC_PDO,    steps[i].sel);
    _husb_write(HUSB_REG_GO_COMMAND, HUSB_CMD_REQUEST_PDO);
    delay(2000);
    s0 = _husb_read(HUSB_REG_PD_STATUS0);
    uint8_t got = (s0 >> 4) & 0x0F;

    Serial.print(F("HUSB238: tried "));
    Serial.print(_husb_volt_name(steps[i].expected));
    Serial.print(F(", got "));
    Serial.println(_husb_volt_name(got));

    if (got == steps[i].expected) break;
  }

  return true;
}
#endif // USE_ADAFRUIT_HUSB

// Negotiated source voltage, in volts. 0 = no contract (unattached
// or source rejected every PDO we asked for).
inline uint8_t husb_negotiatedV() {
  uint8_t s0 = _husb_read(HUSB_REG_PD_STATUS0);
  switch ((s0 >> 4) & 0x0F) {
    case HUSB_VOLT_5V:  return 5;
    case HUSB_VOLT_9V:  return 9;
    case HUSB_VOLT_12V: return 12;
    case HUSB_VOLT_15V: return 15;
    case HUSB_VOLT_18V: return 18;
    case HUSB_VOLT_20V: return 20;
    default:            return 0;
  }
}

// True if the upstream supply has agreed to deliver 20 V specifically.
inline bool husb_has20V() {
  return husb_negotiatedV() == 20;
}

// Negotiated max current available from the supply, in milliamps.
// Returns 0 if no PD contract has been established (unattached).
inline uint16_t husb_maxCurrentMA() {
  uint8_t s0 = _husb_read(HUSB_REG_PD_STATUS0);
  if (((s0 >> 4) & 0x0F) == HUSB_VOLT_UNATTACHED) return 0;
  return _husb_decode_current_mA(s0 & 0x0F);
}

// Print every HUSB238 register relevant to PD status. Use this at
// boot when 20 V isn't negotiated to figure out why — e.g. no
// advertised voltages means the source isn't a PD-capable supply.
inline void husb_dump(Stream& out) {
  out.print(F("HUSB238 (0x"));
  out.print(I2C_ADDR_HUSB238, HEX);
  out.println(F("):"));

  uint8_t s0  = _husb_read(HUSB_REG_PD_STATUS0);
  uint8_t s1  = _husb_read(HUSB_REG_PD_STATUS1);
  uint8_t src = _husb_read(HUSB_REG_SRC_PDO);

  out.print(F(" ST0=0x")); out.print(s0, HEX);
  out.print(F(" V=")); out.print(_husb_volt_name((s0 >> 4) & 0x0F));
  out.print(F(" I="));
  out.print(((s0 >> 4) & 0x0F) == HUSB_VOLT_UNATTACHED
            ? 0 : _husb_decode_current_mA(s0 & 0x0F));
  out.println(F("mA"));

  out.print(F(" ST1=0x")); out.println(s1, HEX);

  // Compact PDO row: "Adv:5,9,15,20" showing which voltages are
  // advertised. Uses ~60 bytes instead of ~200.
  static const uint8_t regs[] = { HUSB_REG_PDO_5V,  HUSB_REG_PDO_9V,
                                  HUSB_REG_PDO_12V, HUSB_REG_PDO_15V,
                                  HUSB_REG_PDO_18V, HUSB_REG_PDO_20V };
  static const uint8_t volts[] = { 5, 9, 12, 15, 18, 20 };
  out.print(F(" Adv:"));
  bool first = true;
  for (uint8_t i = 0; i < 6; ++i) {
    if (_husb_read(regs[i]) & 0x80) {
      if (!first) out.print(',');
      out.print(volts[i]);
      first = false;
    }
  }
  if (first) out.print(F("none"));
  out.println();

  out.print(F(" SRC=0x")); out.print(src, HEX);
  out.print(' '); out.println(_husb_sel_name(src));
}

#else  // !HAS_HUSB238 — chip not on this build target. Stubs treat
       // power as "OK" so a bench rig without PD isn't power-gated off.
inline bool     husb_init()         { return true; }
inline bool     husb_has20V()       { return true; }
inline uint8_t  husb_negotiatedV()  { return 20; }
inline uint16_t husb_maxCurrentMA() { return 3250; }
inline void     husb_dump(Stream&)  {}
#endif

#endif // HUSB238_H
