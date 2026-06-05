#ifndef TPS55288_H
#define TPS55288_H

#include <Arduino.h>
#include <Wire.h>
#include "Pins.h"

/*
  TPS55288.h — buck-boost converter driver.

  The TPS55288 is the chip that actually drives the TEC. Firmware sets
  a voltage limit and a current limit over I2C; the chip handles the
  switching. Output can be enabled or disabled in software.

  Wiring: SDA = A4, SCL = A5. Address selected at compile time in
  Pins.h (0x75 on the prototype, 0x74 on Rev A).

  Sense resistor: TPS_RSENSE in Config.h (10 mOhm).

  Public API:
    tps_init()              probe + safe defaults, output DISABLED
    tps_setOutput(bool)     gate the regulated output on/off
    tps_setVoltageLimit(mV) set V_limit, 800..20000 mV
    tps_setCurrentLimit(mA) set I_limit code (limiter bit kept OFF)

  The chip's internal current limiter is ENABLED (REG_IOUT_LIMIT
  bit 0x80 set) — the TPS hardware-clamps at the configured I limit
  rather than relying on the firmware control loop alone. To revert
  to soft-limit-only behaviour, clear that bit in tps_setCurrentLimit
  below.
*/

// ---- register map (TPS55288 datasheet) -------------------------------
#define TPS_REG_VREF_L      0x00
#define TPS_REG_VREF_H      0x01
#define TPS_REG_IOUT_LIMIT  0x02
#define TPS_REG_VOUT_SR     0x03
#define TPS_REG_VOUT_FS     0x04
#define TPS_REG_CDC         0x05
#define TPS_REG_MODE        0x06
#define TPS_REG_STATUS      0x07

#define TPS_MODE_OE_BIT     0x80   // bit 7 of REG_MODE = output enable

// ---- private I2C helpers (file-local) --------------------------------

static inline void _tps_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_ADDR_TPS55288);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline uint8_t _tps_read(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR_TPS55288);
  Wire.write(reg);
  Wire.endTransmission(false);  // restart, hold the bus
  Wire.requestFrom((uint8_t)I2C_ADDR_TPS55288, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// ---- public API ------------------------------------------------------

// Set the output voltage limit, in millivolts. Clamped to the chip's
// 0.8 V..20 V range. The chip has four selectable feedback ranges
// (5 / 10 / 15 / 20 V full scale); we pick the smallest range that
// covers the requested voltage so the DAC step stays fine.
inline void tps_setVoltageLimit(uint16_t mV) {
  if (mV < 800)   mV = 800;
  if (mV > 20000) mV = 20000;

  // Datasheet table 7-7 / 7-8: full-scale mV, VOUT_FS code, and the
  // resulting ratio of REF voltage to OUT voltage.
  struct VfbRange { uint16_t fs_mV; uint8_t code; float ratio; };
  static const VfbRange ranges[] = {
    {  5000, 0x00, 0.2256f },
    { 10000, 0x01, 0.1128f },
    { 15000, 0x02, 0.0752f },
    { 20000, 0x03, 0.0564f },
  };
  const VfbRange* sel = &ranges[3];
  for (uint8_t i = 0; i < 4; ++i) {
    if (mV <= ranges[i].fs_mV) { sel = &ranges[i]; break; }
  }

  // Convert target output voltage -> required REF voltage -> 10-bit DAC.
  // VREF range is 0.045 V .. ~1.2 V with ~1.129 mV per LSB.
  float ref_v = ((float)mV / 1000.0f) * sel->ratio;
  if (ref_v < 0.045f) ref_v = 0.045f;
  uint16_t code = (uint16_t)((ref_v - 0.045f) / 0.001129f + 0.5f);
  if (code > 0x03FF) code = 0x03FF;

  // Force internal feedback while preserving reserved bits in VOUT_FS.
  uint8_t fs = _tps_read(TPS_REG_VOUT_FS);
  fs &= 0x7C;          // keep reserved bits, clear FB + INTFB
  fs |= sel->code;     // FB stays 0 -> internal feedback divider
  _tps_write(TPS_REG_VOUT_FS, fs);

  // VREF_L must be written before VREF_H — writing high latches the DAC.
  _tps_write(TPS_REG_VREF_L,  code & 0xFF);
  _tps_write(TPS_REG_VREF_H, (code >> 8) & 0x03);
}

// Set the output current limit, in milliamps.
//
// REG_IOUT_LIMIT format:
//   bit 7   = limiter enable (1 = active)
//   bits 6:0 = sense voltage / 0.5 mV (0..127 -> 0..63.5 mV)
//
// The math:
//   1. Convert requested current (mA) to sense voltage:
//        V_sense = I × R_sense = (mA / 1000) × TPS_RSENSE
//   2. Convert V_sense to register steps:
//        steps = V_sense / 0.5 mV = V_sense / 0.0005
//   3. Clamp steps to 0..127 (63.5 mV max sense voltage).
//   4. Write steps with bit 7 set to enable the limiter.
//
// At TPS_RSENSE = 10 mOhm, max encodable current =
//   0.0635 V / 0.010 Ohm = 6.35 A.
inline void tps_setCurrentLimit(uint16_t mA) {
  float v_sense = ((float)mA / 1000.0f) * TPS_RSENSE;  // V
  if (v_sense < 0.0f)    v_sense = 0.0f;
  if (v_sense > 0.0635f) v_sense = 0.0635f;            // 127 LSB cap

  // Round before the cast (+ 0.5f). Without rounding, 40 mA
  // encoded to 0 steps (full dead-zone, controller can't drive
  // sub-50 mA currents at all) and 2000 mA encoded to 39 steps
  // = 1950 mA delivered. Rounding gives the closest valid step.
  // (BUG-005 / PID-5 in the 2026-06-03 audit log.)
  uint8_t steps = (uint8_t)(v_sense / 0.0005f + 0.5f);

  // Bit 7 enables the hardware current limiter.
  _tps_write(TPS_REG_IOUT_LIMIT, 0x80 | (steps & 0x7F));
}

// Cheap ACK probe — true if the chip is alive on the I2C bus.
// The TPS55288 stops responding when its Vin browns out (the chip's
// internal LDO can't sustain the I2C interface below UVLO), so a
// NACK from this probe distinguishes "TPS lost its supply" from
// "supply is insufficient to hold the commanded V_limit" — two
// failure modes that look identical when reading registers because
// Wire.read() returns 0 on a non-responding device. The supply
// check in task_100ms uses this to skip the V_limit comparison
// (which would otherwise false-trigger FAULT_NO_SUPPLY) when the
// chip itself has dropped off the bus.
inline bool tps_isPresent() {
  Wire.beginTransmission(I2C_ADDR_TPS55288);
  return (Wire.endTransmission() == 0);
}

// Read the *currently active* output voltage limit back from the chip,
// in millivolts. Reconstructed from VREF_L / VREF_H (10-bit reference
// code) and VOUT_FS (range select) using the same math as
// tps_setVoltageLimit() in reverse.
//
// Used by the supply-detection probe: when Vin is insufficient to
// hold the commanded Vlim, the TPS55288 silently snaps Vlim back to
// its 5 V default after OE is asserted. Reading Vlim back after a
// brief OE pulse is the firmware's only reliable signal that the
// upstream supply (USB-PD or direct Vin) is actually adequate for
// 12 V TEC drive.
//
// IMPORTANT: this function blindly reads registers; if the chip is
// not on the bus, every read returns 0 and the decoded value is
// ~200 mV (well below SUPPLY_VLIM_FLOOR). Callers that care about
// the difference between "chip is alive but Vlim is 5 V" and
// "chip is gone" must gate this call on tps_isPresent() above.
inline uint16_t tps_getVoltageLimitMV() {
  uint8_t  vref_l  = _tps_read(TPS_REG_VREF_L);
  uint8_t  vref_h  = _tps_read(TPS_REG_VREF_H);
  uint8_t  vout_fs = _tps_read(TPS_REG_VOUT_FS);
  static const float ratios[] = { 0.2256f, 0.1128f, 0.0752f, 0.0564f };
  uint16_t ref_code = (((uint16_t)(vref_h & 0x03)) << 8) | vref_l;
  float    ref_v    = 0.045f + (float)ref_code * 0.001129f;
  float    v_lim    = ref_v / ratios[vout_fs & 0x03];
  return (uint16_t)(v_lim * 1000.0f);
}

// Enable or disable the regulated TEC output rail. Read-modify-write
// on REG_MODE so we don't disturb the other mode bits.
inline void tps_setOutput(bool on) {
  uint8_t mode = _tps_read(TPS_REG_MODE);
  if (on)  mode |=  TPS_MODE_OE_BIT;
  else     mode &= ~TPS_MODE_OE_BIT;
  _tps_write(TPS_REG_MODE, mode);
}

// Print every TPS55288 register to the given stream, annotated with
// expected values or a decoded reading. Useful at boot for verifying
// the chip is wired and configured correctly. Each line:
//   reg name = 0x?? (decoded / expected)
inline void tps_dump(Stream& out) {
  out.println(F("TPS55288 (0x75):"));

  uint8_t vref_l = _tps_read(TPS_REG_VREF_L);
  uint8_t vref_h = _tps_read(TPS_REG_VREF_H);
  uint8_t iout   = _tps_read(TPS_REG_IOUT_LIMIT);
  uint8_t vout_sr= _tps_read(TPS_REG_VOUT_SR);
  uint8_t vout_fs= _tps_read(TPS_REG_VOUT_FS);
  uint8_t cdc    = _tps_read(TPS_REG_CDC);
  uint8_t mode   = _tps_read(TPS_REG_MODE);
  uint8_t status = _tps_read(TPS_REG_STATUS);

  // Decode the V_limit from VREF code + VOUT_FS range select.
  static const float ratios[] = { 0.2256f, 0.1128f, 0.0752f, 0.0564f };
  uint16_t ref_code = (((uint16_t)(vref_h & 0x03)) << 8) | vref_l;
  float    ref_v    = 0.045f + (float)ref_code * 0.001129f;
  float    v_lim    = ref_v / ratios[vout_fs & 0x03];

  // Decode the I_limit from IOUT_LIMIT register steps + TPS_RSENSE.
  // steps × 0.5 mV = sense voltage → I = V_sense / R_sense.
  uint8_t steps = iout & 0x7F;
  float i_lim = (float)steps * 0.0005f / TPS_RSENSE;

  // Compact dump — one register per line, hex value + decoded meaning.
  out.print(F(" VREF=0x"));  out.print(vref_l, HEX);
  out.print(':');            out.println(vref_h, HEX);
  out.print(F(" IOUT=0x"));  out.print(iout, HEX);
  out.print((iout & 0x80) ? F(" lim=ON stp=") : F(" lim=OFF stp="));
  out.println(steps);
  out.print(F(" VFS=0x"));   out.print(vout_fs, HEX);
  out.print(F(" range="));   out.println(vout_fs & 0x03);
  out.print(F(" VSR=0x"));   out.println(vout_sr, HEX);
  out.print(F(" CDC=0x"));   out.print(cdc, HEX);
  out.print(F(" mask:"));
  out.print((cdc & 0x80) ? 'S' : '-');
  out.print((cdc & 0x40) ? 'O' : '-');   // O = OCP indication enabled
  out.println((cdc & 0x20) ? 'V' : '-');
  out.print(F(" MODE=0x"));  out.print(mode, HEX);
  out.println((mode & TPS_MODE_OE_BIT) ? F(" OE=on") : F(" OE=off"));
  out.print(F(" STAT=0x"));  out.println(status, HEX);
  out.print(F(" Vlim=")); out.print(v_lim, 2); out.print(F("V Ilim="));
  out.print(i_lim, 2); out.println(F("A"));
}

// CDC register at reset is 0xE0 = SC_MASK | OCP_MASK | OVP_MASK
// (bits 7,6,5 set). With OCP_MASK=1, the chip's FB/INT pin (wired
// to A0 = HW_TPS_FAULT on Rev A) goes LOW any time the output is
// in current limit — which for a TEC load is the *normal* operating
// mode, not a fault.
//
// Per the datasheet (§7.6.5):
//   "The OCP_MASK must be 0 when the OE bit or the Current_Limit_EN
//    bit is changed from 0 to 1. After the OE bit and the Current_-
//    Limit_EN bit are set, set the OCP_MASK to 1 to enable the OCP
//    fault indication output."
//
// Since we use current limit as a normal operating condition for
// TEC drive (not just transiently at startup), we leave OCP_MASK
// permanently CLEARED. SC_MASK (short-circuit) and OVP_MASK
// (over-voltage) stay enabled — those are real catastrophes worth
// surfacing on the FB/INT pin.
//
//   bit 7  SC_MASK  = 1   short-circuit indication enabled
//   bit 6  OCP_MASK = 0   ignore current-limit (it's intentional)
//   bit 5  OVP_MASK = 1   over-voltage indication enabled
//   bits 4..0          = 0  (no cable droop compensation)
#define TPS_CDC_OPMODE   0xA0

// Probe the chip and write conservative defaults. Output stays
// DISABLED — call tps_setOutput(true) explicitly only after the rest
// of the system has been gated by HUSB238 + fault checks.
inline bool tps_init() {
  // ACK probe first. If the chip isn't on the bus, bail early so the
  // caller can log a warning instead of silently doing nothing.
  Wire.beginTransmission(I2C_ADDR_TPS55288);
  if (Wire.endTransmission() != 0) return false;

  // Output OFF first (read-modify-write on REG_MODE clears OE).
  tps_setOutput(false);

  // Mask the OCP fault indication on the FB/INT pin — see the
  // long comment above TPS_CDC_OPMODE for the why.
  _tps_write(TPS_REG_CDC, TPS_CDC_OPMODE);

  // Start at the configured default max current (DEFAULT_MAX_CURRENT
  // from Config.h). The limiter is ON so the chip hardware-clamps at
  // this level until the control loop writes a different value.
  tps_setCurrentLimit((uint16_t)(DEFAULT_MAX_CURRENT * 1000));

  // 5 V starting voltage limit. Safe even if the output is somehow
  // enabled before the control loop sets V/I properly.
  tps_setVoltageLimit(5000);

  return true;
}

#endif // TPS55288_H
