#ifndef INA226_H
#define INA226_H

#include <Arduino.h>
#include <Wire.h>
#include "Pins.h"
#include "Config.h"

/*
  INA226.h — bidirectional current / bus-voltage / power monitor.

  Reads what the TEC is actually drawing each 100 ms tick. Three
  numbers come out: current (A), bus voltage (V), and power (W). All
  three feed the Serial Plotter line and the `status` serial command.

  The chip also has an alert latch (MASK_ENABLE register, AFF bit)
  that fires on configurable over-current / over-voltage / power
  conditions. We poll the latch over I2C each tick rather than wire
  the D10 alert pin as an interrupt — see flowchart §7.1 for the
  rationale.

  Wiring: I2C @ 0x40. Shunt = TPS_RSENSE (10 mOhm) from Config.h.

  Public API:
    ina_init()          probe, verify mfg ID, write CONFIG + CAL
    ina_readCurrentA()  signed; positive = current INTO the load
    ina_readVoltageV()  bus voltage at IN+
    ina_readPowerW()    chip-computed |I| * V
    ina_alertLatched()  true if any enabled alert has fired since
                        the last call (read clears the latch)
*/

#if HAS_INA226

// ---- register map (INA226 datasheet) ---------------------------------
#define INA_REG_CONFIG       0x00
#define INA_REG_SHUNT_V      0x01
#define INA_REG_BUS_V        0x02
#define INA_REG_POWER        0x03
#define INA_REG_CURRENT      0x04
#define INA_REG_CALIBRATION  0x05
#define INA_REG_MASK_ENABLE  0x06
#define INA_REG_ALERT_LIMIT  0x07
#define INA_REG_MFG_ID       0xFE
#define INA_REG_DIE_ID       0xFF

#define INA_MFG_ID_TI        0x5449   // ASCII "TI" — INA226 manufacturer ID
#define INA_AFF_BIT          0x0010   // bit 4 of MASK_ENABLE = alert function flag

// Measurement range. The TPS55288 can boost current post-inductor, so
// the INA226 can see up to ~16 A on the TEC side. Pick max measured
// current >= that to avoid clipping the signed 16-bit register.
// This is independent of any safety limit configured in the firmware.
#define INA_MAX_CURRENT_A    16.0f

// Calibration math (INA226 datasheet section 8.5.2):
//
//   1. current_LSB = max_expected / 2^15
//        = 16 / 32768 = ~488 uA
//      Round UP to a "nice" number -> 500 uA (0.0005 A).
//      This gives full-scale = ±32767 × 500 uA = ±16.38 A.
//
//   2. power_LSB = 25 × current_LSB
//        = 25 × 500 uA = 12.5 mW
//
//   3. CAL = trunc(0.00512 / (current_LSB × R_shunt))
//      The shunt resistor is INA_RSENSE from Config.h.
//      With 100 mOhm bench shunt:  CAL = 0.00512 / (0.0005 × 0.100) = 102
//      With  10 mOhm target shunt: CAL = 0.00512 / (0.0005 × 0.010) = 1024
//
//   4. To read current:  I = CURRENT_register × current_LSB  (signed)
//      To read voltage:  V = BUS_V_register   × 1.25 mV      (fixed)
//      To read power:    P = POWER_register   × power_LSB
//
static const float    _INA_CURRENT_LSB = 0.0005f;    // 500 uA per bit
static const float    _INA_POWER_LSB   = 0.0125f;    // 12.5 mW per bit (= 25 × current_LSB)
static const float    _INA_BUS_LSB     = 0.00125f;   // 1.25 mV per bit (chip-fixed)
// Round before the cast. Without the + 0.5f, the float evaluates to
// 1023.9998 at INA_RSENSE = 10 mOhm and truncates to 1023 (0x3FF)
// instead of the correct 1024 (0x400). The bias is small (~0.1%)
// but feeds every current/power reading, the PD budget decision,
// and the supply-fault detection floor — systematic error in all
// downstream protection. (BUG-005 / CAL in the 2026-06-03 audit.)
static const uint16_t _INA_CAL_VALUE   =
    (uint16_t)(0.00512f / (_INA_CURRENT_LSB * INA_RSENSE) + 0.5f);

// Maximum measurable current with the current shunt resistor.
// The INA226 SHUNT_V register clips at ±81.92 mV (±32767 × 2.5 uV).
// Any current above this threshold reads as this value — the chip
// can't tell the difference between 0.82 A and 10 A with a 100 mOhm
// shunt. The 10 mOhm resistors on order raise this to 8.19 A.
//
//   100 mOhm: max = 0.8192 A    <-- current bench
//    10 mOhm: max = 8.192 A
//     5 mOhm: max = 16.384 A
static const float _INA_MAX_SHUNT_MV = 81.92f;
static const float _INA_MAX_CURRENT  = _INA_MAX_SHUNT_MV / (INA_RSENSE * 1000.0f);

// Clipping flag — set by ina_readCurrentA() when the reading is at
// or near the hardware limit (>= 95% of max measurable).
static bool _ina_clipped = false;

// CONFIG default:
//   AVG    = 0    (1 sample, no averaging — keep latency low)
//   VBUSCT = 100  (1.1 ms bus conversion time)
//   VSHCT  = 100  (1.1 ms shunt conversion time)
//   MODE   = 111  (continuous shunt + bus)
// = 0x4127 (datasheet table 1)
#define INA_CONFIG_DEFAULT   0x4127

// ---- private I2C helpers (file-local) --------------------------------
//
// The INA226 is a 16-bit big-endian I2C device. Every register is
// two bytes wide and MSB goes on the wire first. These helpers wrap
// the Wire library's beginTransmission / write / endTransmission /
// requestFrom dance so the public API never touches Wire directly.

static inline void _ina_write16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(I2C_ADDR_INA226);
  Wire.write(reg);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

static inline uint16_t _ina_read16(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR_INA226);
  Wire.write(reg);
  Wire.endTransmission(false);  // restart, hold the bus
  Wire.requestFrom((uint8_t)I2C_ADDR_INA226, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

// ---- public API ------------------------------------------------------

inline bool ina_init() {
  // ACK probe.
  Wire.beginTransmission(I2C_ADDR_INA226);
  if (Wire.endTransmission() != 0) return false;

  // Verify the manufacturer ID. Guards against a different I2C device
  // squatting on 0x40 (it's a busy address — many sensors live there).
  if (_ina_read16(INA_REG_MFG_ID) != INA_MFG_ID_TI) return false;

  _ina_write16(INA_REG_CONFIG,      INA_CONFIG_DEFAULT);
  _ina_write16(INA_REG_CALIBRATION, _INA_CAL_VALUE);
  return true;
}

inline float ina_readCurrentA() {
  // CURRENT is a signed 16-bit value. Read into int16_t to preserve
  // sign before scaling.
  int16_t raw = (int16_t)_ina_read16(INA_REG_CURRENT);
  float i = (float)raw * _INA_CURRENT_LSB;
  // Detect hardware clipping: if the absolute current reading is
  // >= 99% of the maximum measurable, the shunt voltage register
  // is saturated and the real current is higher than reported.
  // 99% avoids false positives when the TEC genuinely draws near
  // (but below) the shunt limit — e.g. 0.798 A on a 0.819 A max
  // shunt is a real reading, not clipping.
  _ina_clipped = (fabs(i) >= _INA_MAX_CURRENT * 0.99f);
  return i;
}

inline float ina_readVoltageV() {
  // Bus voltage is NOT affected by shunt overflow — always accurate.
  uint16_t raw = _ina_read16(INA_REG_BUS_V);
  return (float)raw * _INA_BUS_LSB;
}

inline float ina_readPowerW() {
  // When the current reading is clipped, the chip's internal power
  // calculation is also clipped. Use ina_estimatePowerW() for a
  // better estimate in that case.
  uint16_t raw = _ina_read16(INA_REG_POWER);
  return (float)raw * _INA_POWER_LSB;
}

// True if the most recent ina_readCurrentA() was at the hardware
// limit (shunt voltage register saturated). When true, current and
// power readings are capped — the real values are higher.
inline bool ina_isClipped() {
  return _ina_clipped;
}

// Maximum measurable current with the current shunt, in amps.
// Useful for printing "current > X.XX A" when clipped.
inline float ina_maxMeasurable() {
  return _INA_MAX_CURRENT;
}

// Estimate power from bus voltage × the TPS55288 current limit.
// Only meaningful when the INA226 current is clipped and we need a
// better power estimate. The TPS current limit is the ceiling —
// actual power may be lower if the load is below the limit.
//   P_est = V_bus × I_limit_TPS
// Pass the current TPS I_limit in amps.
inline float ina_estimatePowerW(float v_bus, float i_limit_A) {
  return v_bus * i_limit_A;
}

inline bool ina_alertLatched() {
  // Reading MASK_ENABLE clears the AFF bit (datasheet). So each call
  // is "have any enabled alerts fired since the last read?".
  uint16_t me = _ina_read16(INA_REG_MASK_ENABLE);
  return (me & INA_AFF_BIT) != 0;
}

// Print every INA226 register to the given stream, annotated with
// expected values for the constant ones and decoded readings for the
// measurement registers. Useful at boot for verifying the chip is
// wired and configured correctly.
//
// Note: reading MASK_ENABLE clears the AFF latch, so calling this
// during normal operation is effectively a "and reset alert" too.
inline void ina_dump(Stream& out) {
  out.println(F("INA226 (0x40):"));

  uint16_t cfg = _ina_read16(INA_REG_CONFIG);
  int16_t  sht = (int16_t)_ina_read16(INA_REG_SHUNT_V);
  uint16_t bus = _ina_read16(INA_REG_BUS_V);
  uint16_t pwr = _ina_read16(INA_REG_POWER);
  int16_t  cur = (int16_t)_ina_read16(INA_REG_CURRENT);
  uint16_t cal = _ina_read16(INA_REG_CALIBRATION);
  uint16_t me  = _ina_read16(INA_REG_MASK_ENABLE);
  uint16_t mfg = _ina_read16(INA_REG_MFG_ID);
  uint16_t die = _ina_read16(INA_REG_DIE_ID);

  // Compact dump. OK markers show bad values stand out.
  out.print(F(" CFG=0x"));  out.print(cfg, HEX);
  out.println((cfg == INA_CONFIG_DEFAULT) ? F(" OK") : F(" BAD"));
  out.print(F(" SHV=0x"));  out.print((uint16_t)sht, HEX);
  out.print(' '); out.print((float)sht * 0.0025f, 3); out.println(F("mV"));
  out.print(F(" BUS=0x"));  out.print(bus, HEX);
  out.print(' '); out.print((float)bus * _INA_BUS_LSB, 3); out.println('V');
  out.print(F(" PWR=0x"));  out.print(pwr, HEX);
  out.print(' '); out.print((float)pwr * _INA_POWER_LSB, 3); out.println('W');
  out.print(F(" CUR=0x"));  out.print((uint16_t)cur, HEX);
  out.print(' '); out.print((float)cur * _INA_CURRENT_LSB, 3); out.println('A');
  out.print(F(" CAL=0x"));  out.print(cal, HEX);
  out.println((cal == _INA_CAL_VALUE) ? F(" OK") : F(" BAD"));
  out.print(F(" ME=0x"));   out.print(me, HEX);
  out.println((me & INA_AFF_BIT) ? F(" ALERT") : F(" ok"));
  out.print(F(" MFG=0x"));  out.print(mfg, HEX);
  out.println((mfg == INA_MFG_ID_TI) ? F(" TI") : F(" BAD"));
  out.print(F(" DIE=0x"));  out.println(die, HEX);
}

#else  // !HAS_INA226 — chip not on this build target. Keep stubs so the
       // main sketch can call these unconditionally.
inline bool  ina_init()                            { return true; }
inline float ina_readCurrentA()                    { return 0.0f; }
inline float ina_readVoltageV()                    { return 0.0f; }
inline float ina_readPowerW()                      { return 0.0f; }
inline bool  ina_alertLatched()                    { return false; }
inline bool  ina_isClipped()                       { return false; }
inline float ina_maxMeasurable()                   { return 0.0f; }
inline float ina_estimatePowerW(float, float)      { return 0.0f; }
inline void  ina_dump(Stream&)                     {}
#endif

#endif // INA226_H
