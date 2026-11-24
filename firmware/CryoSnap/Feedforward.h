#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

/*
  Feedforward.h — steady-state current preload for the MicroCENTUM MC-18 kit
  (v2.1, MicroCENTUM profile; see Profile.h).

  Why: on a setpoint change the PID integrator is cleared, so the loop
  restarts from zero drive. The MC-18 cold side responds in ~1-2 s, so on
  a warming step (e.g. -27 -> 10 C in Cool mode) it free-runs toward
  ambient and overshoots by ~5 C before the integral rebuilds the
  ~180 mA hold current (bench 2026-10-05, s2: 5.6 C at Kp 17 / Ki 9,
  4.8 C at Kp 26 / Ki 14).

  What: the 2026-10-05 open-loop sweep (s1, 100 mA steps, steady state)
  gives the hold current needed for a given dT = Thot - Tcold on this
  fixture (35 mm CryoSnap heatsink, 30 mm fan at 255, ~23 C ambient).
  On every setpoint change and enable edge in Cool mode the integrator
  is preloaded with that current, so the PID only trims around it.

  Table: measured INA current (mA) vs measured dT (0.1 C units). The
  700 mA command delivered 775 mA, so the table is keyed by delivered
  current, not command. Above the last point the preload clamps.
*/

#include <avr/pgmspace.h>

#define FF_POINTS 18
static const uint16_t FF_MA[FF_POINTS] PROGMEM = {
     0,   78,  196,  303,  405,  511,  619,  775,  798,
   895,  995, 1093, 1193, 1293, 1393, 1494, 1593, 1693 };
static const int16_t  FF_DT10[FF_POINTS] PROGMEM = {
     0,   54,  128,  188,  239,  285,  325,  380,  388,
   417,  442,  465,  484,  501,  519,  534,  546,  555 };

// Hold current (mA, >= 0) for a required dT (C). 0 when no cooling needed.
inline float ff_holdmA(float dT) {
  if (!isfinite(dT) || dT <= 0.0f) return 0.0f;
  float d10 = dT * 10.0f;
  int16_t prev_d = 0; uint16_t prev_i = 0;
  for (uint8_t k = 1; k < FF_POINTS; k++) {
    int16_t  d = (int16_t)pgm_read_word(&FF_DT10[k]);
    uint16_t i = pgm_read_word(&FF_MA[k]);
    if (d10 <= d) {
      return prev_i + (float)(i - prev_i) * (d10 - prev_d) / (float)(d - prev_d);
    }
    prev_d = d; prev_i = i;
  }
  return (float)prev_i;
}

// Preload the integrator so the first PID output is the hold current.
// Cool mode output is negative (out_min = -imax, out_max = 0).
inline void pid_ff_preload(float setpoint, float t_hot, float imax_mA) {
  float ki = pid_getKi();
  if (ki <= 0.0f || !isfinite(setpoint) || !isfinite(t_hot)) return;
  float ff = ff_holdmA(t_hot - setpoint);
  if (ff > imax_mA) ff = imax_mA;
  pid_setIntegral(-ff / ki);
}

// PFM over-delivery trim (MC-18, staircase s5 2026-10-05, mean of up and
// down): in pulse mode the TPS55288 delivers more than its I_limit between
// 450 and ~735 mA (+5 mA at 450 rising to +32 at 700; flat ~0.73 A above).
// The command is reduced by the expected excess. Only applied in PFM.
#define TRIM_POINTS 7
static const uint16_t TRIM_AT[TRIM_POINTS] PROGMEM = { 450, 500, 550, 600, 650, 700, 735 };
static const uint8_t  TRIM_BY[TRIM_POINTS] PROGMEM = {   5,  10,  15,  19,  27,  32,   0 };

inline uint16_t pfm_trim_mA(uint16_t mA) {
  uint16_t a0 = pgm_read_word(&TRIM_AT[0]);
  if (mA <= a0) return mA;
  for (uint8_t k = 1; k < TRIM_POINTS; k++) {
    uint16_t a1 = pgm_read_word(&TRIM_AT[k]);
    if (mA <= a1) {
      uint8_t b0 = pgm_read_byte(&TRIM_BY[k - 1]), b1 = pgm_read_byte(&TRIM_BY[k]);
      int16_t d = (int16_t)b0 + (int16_t)(((int32_t)(b1 - b0) * (int32_t)(mA - a0)) / (int32_t)(a1 - a0));
      return mA - (uint16_t)d;
    }
    a0 = a1;
  }
  return mA;
}

#endif // FEEDFORWARD_H
