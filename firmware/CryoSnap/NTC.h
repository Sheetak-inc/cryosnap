#ifndef NTC_H
#define NTC_H

#include <Arduino.h>
#include "Pins.h"
#include "Config.h"

/*
  NTC.h — 3-channel thermistor read with running average.

  Three NTC inputs come into the Nano ADC after op-amp conditioning
  (TLV9004 quad op-amp on the Rev A board). Read all three each
  100 ms tick.

  Channel roles in THIS control scheme:
    NTC_1 (A1) — cold-side temperature.
                 This is the ONLY sensor the bang-bang / deadband /
                 damping control loop uses for its decision.

    NTC_2 (A2) — hot-side temperature.
                 Read and reported on the Serial Plotter but NOT
                 used for control in this rev. A more elaborate
                 scheme could use it for hot-side protection or
                 delta-T limiting.

    NTC_3 (A3) — ambient temperature.
                 Read and reported but NOT used for control. Could
                 feed an auto-derate or efficiency calculation in a
                 future rev.

  All three appear on the plotter line so the maker always has
  visibility, even though only NTC_1 drives the TEC.

  Temperature conversion
  ----------------------
  Uses a linear model:
    T(C) = raw_ADC * NTC_SCALE + NTC_OFFSET

  Defaults in Config.h (NTC_SCALE=0.1023, NTC_OFFSET=-27.6) are
  empirical for the bench prototype's NTC + op-amp network.
  Recalibrate via the `cal1` / `cal2` two-point serial commands
  if the NTC part or conditioning circuit changes.

  Running average
  ---------------
  Each channel maintains a circular buffer of NTC_AVG_LEN raw ADC
  samples (default 10). The conversion is applied to the averaged
  raw value for stability. At the 100 ms tick rate, the average
  spans the last 1 second of readings.
*/

// Runtime calibration — initialized from Config.h, adjustable via
// serial "cal" commands. Volatile (lost on reboot).
static float _ntc_scale  = NTC_SCALE;
static float _ntc_offset = NTC_OFFSET;

// Per-channel circular buffer for running average.
static uint16_t _ntc_buf[3][NTC_AVG_LEN];
static uint8_t  _ntc_idx = 0;
static bool     _ntc_filled = false;  // true after first NTC_AVG_LEN samples

// Cached averaged raw ADC and converted temperatures.
static float _ntc_raw_avg[3] = { 0, 0, 0 };
static float _ntc_temp[3] = { NAN, NAN, NAN };

inline void ntc_init() {
#if USE_EXTERNAL_AREF
  analogReference(EXTERNAL);
#endif
  // Zero the buffers.
  memset(_ntc_buf, 0, sizeof(_ntc_buf));
}

// Read all three NTC channels, feed the running average, and cache
// the converted temperatures. Call once per 100 ms tick. Returns the
// cold-side temperature (NTC_1) directly.
inline float ntc_readAll() {
  static const uint8_t pins[3] = { HW_NTC_1, HW_NTC_2, HW_NTC_3 };

  for (uint8_t ch = 0; ch < 3; ++ch) {
    _ntc_buf[ch][_ntc_idx] = analogRead(pins[ch]);
  }
  _ntc_idx = (_ntc_idx + 1) % NTC_AVG_LEN;
  if (_ntc_idx == 0) _ntc_filled = true;

  // Compute average from the filled portion of the buffer.
  uint8_t count = _ntc_filled ? NTC_AVG_LEN : _ntc_idx;
  if (count == 0) count = 1;  // guard against /0 on first call

  for (uint8_t ch = 0; ch < 3; ++ch) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; ++i) sum += _ntc_buf[ch][i];
    _ntc_raw_avg[ch] = (float)sum / count;
    _ntc_temp[ch] = _ntc_raw_avg[ch] * _ntc_scale + _ntc_offset;
  }

  return _ntc_temp[0];  // cold-side
}

// Get the cached temperature for a given channel (1, 2, or 3).
inline float ntc_getC(uint8_t chan) {
  if (chan < 1 || chan > 3) return NAN;
  return _ntc_temp[chan - 1];
}

// Get the averaged raw ADC value for a channel (for calibration).
inline float ntc_getRawAvg(uint8_t chan) {
  if (chan < 1 || chan > 3) return 0;
  return _ntc_raw_avg[chan - 1];
}

// Get/set the runtime calibration constants.
inline float ntc_getScale()  { return _ntc_scale; }
inline float ntc_getOffset() { return _ntc_offset; }
inline void  ntc_setScale(float s)  { _ntc_scale = s; }
inline void  ntc_setOffset(float o) { _ntc_offset = o; }

#endif // NTC_H
