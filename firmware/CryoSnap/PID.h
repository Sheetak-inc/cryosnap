#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "Config.h"

/*
  PID.h — minimal closed-loop controller for TEC drive.

  Lightweight, explainable, no library dependencies. The controller
  takes a setpoint and a measured temperature (both in degrees C)
  and produces a SIGNED current command in milliamps:

    output > 0   →  drive direction = HEAT, magnitude =  output
    output < 0   →  drive direction = COOL, magnitude = -output
    output == 0  →  TEC off

  Sign convention:

      error      = setpoint - measured
      integral  += error * dt
      derivative = (error - lastError) / dt
      output     = Kp*error + Ki*integral + Kd*derivative
      lastError  = error

  Anti-windup
  -----------
  The integrator is clamped so Ki * integral cannot, on its own,
  exceed the saturation ceiling passed in as out_max — typically
  the configured TPS current limit. When the chip is already
  current-limited and the firmware can't push harder, the
  integrator stops growing instead of running away. (A small
  improvement on the textbook PID — without it, a brief
  saturation event leaves a long-lived integrator residual.)

  The output itself is hard-clamped to ±out_max as a final
  belt-and-braces step so the caller can map directly to drive_mA.

  State and tuning constants are file-static; serial commands
  and EEPROM persistence go through the get/set helpers, the
  same pattern NTC.h uses for its calibration constants.

  Public API:
    pid_init()                    one-shot, currently just resets state
    pid_reset()                   clear integral + lastError
    pid_compute(sp, m, dt, max)   returns signed mA output
    pid_set{Kp,Ki,Kd}(v)          update tuning constants
    pid_get{Kp,Ki,Kd}()           read back tuning constants

  ---------------------------------------------------------------
  HOW TO TUNE (starting points for a new TEC/heatsink combo)
  ---------------------------------------------------------------
  Defaults in Config.h are conservative for the CryoSnap dev-kit
  TEC + stock heatsink/fan: Kp=200 mA/°C, Ki=5 mA/(°C·s), Kd=0.
  If you swap the TEC, the cold mass, or the heatsink, expect to
  retune. Live-edit from the serial console:

      kp 250        set Kp to 250 mA/°C
      ki 8          set Ki (also resets the integrator)
      kd 0          keep Kd at 0 until the rest of the loop is calm
      save          persist to EEPROM once you like the result

  A pragmatic tuning recipe (Ziegler-Nichols-lite):

    1. Set Ki = 0, Kd = 0. Pick a step (e.g. setpoint 25 → 15 °C).
    2. Raise Kp until the response oscillates steadily around the
       setpoint without quite settling. Call that gain Ku and the
       oscillation period Tu (seconds).
    3. Back off to Kp ≈ 0.6 * Ku. Then set Ki ≈ 1.2 * Kp / Tu.
       Leave Kd = 0 unless you have measurable overshoot you can't
       remove with Kp/Ki alone — derivative on noisy NTC readings
       often hurts more than it helps.
    4. Watch the `plot` stream while you nudge values. The signed
       drive_mA in that stream is exactly what pid_compute() returned
       (clamped to imax), so you can see the integrator unwinding.

  Symptom → knob:
    slow to reach setpoint, no overshoot   → raise Kp
    overshoots and rings                   → lower Kp, or add small Kd
    settles offset-low / offset-high       → raise Ki
    integrator winds up during current-limit→ already handled (anti-windup
                                              clamp below); verify by
                                              checking _pid_integral stays
                                              within ±out_max/Ki
    jitter / fan-speed coupling            → lower Kd or set to 0,
                                             check NTC averaging in NTC.h

  dt is fixed at LOOP_INTERVAL_MS / 1000 (0.1 s by default) because
  task_100ms() calls pid_compute() on a strict cadence. If you change
  LOOP_INTERVAL_MS in Config.h, Ki and Kd both scale with it — re-tune.
*/

static float _pid_kp       = DEFAULT_KP;
static float _pid_ki       = DEFAULT_KI;
static float _pid_kd       = DEFAULT_KD;
static float _pid_integral = 0.0f;
static float _pid_last_err = 0.0f;

inline void pid_reset() {
  _pid_integral = 0.0f;
  _pid_last_err = 0.0f;
}

inline void pid_init() {
  pid_reset();
}

// Compute the next PID output. setpoint and measured are in
// degrees C, dt is in seconds, out_max is the saturation ceiling
// in the same unit as the output (milliamps in our use).
//
// Output is signed: positive = heat, negative = cool. Caller
// maps the sign to the H-bridge direction and abs() to drive_mA.
inline float pid_compute(float setpoint, float measured,
                         float dt, float out_max) {
  float err = setpoint - measured;

  // Integrate then clamp. Skip integration entirely if Ki is zero
  // — keep the residual at zero so toggling Ki later doesn't snap
  // a stale value through.
  if (_pid_ki > 0.0f) {
    _pid_integral += err * dt;
    float i_lim = out_max / _pid_ki;
    if (_pid_integral >  i_lim) _pid_integral =  i_lim;
    if (_pid_integral < -i_lim) _pid_integral = -i_lim;
  } else {
    _pid_integral = 0.0f;
  }

  float deriv = (err - _pid_last_err) / dt;
  _pid_last_err = err;

  float out = _pid_kp * err
            + _pid_ki * _pid_integral
            + _pid_kd * deriv;

  if (out >  out_max) out =  out_max;
  if (out < -out_max) out = -out_max;
  return out;
}

inline void  pid_setKp(float v) { _pid_kp = v; }
inline void  pid_setKi(float v) { _pid_ki = v; pid_reset(); }
inline void  pid_setKd(float v) { _pid_kd = v; }
inline float pid_getKp() { return _pid_kp; }
inline float pid_getKi() { return _pid_ki; }
inline float pid_getKd() { return _pid_kd; }

#endif // PID_H
