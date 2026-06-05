#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <math.h>
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
      derivative = -d(measured)/dt   (derivative-on-measurement;
                                      d(setpoint-measured)/dt with
                                      sp constant equals -d(m)/dt)
      output     = Kp*error + Ki*integral(error*dt) + Kd*derivative

  Anti-windup (BUG-002 + BUG-006 in the 2026-06-03 audit)
  -------------------------------------------------------
  Conditional integration. Every tick we compute the PROPOSED total
  output as if integration happened, and only commit the integration
  if that proposal is not saturating in the same direction as the
  current error. This correctly handles two failure modes:

    (a) Hard PID saturation against the [out_min, out_max] bounds.
        Old fixed-clamp anti-windup let the integral grow up to
        ±out_max/Ki even while the proportional term was already
        pinning the output at the bound — long settling tails
        after big steps.
    (b) Mode-forbidden direction. The caller bounds [out_min,
        out_max] to express Cool / Heat / Auto:
              MODE_COOL:  out_min = -imax, out_max = 0
              MODE_HEAT:  out_min = 0,     out_max = +imax
              MODE_AUTO:  out_min = -imax, out_max = +imax
        Old code integrated freely then post-clamped; an extended
        wrong-side excursion could leave a residual that delayed
        the correct direction by seconds-to-minutes once the
        actuator was free again. Bench-measured 383× delay in the
        audit. Conditional integration freezes the integrator the
        moment the proposal would push past the forbidden bound,
        so no residual builds.

  Derivative on measurement (BUG-007)
  -----------------------------------
  Differentiating the error gives a one-tick spike whenever the
  setpoint steps; differentiating the measurement is mathematically
  equivalent (with sp constant) and avoids the spike. The
  _pid_seeded flag suppresses derivative until measurement history
  has been seeded — either by a prior pid_compute() OR by
  pid_observe() in the deadband path. After pid_reset() the
  measurement is unseeded and the very next compute returns no
  derivative term, so the initial measurement doesn't appear to
  have "jumped" from 0.

  Time-step robustness (BUG-008 + BUG-010)
  ----------------------------------------
  Caller passes the *actual* elapsed time between ticks (measured
  via millis()), not a hardcoded nominal. Stretched ticks (fan
  tach poll, fault dumps, direction-flip delay) get correct
  integration and derivative math instead of fictional 0.1 s.

  pid_compute() also guards against pathological inputs
  (non-finite setpoint / measurement / dt, or dt ≤ 0) and
  returns 0 in those cases
  rather than poisoning the TPS current command with inf/NaN.

  State and tuning constants are file-static; serial commands
  and EEPROM persistence go through the get/set helpers, the
  same pattern NTC.h uses for its calibration constants.

  Public API:
    pid_init()                          one-shot, currently just resets state
    pid_reset()                         clear integral + measurement seeding
    pid_compute(sp, m, dt, min, max)    returns signed mA output
                                        bounded to [min, max]
    pid_observe(measured)               update measurement history WITHOUT
                                        computing output. Use when the
                                        caller bypasses pid_compute (e.g.
                                        inside deadband) so the next real
                                        compute doesn't see a stale derivative.
    pid_set{Kp,Ki,Kd}(v)                update tuning constants
    pid_get{Kp,Ki,Kd}()                 read back tuning constants

  ---------------------------------------------------------------
  HOW TO TUNE (starting points for a new TEC/heatsink combo)
  ---------------------------------------------------------------
  Defaults in Config.h are conservative for the CryoSnap dev-kit
  TEC + stock heatsink/fan: Kp=200 mA/°C, Ki=5 mA/(°C·s), Kd=0.
  If you swap the TEC, the cold mass, or the heatsink, expect to
  retune. Live-edit from the serial console:

      kp 250        set Kp to 250 mA/°C
      ki 8          set Ki (also resets the integrator;
                    derivative history is preserved)
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
       often hurts more than it helps. With the
       derivative-on-measurement change in 0.7.8, raising Kd no
       longer kicks on setpoint steps, but NTC noise still feeds
       directly into the derivative.
    4. Watch the `plot` stream while you nudge values. The signed
       drive_mA in that stream is exactly what pid_compute() returned
       (clamped to imax), so you can see the integrator unwinding.

  Symptom → knob:
    slow to reach setpoint, no overshoot   → raise Kp
    overshoots and rings                   → lower Kp, or add small Kd
    settles offset-low / offset-high       → raise Ki
    jitter / fan-speed coupling            → lower Kd or set to 0,
                                             check NTC averaging in NTC.h

  The caller (task_100ms) measures real elapsed time and passes it
  as dt, so re-tuning is not required when LOOP_INTERVAL_MS changes
  — gains stay in physical units (mA/°C and mA/(°C·s)).
*/

static float _pid_kp            = DEFAULT_KP;
static float _pid_ki            = DEFAULT_KI;
static float _pid_kd            = DEFAULT_KD;
static float _pid_integral      = 0.0f;
static float _pid_last_measured = 0.0f;
static bool  _pid_seeded        = false;  // false until first pid_compute / pid_observe after reset

inline void pid_reset() {
  _pid_integral = 0.0f;
  _pid_seeded   = false;  // first compute after reset has no derivative kick
}

inline void pid_init() {
  pid_reset();
}

// Update the measurement history without computing an output. Call
// every tick where the caller is bypassing pid_compute() (e.g.
// inside the deadband) — without this, leaving the deadband would
// see a stale _pid_last_measured and produce a one-tick derivative
// spike proportional to how much the temperature drifted while we
// weren't looking.
inline void pid_observe(float measured) {
  if (isfinite(measured)) {
    _pid_last_measured = measured;
    _pid_seeded        = true;
  }
}

// Compute the next PID output. setpoint and measured are in
// degrees Celsius, dt is the elapsed time since the previous call
// in seconds (caller measures via millis()), and [out_min, out_max]
// bounds the legal output range.
//
// out_min/out_max express the Mode constraint:
//   MODE_COOL:  out_min = -imax,  out_max =  0
//   MODE_HEAT:  out_min =  0,     out_max = +imax
//   MODE_AUTO:  out_min = -imax,  out_max = +imax
//
// The bounds participate in the anti-windup decision; integration is
// frozen the moment the proposed total output would saturate in the
// direction of the current error.
inline float pid_compute(float setpoint, float measured,
                         float dt, float out_min, float out_max) {
  // BUG-010: pathological inputs (non-finite setpoint / measurement /
  // dt, or dt <= 0) return a safe zero rather than poisoning the TPS
  // current command with inf/NaN. Caller will see drive_mA=0 and the
  // TEC stays off until inputs recover. NOTE: the early-return path
  // intentionally leaves _pid_last_measured and _pid_seeded
  // untouched, so a future caller that bypasses this guard for some
  // ticks (the current caller doesn't — it gates on !isnan before
  // entering the PID branch) would still see consistent derivative
  // history once valid inputs resume.
  if (!isfinite(setpoint) || !isfinite(measured)
      || !isfinite(dt)    || dt <= 0.0f) {
    return 0.0f;
  }

  float err = setpoint - measured;

  // BUG-007: derivative on MEASUREMENT (mathematically equivalent
  // to derivative on error when setpoint is constant, but free of
  // the one-tick kick when setpoint steps). First call after reset
  // has no history → derivative is 0.
  float d_term = 0.0f;
  if (_pid_seeded && _pid_kd != 0.0f) {
    d_term = _pid_kd * (-(measured - _pid_last_measured) / dt);
  }
  _pid_last_measured = measured;
  _pid_seeded        = true;

  float p_term = _pid_kp * err;

  // BUG-002 + BUG-006: conditional integration. Compute the proposed
  // total output as if we integrated, and only commit the integration
  // if the proposal is NOT saturating in the err direction.
  if (_pid_ki > 0.0f) {
    float i_proposed      = _pid_integral + err * dt;
    float i_term_proposed = _pid_ki * i_proposed;
    float total_proposed  = p_term + i_term_proposed + d_term;

    bool clamp_high = (total_proposed >= out_max) && (err > 0.0f);
    bool clamp_low  = (total_proposed <= out_min) && (err < 0.0f);

    if (!clamp_high && !clamp_low) {
      _pid_integral = i_proposed;
    }
    // else: hold the integral steady. NOT reset to 0 — that would
    // lose progress; merely freezing it lets the proportional term
    // do the work until the actuator is free again.
  } else {
    _pid_integral = 0.0f;
  }

  float out = p_term + _pid_ki * _pid_integral + d_term;

  // Final hard clamp to the legal range. Belt-and-braces against
  // numerical edge cases where total_proposed straddled a bound.
  if (out > out_max) out = out_max;
  if (out < out_min) out = out_min;
  return out;
}

inline void  pid_setKp(float v) { _pid_kp = v; }
inline void  pid_setKi(float v) {
  _pid_ki       = v;
  _pid_integral = 0.0f;  // changing Ki invalidates the accumulated integral
  // _pid_seeded stays — derivative tracking is independent of Ki
}
inline void  pid_setKd(float v) { _pid_kd = v; }
inline float pid_getKp() { return _pid_kp; }
inline float pid_getKi() { return _pid_ki; }
inline float pid_getKd() { return _pid_kd; }

#endif // PID_H
