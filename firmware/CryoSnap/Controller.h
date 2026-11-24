#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

/*
  Controller.h — one-shot time-anchored parameter sweep.

  A `controller` console command fires user_controller(elapsed_s)
  ONCE at a chosen elapsed time. The firmware provides the
  scheduling; the actual profile lives in user_controller() below
  — edit that body for your demo run.

  Console:
    controller          arm; fire at elapsed_s = 0 on the NEXT
                        enable button press
    controller <s>      fire NOW once at elapsed_s = s
    controller -1       cancel an armed / pending fire

  One-shot semantics: user_controller() runs exactly ONCE per
  command. If you want a stepped profile, either:
    - issue successive `controller` commands as you go, or
    - schedule the next step from inside user_controller()
      itself by calling ctrl_start(next_s).

  All globals from CryoSnap.ino are visible here because
  this header is included AFTER the runtime-state block. Mutate
  g_setpoint, g_kp/Ki/Kd, g_imax_mA, g_mode, g_use_pid, etc. as
  your profile needs.
*/

static struct {
  bool          active;     // schedule pending — fires when elapsed reaches fire_s
  bool          armed;      // waiting for enable rising edge
  unsigned long t0_ms;      // anchor: millis() when elapsed_s = 0
  float         fire_s;     // scheduled fire time
} _ctrl = { false, false, 0, 0 };

inline bool ctrl_active() { return _ctrl.active; }
inline bool ctrl_armed()  { return _ctrl.armed;  }

// Cancel any pending or armed fire.
inline void ctrl_stop() {
  _ctrl.active = false;
  _ctrl.armed  = false;
}

// Arm: fire at elapsed_s = 0 on the next enable button press.
// Used by the no-arg `controller` command.
inline void ctrl_arm() {
  _ctrl.armed   = true;
  _ctrl.active  = false;
  _ctrl.fire_s  = 0.0f;
}

// Schedule a fire NOW at the given elapsed time (one-shot).
inline void ctrl_start(float fire_s) {
  _ctrl.armed   = false;
  _ctrl.active  = true;
  _ctrl.t0_ms   = millis();
  _ctrl.fire_s  = fire_s;
}

// Called by the enable rising-edge handlers (button + serial cmd).
// Promotes an armed schedule to active, anchored to NOW.
inline void ctrl_on_enable() {
  if (_ctrl.armed) {
    _ctrl.armed   = false;
    _ctrl.active  = true;
    _ctrl.t0_ms   = millis();
    _ctrl.fire_s  = 0.0f;
  }
}

// Forward decl — user_controller() is the customisation point and
// sits in the middle of this file for easy editing; ctrl_tick()
// lives at the bottom.
inline void ctrl_tick();

// =========================================================================
// USER CUSTOMIZATION ZONE — edit user_controller() to define your
// time-based profile. Default body is empty so the firmware compiles
// cleanly with no modification.
// =========================================================================

inline void user_controller(float elapsed_s) {
  // EXAMPLE — set a cool setpoint for a quick demo step:
  //
  //   g_setpoint = 5.0f;        // step to 5 °C
  //   g_use_pid  = true;        // make sure PID is selected
  //
  // Or chain into a stepped profile by re-arming for the next step:
  //
  //   if      (elapsed_s <  60.0f) { g_setpoint = 25.0f; ctrl_start(60.0f);  }
  //   else if (elapsed_s < 180.0f) { g_setpoint = 15.0f; ctrl_start(180.0f); }
  //   else if (elapsed_s < 300.0f) { g_setpoint =  5.0f; ctrl_start(300.0f); }
  //   else                         { /* done — no further fires */ }
  //
  // All the firmware globals are in scope: g_setpoint, g_kp/Ki/Kd,
  // g_imax_mA, g_mode, g_use_pid, g_enabled, ... mutate as you like.
  //
  // Default body is empty — add your profile above and rebuild.
  (void)elapsed_s;
}

// =========================================================================

inline void ctrl_tick() {
  if (!_ctrl.active) return;
  float elapsed_s = (millis() - _ctrl.t0_ms) / 1000.0f;
  if (elapsed_s < _ctrl.fire_s) return;

  // One-shot: clear before the call so the user body can re-arm or
  // schedule the next step via ctrl_start() without us overwriting it.
  _ctrl.active = false;
  user_controller(_ctrl.fire_s);
}

#endif // CONTROLLER_H
