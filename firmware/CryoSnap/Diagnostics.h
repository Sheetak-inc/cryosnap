#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>
#include "Config.h"

/*
  Diagnostics.h — cross-chip consistency checks, run from task_slow().

  Reports state-change events advisory-only. None of these latch a
  fault — the TPS handles its own catastrophic shutdowns; this
  module just helps the operator pinpoint *which* part of the chain
  has gone sideways when something behaves unexpectedly.

    (a) HUSB negotiated, but TPS doesn't ACK — the input rail is
        there but the converter is off. Usually the bench rocker
        switch or a USB-C cable issue.
    (c) TPS responding, but INA doesn't ACK — current sensing died.
    (d) TPS responding, but CDC register reverted to power-on
        default (0xE0 instead of TPS_CDC_OPMODE=0xA0). The chip
        silently reset its registers — usually a Vin brownout
        during high-current drive that didn't go far enough to
        drop the I2C interface. Firmware-side state is now stale.
        Recover via _tps_only_recover() which re-runs tps_init()
        and clears the supply-fault counters. BUG-003 addendum
        Fix 2.
    (e) TPS OE on, but INA reads ~0 A for several ticks — probably
        a disconnected or open TEC.

  SCP detection (case d) lives in task_100ms — the TPS STATUS
  register is polled at the fast cadence so a real short surfaces
  within one tick. The chip-presence pings here stay at 1 s
  because the 25 ms timeout-on-missing-chip would blow the 100 ms
  task budget.

  Skipping (b) per spec ("HUSB not negotiated, TPS active — ignore
  HUSB"): no message; the firmware already trusts the TPS in that
  case.

  To add another cross-chip check: follow the pattern below — keep
  a `static bool _x_last` to track the previous state, fire on the
  rising edge, then update the snapshot at the bottom of the
  function. To silence a category: comment out the corresponding
  Serial.println — the underlying state tracking is fine to leave
  in place.

  Gated by ENABLE_DIAGNOSTICS in Config.h.
*/

#if ENABLE_DIAGNOSTICS

inline void task_diag() {
  // Snapshot last-tick state so we only print on rising edges.
  static bool    _husb_neg_last     = true;
  static bool    _tps_resp_last     = true;
  static bool    _ina_resp_last     = true;
  static uint8_t _no_current_count  = 0;

  bool husb_neg = (husb_negotiatedV() == 20);
  bool tps_resp = i2cPing(I2C_ADDR_TPS55288);
  bool ina_resp = i2cPing(I2C_ADDR_INA226);

  bool oe_on = false;
  if (tps_resp) {
    uint8_t mode_reg = _tps_read(TPS_REG_MODE);
    oe_on = (mode_reg & TPS_MODE_OE_BIT) != 0;
  }

  // (a) PD ok but TPS missing — fire on the transition into that state.
  if (husb_neg && !tps_resp && (!_husb_neg_last || _tps_resp_last)) {
    Serial.println(F("DIAG: PD ok but TPS not responding -- power switch off or USB-C unplugged"));
  }

  // (c) TPS ok but INA missing.
  if (tps_resp && !ina_resp && (!_tps_resp_last || _ina_resp_last)) {
    Serial.println(F("DIAG: Current sensing INA226 offline"));
  }

  // (d) TPS ok but CDC reverted to power-on default — the chip
  // silently reset during a Vin transient and tps_init's CDC write
  // never re-landed. Without this check, the firmware would happily
  // command V/I writes that the chip might also drop, eventually
  // accumulating supply-fault counter increments and tripping NOPSU
  // for the wrong reason (BUG-003 addendum 2026-06-05). Cheap: one
  // I2C byte read at 1 s cadence.
  if (tps_resp) {
    uint8_t cdc = tps_getCDC();
    if (tps_lastReadOk() && cdc != TPS_CDC_OPMODE) {
      // Silent recover — drift is invisible to the operator but
      // brief; trips surface via NACK counter if recovery fails.
      _tps_only_recover();
    }
  }

  // (e) TPS driving but no current visible at INA. Debounce for 3
  // consecutive ticks so a brief off-period or transient doesn't fire.
  if (g_enabled && oe_on && ina_resp) {
    float i = ina_readCurrentA();
    if (fabs(i) < 0.05f) {
      if (++_no_current_count == 3) {
        Serial.println(F("DIAG: TPS driving but no current -- check for disconnected or failed TEC"));
      }
    } else {
      _no_current_count = 0;
    }
  } else {
    _no_current_count = 0;
  }

  _husb_neg_last = husb_neg;
  _tps_resp_last = tps_resp;
  _ina_resp_last = ina_resp;
}

#else  // !ENABLE_DIAGNOSTICS — stub so callers compile unconditionally
inline void task_diag() {}
#endif

#endif // DIAGNOSTICS_H
