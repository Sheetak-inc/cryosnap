#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <Arduino.h>
#include "Pins.h"
#include "TPS55288.h"

/*
  HBridge.h — H-bridge direction control.

  The DRV8701E is a gate driver with a single PH/EN interface. On
  this board EN is hardwired to 3.3 V (always enabled) and NSLEEP is
  hardwired to 3.3 V (always awake). The firmware only controls the
  direction (heat vs cool) via PIN_DIR = D6.

  This header wraps that single GPIO in a small API so the H-bridge
  has its own file like every other peripheral. If a future rev uses
  a more capable driver (SPI-configured, current-sense feedback,
  etc.) the API surface stays the same and only this file changes.

  Direction-change safety:
    When the TEC polarity needs to flip while the TPS output is
    active, we must disable the TPS first, let the output settle,
    toggle PIN_DIR, then re-enable. This prevents shoot-through
    current in the H-bridge MOSFETs — especially important when
    deadband is configured to zero.

  Public API:
    hb_init()                   configure PIN_DIR as output, default HB_COOL
    hb_setDirection(dir)        set direction (HEAT / COOL)
    hb_getDirection()           read current direction
    hb_safeDirectionChange(dir) TPS off -> settle -> toggle -> TPS on
*/

// H-bridge direction polarity. Bench-confirmed on Rev A 2026-06-03:
// the production PCB wiring uses the same polarity as the prototype
// (LOW = drive toward Heat side, HIGH = drive toward Cool side).
// The earlier Rev A comment claimed the opposite — that was wrong,
// and the device heated when commanded to cool. Tracked in BUG-000
// of the audit log.
//
// TARGET_REVB is currently assumed to share Rev A's wiring. This is
// an UNVERIFIED guess until someone exercises the heat/cool commands
// on Rev B silicon and confirms the thermal direction matches the
// labels. If a Rev B build heats when commanded to cool, swap the
// HB_COOL/HB_HEAT pair in the TARGET_REVB branch below.
#if BUILD_TARGET == TARGET_PROTO
  #define HB_COOL  HIGH
  #define HB_HEAT  LOW
#elif BUILD_TARGET == TARGET_REVB
  #warning "TARGET_REVB H-bridge polarity is inherited from Rev A and is NOT bench-verified. See HBridge.h. If the device heats when commanded to cool, swap HB_COOL/HB_HEAT here."
  #define HB_COOL  HIGH
  #define HB_HEAT  LOW
#else  // TARGET_REVA
  #define HB_COOL  HIGH
  #define HB_HEAT  LOW
#endif

// Settle time after disabling TPS before toggling direction.
// 5 ms is conservative for the output caps to discharge; the MOSFET
// gate driver responds in microseconds.
#define HB_SETTLE_MS  5

static uint8_t _hb_direction = HB_COOL;

// Configure PIN_DIR as output and default to HB_COOL. The actual
// HIGH/LOW level depends on BUILD_TARGET (see the polarity block
// above) — on TARGET_REVA/TARGET_REVB this is HIGH, on the
// prototype it is also HIGH. Direction only matters when TPS
// output is enabled, so this default is just to avoid leaving the
// pin floating at boot.
inline void hb_init() {
  pinMode(HW_HB_DIR, OUTPUT);
  digitalWrite(HW_HB_DIR, HB_COOL);
  _hb_direction = HB_COOL;
}

// Set direction without safety sequence. Use only when TPS output
// is already disabled (e.g. during init or inside deadband).
inline void hb_setDirection(uint8_t dir) {
  _hb_direction = dir;
  digitalWrite(HW_HB_DIR, dir);
}

// Read the current direction.
inline uint8_t hb_getDirection() {
  return _hb_direction;
}

// Safe direction change: disable TPS output, wait for the bus to
// settle, toggle PIN_DIR, then re-enable TPS. Call this from the
// 100 ms control task when the direction needs to flip while drive
// is active. The 5 ms delay is acceptable inside the 100 ms tick
// since direction changes are rare events.
inline void hb_safeDirectionChange(uint8_t newDir) {
  if (newDir == _hb_direction) return;  // no change needed
  tps_setOutput(false);
  delay(HB_SETTLE_MS);
  hb_setDirection(newDir);
  tps_setOutput(true);
}

#endif // HBRIDGE_H
