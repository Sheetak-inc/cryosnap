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
    hb_init()                   configure PIN_DIR as output, set LOW
    hb_setDirection(dir)        set direction (HEAT / COOL)
    hb_getDirection()           read current direction
    hb_safeDirectionChange(dir) TPS off -> settle -> toggle -> TPS on
*/

// H-bridge direction polarity. On the Rev A PCB, LOW=Cool / HIGH=Heat.
// On the bench prototype the wiring is inverted — swap here so the
// control logic doesn't need to know.
#if BUILD_TARGET == TARGET_PROTO
  #define HB_COOL  HIGH   // prototype: inverted wiring
  #define HB_HEAT  LOW
#else
  #define HB_COOL  LOW    // Rev A PCB: correct wiring
  #define HB_HEAT  HIGH
#endif

// Settle time after disabling TPS before toggling direction.
// 5 ms is conservative for the output caps to discharge; the MOSFET
// gate driver responds in microseconds.
#define HB_SETTLE_MS  5

static uint8_t _hb_direction = HB_COOL;

// Configure PIN_DIR as output, default to COOL (LOW).
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
