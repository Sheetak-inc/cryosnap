#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H

#include <Arduino.h>
#include "Pins.h"

/*
  FanControl.h — 4-wire fan PWM + tach driver.

  PWM output
  ----------
  HW_FAN_PWM = D3 = OC2B (Timer2 output compare B).

  4-wire PC fans expect a ~25 kHz PWM signal. Timer2 is configured in
  Fast PWM mode with TOP = OCR2A = 79 and prescaler = 8:

    f_pwm = F_CPU / (prescaler * (1 + TOP))
          = 16 000 000 / (8 * 80)
          = 25 000 Hz

  Duty cycle is controlled by OCR2B:
    OCR2B = 0   -> 0% (fan off or at minimum speed, depends on fan)
    OCR2B = 79  -> 100% (full speed)

  The public API takes 0..255 for familiarity with analogWrite(), and
  maps it internally to 0..79.

  Tach input
  ----------
  HW_FAN_TACH = D2.

  Polled approach — no ISR. The function fan_pollRPM() blocks for up
  to 100 ms (or 10 edges, whichever comes first), times the period
  between consecutive falling edges, and averages them to compute RPM.

  Why no ISR: the bench prototype's tach line picks up PWM switching
  noise that inflates edge counts in an ISR. By polling and measuring
  actual inter-edge time, we can debounce properly — any "edge" that
  arrives less than FAN_TACH_DEBOUNCE_US after the previous one is
  rejected as noise.

  fan_pollRPM() is called from the 1 s slow task (not the 100 ms
  control task) because it blocks. 100 ms of blocking in a 1 s window
  is acceptable.

  Future enhancement: move tach to D5 (T1) and use Timer1 hardware
  counter mode — pure hardware counting with no blocking.

  Public API:
    fan_init()              configure Timer2 PWM, tach pin as input
    fan_setSpeed(duty)      set fan PWM (0..255 mapped to 0..79)
    fan_pollRPM()           blocking poll, returns averaged RPM
*/

// Timer2 configuration for 25 kHz PWM on OC2B (D3).
#define FAN_PWM_TOP  79   // (F_CPU / prescaler / 25000) - 1 = 79

// Tach: 2 pulses per revolution is the standard for PC fans.
#define FAN_TACH_PULSES_PER_REV  2

// Maximum time to spend polling for edges (microseconds).
#define FAN_TACH_WINDOW_US       100000UL   // 100 ms

// Maximum number of inter-edge periods to collect before returning.
#define FAN_TACH_MAX_EDGES       10

// Minimum microseconds between real falling edges. Anything shorter
// is rejected as noise from PWM switching coupling into the tach wire.
// At 6000 RPM with 2 pulses/rev: real period = 5000 us. We use
// 2000 us (half of a 6000 RPM period) as the floor.
#define FAN_TACH_DEBOUNCE_US     2000

// Maximum allowed deviation from the average period. If any single
// measured period deviates more than 25% from the batch average, the
// entire sample is considered noisy and discarded — we keep the
// previous RPM reading instead.
#define FAN_TACH_TOLERANCE_PCT  25

// Most recent RPM — updated by fan_pollRPM(), read by anyone.
static uint16_t _fan_last_rpm = 0;

// ---- public API ---------------------------------------------------------

inline void fan_init() {
  // --- PWM on D3 (OC2B) via Timer2 ---
  pinMode(HW_FAN_PWM, OUTPUT);

  // Timer2 Fast PWM mode 7: WGM2[2:0] = 111, TOP = OCR2A.
  //   TCCR2A: COM2B1=1 (non-inverting OC2B), WGM21=1, WGM20=1
  //   TCCR2B: WGM22=1, CS21=1 (prescaler=8)
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22)  | (1 << CS21);
  OCR2A  = FAN_PWM_TOP;   // TOP = 79 -> 25 kHz
  OCR2B  = 0;             // start with fan off

  // --- Tach on D2 ---
  // Plain INPUT — no pull-up from MCU. The board must provide an
  // external pull-up (~10K to 5 V) on the tach line; without it
  // the line floats and fan_pollRPM() reports zero. The Rev A
  // schematic includes this; on a bare breadboard prototype you'll
  // need to add the resistor yourself.
  pinMode(HW_FAN_TACH, INPUT);
}

// Set fan speed. duty = 0 (off) .. 255 (full). Internally mapped to
// 0..FAN_PWM_TOP (79) to match the Timer2 TOP value.
inline void fan_setSpeed(uint8_t duty) {
  // Map 0..255 -> 0..79. Use 16-bit intermediate to avoid overflow.
  OCR2B = (uint8_t)(((uint16_t)duty * FAN_PWM_TOP) / 255);
}

// Poll the tach line for up to FAN_TACH_WINDOW_US (100 ms) or
// FAN_TACH_MAX_EDGES (10) inter-edge periods, whichever comes first.
// Computes RPM from the average period. Built-in debounce rejects
// edges closer than FAN_TACH_DEBOUNCE_US apart.
//
// BLOCKING — call from the 1 s slow task, not the 100 ms control
// task. Returns 0 if no valid edges were seen (fan stalled or off).
inline uint16_t fan_pollRPM() {
  unsigned long window_start = micros();
  uint8_t  pin    = HW_FAN_TACH;
  uint8_t  count  = 0;           // number of measured periods
  uint32_t periods[FAN_TACH_MAX_EDGES];

  // --- Wait for the first falling edge (HIGH -> LOW) ---
  // Sync to a known point before we start timing.
  // Step 1: wait for pin to go HIGH (or timeout).
  while (digitalRead(pin) == LOW) {
    if (micros() - window_start > FAN_TACH_WINDOW_US) goto validate;
  }
  // Step 2: wait for the actual falling edge (HIGH -> LOW).
  while (digitalRead(pin) == HIGH) {
    if (micros() - window_start > FAN_TACH_WINDOW_US) goto validate;
  }

  {
    unsigned long last_edge = micros();

    // --- Collect up to FAN_TACH_MAX_EDGES inter-edge periods ---
    while (count < FAN_TACH_MAX_EDGES) {
      // Wait for pin to go HIGH again.
      while (digitalRead(pin) == LOW) {
        if (micros() - window_start > FAN_TACH_WINDOW_US) goto validate;
      }
      // Wait for next falling edge.
      while (digitalRead(pin) == HIGH) {
        if (micros() - window_start > FAN_TACH_WINDOW_US) goto validate;
      }

      unsigned long now = micros();
      unsigned long period = now - last_edge;

      // Debounce: reject edges that arrive too quickly (noise).
      if (period >= FAN_TACH_DEBOUNCE_US) {
        periods[count] = period;
        count++;
        last_edge = now;
      }
      // If the edge was too fast (noise), we just loop back and
      // wait for the next real one — last_edge stays unchanged.
    }
  }

validate:
  if (count == 0) {
    // No edges at all — fan is stopped or disconnected. Report 0.
    _fan_last_rpm = 0;
    return 0;
  }

  // Compute the average period.
  uint32_t sum_us = 0;
  for (uint8_t i = 0; i < count; ++i) sum_us += periods[i];
  uint32_t avg_period = sum_us / count;

  // Validate: every individual period must be within ±25% of the
  // average. If ANY sample deviates more, the whole batch is noisy
  // — discard it and keep the previous RPM reading. This catches
  // sporadic noise spikes that pass the debounce threshold but are
  // still wrong (e.g. one 3 ms glitch among 10 ms real periods).
  uint32_t lo = avg_period - (avg_period * FAN_TACH_TOLERANCE_PCT / 100);
  uint32_t hi = avg_period + (avg_period * FAN_TACH_TOLERANCE_PCT / 100);
  for (uint8_t i = 0; i < count; ++i) {
    if (periods[i] < lo || periods[i] > hi) {
      // Outlier detected — keep previous RPM, skip this batch.
      return _fan_last_rpm;
    }
  }

  // All periods are consistent — compute RPM.
  // RPM = 60 000 000 us/min / (avg_period_us * pulses_per_rev)
  _fan_last_rpm = (uint16_t)(60000000UL / (avg_period * FAN_TACH_PULSES_PER_REV));
  return _fan_last_rpm;
}

// Return the most recent RPM without re-polling. Useful in the
// 100 ms task when you need the value but don't want to block.
inline uint16_t fan_lastRPM() {
  return _fan_last_rpm;
}

#endif // FAN_CONTROL_H
