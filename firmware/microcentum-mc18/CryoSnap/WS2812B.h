#ifndef WS2812B_H
#define WS2812B_H

#include <Arduino.h>
#include "Pins.h"

/*
  WS2812B.h — 23-LED chain driver.

  Bit-banged WS2812B output on HW_LED_DAT (D7 = PD7). Zero
  dependencies — no FastLED, no NeoPixel — so the project compiles
  in a stock Arduino IDE with nothing to install.

  The inner loop is timing-critical (~1.25 us per bit) and runs with
  global interrupts disabled for the duration of one frame. Worst
  case at 23 LEDs: 23 * 24 bits * 1.25 us = ~690 us. This is well
  under the fan tach period (10 ms at 6000 RPM), so no tach edges
  are missed.

  WS2812B protocol (at 16 MHz, 1 cycle = 62.5 ns):
    T0H = 350 ns  ~6 cycles    (data bit = 0, line HIGH)
    T0L = 800 ns  ~13 cycles   (data bit = 0, line LOW)
    T1H = 700 ns  ~11 cycles   (data bit = 1, line HIGH)
    T1L = 600 ns  ~10 cycles   (data bit = 1, line LOW)
    RES > 50 us                (latch / reset pulse)

  Bit order: GRB, MSB first. Each LED eats 24 bits then passes
  remaining data downstream.

  The implementation uses direct port manipulation on PORTD bit 7
  and inline assembly to count cycles precisely. The approach is
  adapted from the public-domain light_ws2812 library by cpldcpu.

  LED layout (spec section 24):
    1..10   NTC_1 temperature bar
    11..20  setpoint bar
    21      mode indicator
    22      enable indicator
    23      H-bridge state indicator

  Public API:
    leds_init()           configure pin as output, set LOW
    leds_show(colors)     push a frame of LED_COUNT colors
    leds_fill(color)      fill all LEDs with a single color
    leds_clear()          turn all LEDs off (shortcut for fill black)
*/

#define LED_COUNT 23

struct LedColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// ---- port mapping for HW_LED_DAT --------------------------------------
// D0..D7 live on PORTD; the bit number equals the digital pin number.
// Both supported targets put HW_LED_DAT on PORTD (proto = D7,
// Rev A = D5), so we hard-code PORTD/DDRD and derive the bit from
// HW_LED_DAT. If a future board moves the LED pin to PORTB (D8..D13)
// or PORTC (A0..A5), add the matching #if branch below.
#if HW_LED_DAT >= 0 && HW_LED_DAT <= 7
  #define WS_PORT  PORTD
  #define WS_DDR   DDRD
  #define WS_BIT   HW_LED_DAT
#else
  #error "HW_LED_DAT must be on PORTD (D0..D7) — update WS2812B.h to support other ports"
#endif
#define WS_HI    (WS_PORT |=  (1 << WS_BIT))
#define WS_LO    (WS_PORT &= ~(1 << WS_BIT))

// ---- private: send one byte (8 bits, MSB first) -------------------------
//
// Each bit is a fixed-width pulse on the data line:
//   0-bit: HIGH for ~6 cycles, LOW for ~13 cycles
//   1-bit: HIGH for ~11 cycles, LOW for ~10 cycles
//
// The inline asm uses nop sled + sbi/cbi on the port register. The
// cycle counts include the loop overhead (sbrc/rjmp/lsl) so the
// total period per bit stays close to 1.25 us.
//
// This runs at 16 MHz only. If the clock is different (e.g. 8 MHz
// pro-mini) the nop counts need adjusting.

static inline void _ws_send_byte(uint8_t b) __attribute__((always_inline));
static inline void _ws_send_byte(uint8_t b) {
  volatile uint8_t bit_count = 8;
  // Use direct port register address for sbi/cbi in asm.
  // PORTD is at I/O address 0x0B.
  asm volatile (
    "ws_bit_loop_%=:              \n\t"  // --- start of one bit ---
    "    sbi  %[port], %[bit]     \n\t"  // 2c  line HIGH
    "    sbrc %[byte], 7          \n\t"  // 1/2c skip next if bit7=0
    "    rjmp ws_one_%=           \n\t"  // 2c   branch to 1-bit path
    // --- 0-bit: HIGH held ~6 cycles total (sbi=2 + sbrc=1 + nop*1=1 + cbi=2 = 6)
    "    nop                      \n\t"  // 1c
    "    cbi  %[port], %[bit]     \n\t"  // 2c  line LOW
    // LOW for ~13 cycles: we need ~7 nops + lsl(1) + dec(1) + brne(2) = ~11
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    nop                      \n\t"
    "    rjmp ws_next_%=          \n\t"  // 2c  -> common tail
    // --- 1-bit: HIGH held ~11 cycles total (sbi=2 + sbrc=2 + rjmp=2 + nops=3 + cbi=2 = 11)
    "ws_one_%=:                   \n\t"
    "    nop                      \n\t"  // 1c
    "    nop                      \n\t"  // 1c
    "    nop                      \n\t"  // 1c
    "    cbi  %[port], %[bit]     \n\t"  // 2c  line LOW
    // LOW for ~10 cycles: nops + lsl + dec + brne
    "    nop                      \n\t"
    "    nop                      \n\t"
    // --- common tail: shift to next bit, loop ---
    "ws_next_%=:                  \n\t"
    "    lsl  %[byte]             \n\t"  // 1c  shift left, next bit into bit7
    "    dec  %[count]            \n\t"  // 1c
    "    brne ws_bit_loop_%=      \n\t"  // 2c (taken) / 1c (fall through)
    : [byte]  "+r" (b),
      [count] "+r" (bit_count)
    : [port]  "I"  (_SFR_IO_ADDR(WS_PORT)),
      [bit]   "I"  (WS_BIT)
  );
}

// ---- public API ---------------------------------------------------------

inline void leds_init() {
  WS_DDR |= (1 << WS_BIT);   // output
  WS_LO;                      // idle LOW
}

// Push a frame of LED_COUNT colors out the data line, scaled by an
// 8-bit brightness factor (0 = off, 255 = full). The scale is applied
// per byte: out = (in * brightness) >> 8. This is cheap (mul + shift,
// ~5 cycles per byte) and runs BEFORE the cli() block, so it doesn't
// add to the worst-case interrupts-off window.
//
// Blocks for ~690 us with interrupts disabled.
inline void leds_show(const LedColor* colors, uint8_t brightness = 255) {
  // Pre-scale into a small stack buffer (3 bytes per LED, GRB order).
  // 23 LEDs * 3 = 69 bytes — well within Arduino Nano stack budget.
  uint8_t scaled[LED_COUNT * 3];
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    scaled[i * 3 + 0] = ((uint16_t)colors[i].g * brightness) >> 8;
    scaled[i * 3 + 1] = ((uint16_t)colors[i].r * brightness) >> 8;
    scaled[i * 3 + 2] = ((uint16_t)colors[i].b * brightness) >> 8;
  }

  uint8_t sreg = SREG;
  cli();  // interrupts off for the entire frame

  for (uint16_t i = 0; i < LED_COUNT * 3; ++i) {
    _ws_send_byte(scaled[i]);
  }

  SREG = sreg;  // restore interrupt state

  // Latch: hold LOW for > 50 us. The time spent returning from this
  // function + the caller's next Serial.print or analogRead will
  // easily exceed 50 us, but be explicit just in case.
  delayMicroseconds(60);
}

// Set every LED to the same color.
inline void leds_fill(LedColor c) {
  LedColor frame[LED_COUNT];
  for (uint8_t i = 0; i < LED_COUNT; ++i) frame[i] = c;
  leds_show(frame);
}

// Turn all LEDs off.
inline void leds_clear() {
  leds_fill({0, 0, 0});
}

#endif // WS2812B_H
