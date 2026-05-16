#ifndef LED_RENDER_H
#define LED_RENDER_H

#include <Arduino.h>
#include "Config.h"
#include "WS2812B.h"

/*
  LedRender.h — build and push the WS2812 status frame.

  Called from task_100ms() once per control tick. Reads firmware
  state directly (g_fault, g_setpoint, g_mode, g_enabled,
  g_led_brightness) and takes the rest as parameters so the caller
  doesn't have to publish locals as globals.

  LED layout (23 pixels in chain order):
    0..9    Temperature bar (cold-side NTC), blue→red gradient,
            range = POT_TEMP_MIN..POT_TEMP_MAX so it lines up with
            the setpoint bar at the same column index.
    10..19  Setpoint bar, colour follows mode (red/green/blue),
            range = POT_TEMP_MIN..POT_TEMP_MAX.
    20      Mode indicator (red=Heat, green=Auto, blue=Cool).
    21      Enable indicator (purple=enabled, off=disabled).
    22      H-bridge state (red=heating, blue=cooling, off=off).

  When a fault is latched, all 23 LEDs slow-flash red at 1 Hz
  per the spec and the normal mapping is skipped.

  Customisation hints:
    - To change a bar's range, edit POT_TEMP_MIN / POT_TEMP_MAX in
      Config.h. Both bars use the same range so they stay aligned.
    - To remove the fault flash, set ENABLE_FAULT_LED_FLASH = 0.
    - To drop the fade-on-edge LED smoothing, set ENABLE_LED_FADE
      = 0 (saves a few hundred bytes of flash on a tight build).
    - To repurpose any LED, edit the indicator block at the bottom
      of leds_render() — each indicator is just a colour assigned
      to one frame[] slot.
*/

inline void leds_render(float t_cold, bool tec_on, uint8_t drive_dir) {
  LedColor frame[LED_COUNT];

  // Fault override: slow-flash red at 1 Hz (500 ms on, 500 ms off).
#if ENABLE_FAULT_LED_FLASH
  if (g_fault != FAULT_NONE) {
    bool on = (millis() / 500) & 1;
    LedColor c = on ? (LedColor){ 255, 0, 0 } : (LedColor){ 0, 0, 0 };
    for (uint8_t i = 0; i < LED_COUNT; ++i) frame[i] = c;
    leds_show(frame, g_led_brightness);
    return;
  }
#endif

  // ---- Temperature bar (LEDs 0..9) -----------------------------------
  // Blue (cold) → red (hot) gradient. Last LED fades proportionally
  // to the fractional part of the mapped position so the bar doesn't
  // flicker when the reading oscillates between two integer counts.
  //
  // Example: t_cold = 14.5 C with the default -10..+40 range:
  //   position = (14.5 - (-10)) * 10 / 50 = 4.9
  //   LEDs 0-3: full brightness
  //   LED 4:    90% brightness (fractional part)
  //   LEDs 5-9: off
  {
    const float t_lo   = (float)POT_TEMP_MIN;
    const float t_span = (float)(POT_TEMP_MAX - POT_TEMP_MIN);
    float temp_pos = 0;
    if (!isnan(t_cold)) {
      temp_pos = (constrain(t_cold, t_lo, t_lo + t_span) - t_lo)
                 * 10.0f / t_span;
    }
    uint8_t full_leds = (uint8_t)temp_pos;
#if ENABLE_LED_FADE
    uint8_t fade = (uint8_t)((temp_pos - full_leds) * 255);
#endif

    for (uint8_t i = 0; i < 10; ++i) {
      uint8_t r_full = (uint8_t)((uint16_t)i * 255 / 9);
      uint8_t b_full = 255 - r_full;

      if (i < full_leds) {
        frame[i] = { r_full, 0, b_full };
      }
#if ENABLE_LED_FADE
      else if (i == full_leds && fade > 0) {
        frame[i] = { (uint8_t)(r_full * fade / 255),
                     0,
                     (uint8_t)(b_full * fade / 255) };
      }
#endif
      else {
        frame[i] = { 0, 0, 0 };
      }
    }
  }

  // ---- Setpoint bar (LEDs 10..19) ------------------------------------
  // Uses the same POT_TEMP_MIN..POT_TEMP_MAX range as the temp bar
  // so columns line up visually. The full pot rotation lights all
  // ten LEDs end-to-end.
  {
    const float sp_lo   = (float)POT_TEMP_MIN;
    const float sp_span = (float)(POT_TEMP_MAX - POT_TEMP_MIN);
    float sp_pos = (constrain(g_setpoint, sp_lo, sp_lo + sp_span) - sp_lo)
                   * 10.0f / sp_span;
    uint8_t full_leds = (uint8_t)sp_pos;
#if ENABLE_LED_FADE
    uint8_t fade = (uint8_t)((sp_pos - full_leds) * 255);
#endif

    LedColor sp_color;
    if      (g_mode == MODE_HEAT) sp_color = { 255, 0, 0 };
    else if (g_mode == MODE_COOL) sp_color = { 0, 0, 255 };
    else                          sp_color = { 0, 255, 0 };

    for (uint8_t i = 10; i < 20; ++i) {
      uint8_t idx = i - 10;
      if (idx < full_leds) {
        frame[i] = sp_color;
      }
#if ENABLE_LED_FADE
      else if (idx == full_leds && fade > 0) {
        frame[i] = { (uint8_t)(sp_color.r * fade / 255),
                     (uint8_t)(sp_color.g * fade / 255),
                     (uint8_t)(sp_color.b * fade / 255) };
      }
#endif
      else {
        frame[i] = { 0, 0, 0 };
      }
    }
  }

  // ---- Indicators (LEDs 20..22) --------------------------------------
  // LED 20: mode (red=Heat, green=Auto, blue=Cool).
  if      (g_mode == MODE_HEAT) frame[20] = { 255, 0, 0 };
  else if (g_mode == MODE_AUTO) frame[20] = { 0, 255, 0 };
  else                          frame[20] = { 0, 0, 255 };

  // LED 21: enable indicator (purple=enabled, off=disabled).
  frame[21] = g_enabled ? (LedColor){ 128, 0, 128 } : (LedColor){ 0, 0, 0 };

  // LED 22: H-bridge state (red=heating, blue=cooling, off=off).
  if (!tec_on)                    frame[22] = { 0, 0, 0 };
  else if (drive_dir == HB_HEAT)  frame[22] = { 255, 0, 0 };
  else                            frame[22] = { 0, 0, 255 };

  leds_show(frame, g_led_brightness);
}

#endif // LED_RENDER_H
