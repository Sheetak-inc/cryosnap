#ifndef OLED_RENDER_H
#define OLED_RENDER_H

#include <Arduino.h>
#include "Config.h"
#include "Version.h"
#include "OLED.h"
#include "NTC.h"
#include "INA226.h"

/*
  OledRender.h — paint the SSD1306 status frame.

  Called from task_slow() once per OLED_REFRESH_TICKS slow-task
  ticks (1 s by default). Reads firmware state directly
  (g_enabled, g_mode, g_setpoint, g_fault, g_imax_mA) plus the
  NTC and INA226 driver reads.

  Layout — 4 compact rows that fit both 128x32 and 128x64 panels.
  (On a 128x64 the bottom 4 rows stay blank; redesign here if you
  want to use the extra real estate.) Each row is 21 chars wide
  with the bundled 5x7 font + 1 px gap = 6 px per char.

    row 0: "vX.Y.Z ON Cool"          version + enable + mode
    row 1: "Set:25.0 T:23.4C"        setpoint + cold-side temp
    row 2: "1.23A 12.0V 14.8W"       INA electrical summary
    row 3: "Flt:- H:30.1C"           fault + hot-side temp

  Customisation hints:
    - To rearrange rows: just reorder the oled_clearRow/setCursor
      blocks. clearRow() per row prevents stale chars from sticking
      when a value shrinks (e.g. "12.34" → "1.2").
    - To add a row on a 128x64 panel: bump OLED_HEIGHT in Config.h
      to 64 and append more oled_setCursor(0, 4..7) blocks.
    - To slow / speed the refresh: change OLED_REFRESH_TICKS in
      Config.h. 1 = each slow task (1 s).
    - To disable entirely: set ENABLE_OLED_DISPLAY = 0; this whole
      block compiles out and the stubs in OLED.h take over.
*/

#if HAS_OLED && ENABLE_OLED_DISPLAY

inline void oled_render() {
  static uint8_t _tick_count = 0;
  if (!oled_present() || ++_tick_count < OLED_REFRESH_TICKS) return;
  _tick_count = 0;

  // ---- Row 0: version + enable + mode -----------------------------------
  oled_clearRow(0);
  oled_setCursor(0, 0);
  oled_print('v');
  oled_print_P(F(FW_VERSION_STR));
  oled_print(' ');
  oled_print_P(g_enabled ? F("ON") : F("OFF"));
  oled_print(' ');
  oled_print_P(g_mode == MODE_COOL ? F("Cool") :
               g_mode == MODE_HEAT ? F("Heat") : F("Auto"));

  // ---- Row 1: setpoint + cold-side temp ---------------------------------
  oled_clearRow(1);
  oled_setCursor(0, 1);
  oled_print_P(F("Set:"));
  oled_print(g_setpoint, 1);
  oled_print_P(F(" T:"));
  oled_print(ntc_getC(1), 1);
  oled_print('C');

  // ---- Row 2: I / V / P from the INA226 ---------------------------------
#if HAS_INA226
  {
    float ina_i = ina_readCurrentA();
    float ina_v = ina_readVoltageV();
    float ina_p = ina_readPowerW();
    bool  clip  = ina_isClipped();

    oled_clearRow(2);
    oled_setCursor(0, 2);
    if (clip) oled_print('>');
    oled_print(ina_i, 2);
    oled_print_P(F("A "));
    oled_print(ina_v, 1);
    oled_print_P(F("V "));
    if (clip) oled_print('~');
    oled_print(clip ? ina_estimatePowerW(ina_v, (float)g_imax_mA / 1000.0f)
                    : ina_p, 1);
    oled_print('W');
  }
#endif

  // ---- Row 3: fault state + hot-side temp -------------------------------
  oled_clearRow(3);
  oled_setCursor(0, 3);
  oled_print_P(F("Flt:"));
  switch (g_fault) {
    case FAULT_NONE:      oled_print('-');             break;
    case FAULT_HOT_SIDE:  oled_print_P(F("OVERTEMP")); break;
    case FAULT_INA_ALERT: oled_print_P(F("INA"));      break;
    case FAULT_TPS_PG:    oled_print_P(F("TPS"));      break;
    case FAULT_HUSB_20V:  oled_print_P(F("PD"));       break;
    case FAULT_FAN_TACH:  oled_print_P(F("FAN"));      break;
    case FAULT_NO_SUPPLY: oled_print_P(F("NoPSU"));    break;
    default:              oled_print((int)g_fault);    break;
  }
  oled_print_P(F(" H:"));
  oled_print(ntc_getC(2), 1);
  oled_print('C');
}

#else  // !HAS_OLED || !ENABLE_OLED_DISPLAY — stub so callers compile
inline void oled_render() {}
#endif

#endif // OLED_RENDER_H
