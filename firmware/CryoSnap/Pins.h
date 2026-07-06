#ifndef PINS_H
#define PINS_H

/*
  Pins.h — board target selection.

  This is the one place that resolves *which* set of pin numbers the
  firmware uses. Two targets are supported:

    TARGET_REVA   First-spin production PCB (Arduino Nano).
                  Pin values are hard-coded here from the verified
                  schematic. See docs/20260411 - Firmware Flowchart
                  Rev A.md section 5 for the full table.

    TARGET_REVB   Next-spin production PCB (Arduino Nano). Drops
                  the TPS_FAULT discrete input (TPS chip handles
                  catastrophic shutdowns and the firmware polls
                  TPS STATUS over I2C anyway). Moves the fan tach
                  to D5 / Timer1's T1 counter input so a hardware
                  counter path can replace the polled edge timing.

  Throughout the firmware, use the HW_* names below — never the raw
  pin numbers — so a pin move only requires editing this file.
*/

#define TARGET_REVA   2
#define TARGET_REVB   3

#ifndef BUILD_TARGET
  #define BUILD_TARGET TARGET_REVB
#endif

#include "Config.h"

#if BUILD_TARGET == TARGET_REVA
  // -- Production Rev A PCB -----------------------------------------------
  // Schematic-verified pinout. ATmega328P / Arduino Nano Classic.
  #define HW_FAN_TACH       2     // INT0  — fan tach ISR
  #define HW_FAN_PWM        3     // OC2B  — Timer2 hardware PWM
  #define HW_LED_DAT        5     // WS2812B chain
  #define HW_HB_DIR         6     // DRV8701E direction (PH/EN)
  #define HW_BUTTON_EN      8     // active low, internal pullup
  #define HW_NTC_1          A1
  #define HW_NTC_2          A2
  #define HW_NTC_3          A3
  #define HW_MODE_SWITCH    A6
  #define HW_POT            A7
  #define HW_TPS_FAULT      A0    // TPS55288 PG/Fault — polled digital read
  #define HW_INA_ALERT      10    // INA226 Alert — polled (optional ISR future)
  #define HAS_INA226        1
  #define HAS_HUSB238       1
  #define HAS_TPS_FAULT     1
  #define HAS_OLED          1
  #define I2C_ADDR_TPS55288 0x74  // strap-selected on Rev A
  // Seebeck polarity-flip mitigation recipe (see Config.h). Rev A keeps
  // the legacy OE-off measured-wait: the Rev A cap/bridge topology
  // tolerates the brief dead-rail dwell (validated on the Rev A rig).
  #ifndef SEEBECK_POWERED_FLIP
  #define SEEBECK_POWERED_FLIP 0
  #endif

#elif BUILD_TARGET == TARGET_REVB
  // -- Production Rev B PCB -----------------------------------------------
  // Schematic-verified pinout. ATmega328P / Arduino Nano Classic.
  // Differences vs Rev A:
  //   - INA226 Alert moves to D2 (was D10) — frees the SPI group.
  //   - Fan tach moves to D5 (was D2) — Timer1 T1 input lets a
  //     hardware counter replace the polled edge timing.
  //   - LED data moves to D4 (was D5, now used by tach).
  //   - Button Enable moves to D7 (was D8).
  //   - TPS_FAULT discrete input is removed entirely; the firmware's
  //     polled TPS STATUS read in task_100ms covers it.
  //   - AREF tied to the 3.3 V rail (op-amp NTC network reference;
  //     see Config.h USE_EXTERNAL_AREF). Operator-verified
  //     AREF = 3.3 V on the 2026-06-10 Rev B bring-up — NTC1 read
  //     room temperature (23.3 C) which is only consistent with a
  //     3.3 V reference; a 5 V AREF would have produced ~50% off
  //     numbers via the firmware's linear NTC formula.
  #define HW_FAN_TACH       5     // T1 input — Timer1 hardware counter
  #define HW_FAN_PWM        3     // OC2B  — Timer2 hardware PWM
  #define HW_LED_DAT        4     // WS2812B chain
  #define HW_HB_DIR         6     // DRV8701E direction (PH/EN)
  #define HW_BUTTON_EN      7     // active low, internal pullup
  #define HW_NTC_1          A1
  #define HW_NTC_2          A2
  #define HW_NTC_3          A3
  #define HW_MODE_SWITCH    A6
  #define HW_POT            A7
  #define HW_INA_ALERT      2     // INT0  — INA226 Alert (polled; ISR-ready)
  #define HAS_INA226        1
  #define HAS_HUSB238       1
  #define HAS_TPS_FAULT     0     // no discrete fault line on Rev B
  #define HAS_OLED          1
  #define I2C_ADDR_TPS55288 0x74  // strap-selected on Rev B (same as Rev A)
  // Seebeck polarity-flip mitigation recipe (see Config.h). Rev B uses
  // the LOW-V powered flip: an OE-off dwell drives the reversed Seebeck
  // EMF below ground and wedges the TPS off the I2C bus (bench: 54/54
  // kills), so the converter must stay POWERED through the reversal.
  #ifndef SEEBECK_POWERED_FLIP
  #define SEEBECK_POWERED_FLIP 1
  #endif

#else
  #error "BUILD_TARGET must be TARGET_REVA or TARGET_REVB"
#endif

// Fallback for an unknown BUILD_TARGET (or a -D override): the legacy
// OE-off path is the safe default for a board whose flip behaviour
// has not been characterised.
#ifndef SEEBECK_POWERED_FLIP
#define SEEBECK_POWERED_FLIP 0
#endif

// Common I2C device addresses.
#define I2C_ADDR_INA226   0x40
#define I2C_ADDR_HUSB238  0x08
#define I2C_ADDR_OLED     0x3C   // SSD1306 (alt 0x3D — strap-selected)

#endif // PINS_H
