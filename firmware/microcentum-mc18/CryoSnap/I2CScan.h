#ifndef I2C_SCAN_H
#define I2C_SCAN_H

#include <Arduino.h>
#include <Wire.h>
#include "Pins.h"

/*
  I2CScan.h — bus-presence helpers.

  i2cScan() walks every 7-bit address (0x03..0x77), does a zero-byte
  write transaction, and prints any address that ACKs. Use this at
  boot to confirm that every expected I2C device is wired and
  powered. Easy to comment out later in production firmware.

  i2cPing(addr) is a one-shot presence check for a single device.

  Both helpers assume Wire.begin() has already been called.
*/

// Print one line per discovered device, plus a summary, to the given
// stream (typically Serial). Returns the number of devices that ACKed.
//
// Each probe respects the per-transaction timeout set by
// Wire.setWireTimeout() in setup() — without that, a stuck SDA line
// would freeze the scan. With the 25 ms timeout, the worst case is
// ~3 s for a fully empty bus.
inline uint8_t i2cScan(Stream& out) {
  uint8_t found = 0;
  out.println(F("I2C scan starting..."));
  for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      out.print(F("  found 0x"));
      if (addr < 0x10) out.print('0');
      out.print(addr, HEX);
      // Annotate the addresses we know about so the maker can tell at
      // a glance whether the bus is correctly wired. Build-target
      // peripherals are guarded; OLED is annotated unconditionally
      // since the prototype's I2C 0.91" SSD1306 module is a fixture
      // even though it isn't part of the firmware scope.
      if (addr == I2C_ADDR_TPS55288)    out.print(F("  (TPS55288)"));
      else if (addr == 0x3C)            out.print(F("  (OLED SSD1306)"));
#if HAS_INA226
      else if (addr == I2C_ADDR_INA226) out.print(F("  (INA226)"));
#endif
#if HAS_HUSB238
      else if (addr == I2C_ADDR_HUSB238) out.print(F("  (HUSB238)"));
#endif
      out.println();
      ++found;
    }
  }
  out.print(F("I2C scan done: "));
  out.print(found);
  out.println(F(" device(s) responded."));
  return found;
}

// Single-shot presence check. Returns true if the device at addr ACKs.
inline bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

#endif // I2C_SCAN_H
