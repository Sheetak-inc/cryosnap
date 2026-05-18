#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "Pins.h"
#include "Config.h"

/*
  OLED.h — SSD1306 128x64 I2C OLED, text-only driver.

  Why a custom driver: Adafruit_SSD1306 + Adafruit_GFX together pull
  in ~5 KB flash and a 1 KB RAM framebuffer (half of the Nano's RAM).
  The dev kit only needs a status readout — no graphics — so this
  driver writes characters straight to the SSD1306 page registers
  with NO local framebuffer. ~150 B RAM cost for the whole subsystem.

  Trade-off: you can't do random-access pixel writes, only character
  cells aligned to 8-pixel page rows. Each row holds 21 characters
  of 5x7 font (5 pixel cells + 1 pad column = 6 px/char, 128/6 = 21).

  Wiring: I2C @ 0x3C (or 0x3D — strap-selected, see datasheet).
  Same SDA/SCL as the rest of the I2C bus — already initialized by
  Wire.begin() in setup().

  Public API:
    oled_init()                 probe, init, clear; returns true on ACK
    oled_present()              true if oled_init succeeded
    oled_clear()                blank the entire screen
    oled_setCursor(col, row)    page row 0..7, pixel column 0..127
    oled_print(c)               single character
    oled_print(str)             zero-terminated C string (RAM)
    oled_print_P(F("..."))      F() / PSTR string from flash
    oled_print(int)             signed integer
    oled_print(uint)            unsigned integer
    oled_print(float, decimals) float with N decimals (max 3)
    oled_clearRow(row)          blank one row (faster than full clear)
*/

#if HAS_OLED && ENABLE_OLED_DISPLAY

#ifndef I2C_ADDR_OLED
#define I2C_ADDR_OLED 0x3C
#endif

// SSD1306 register / command set — only the subset we need.
#define SSD_CMD               0x00   // control byte: command stream
#define SSD_DATA              0x40   // control byte: data stream
#define SSD_DISPLAY_OFF       0xAE
#define SSD_DISPLAY_ON        0xAF
#define SSD_NORMAL_DISPLAY    0xA6
#define SSD_SET_CONTRAST      0x81
#define SSD_SET_MEMORY_MODE   0x20   // arg: 0=horiz, 1=vert, 2=page
#define SSD_SET_COLUMN_ADDR   0x21   // args: start, end
#define SSD_SET_PAGE_ADDR     0x22   // args: start, end
#define SSD_SET_START_LINE    0x40
#define SSD_SEG_REMAP         0xA0   // |0x01 -> mirror columns
#define SSD_COM_SCAN_DEC      0xC8
#define SSD_COM_PIN_CFG       0xDA
#define SSD_SET_CLOCK_DIV     0xD5
#define SSD_SET_MULTIPLEX     0xA8
#define SSD_SET_DISPLAY_OFFSET 0xD3
#define SSD_CHARGE_PUMP       0x8D
#define SSD_PRECHARGE         0xD9
#define SSD_VCOMH             0xDB

// Geometry. OLED_HEIGHT comes from Config.h (default 32; override
// to 64 if a taller module is fitted). The init sequence and page
// count adapt below.
#define OLED_WIDTH   128
#define OLED_PAGES   (OLED_HEIGHT / 8)
#define OLED_COLS_PER_CHAR 6   // 5 px font + 1 px gap

#if OLED_HEIGHT == 32
  #define _OLED_INIT_MUX     0x1F   // 32-line multiplex
  #define _OLED_INIT_COMPIN  0x02   // sequential, no remap (128x32)
#elif OLED_HEIGHT == 64
  #define _OLED_INIT_MUX     0x3F   // 64-line multiplex
  #define _OLED_INIT_COMPIN  0x12   // alternative + no remap (128x64)
#else
  #error "OLED_HEIGHT must be 32 or 64"
#endif

// =====================================================================
// 5x7 font, ASCII 32..126.
//
// Each glyph is 5 column bytes; the LSB of each byte is the top pixel.
// 95 glyphs * 5 bytes = 475 B in PROGMEM.
//
// Public-domain font derived from the cp437 8x8 set, trimmed to the
// classic 5x7 cell used by HD44780-style LCDs and many 128x64 OLEDs.
// =====================================================================
static const uint8_t _oled_font5x7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00, // ' '
  0x00,0x00,0x5F,0x00,0x00, // '!'
  0x00,0x07,0x00,0x07,0x00, // '"'
  0x14,0x7F,0x14,0x7F,0x14, // '#'
  0x24,0x2A,0x7F,0x2A,0x12, // '$'
  0x23,0x13,0x08,0x64,0x62, // '%'
  0x36,0x49,0x55,0x22,0x50, // '&'
  0x00,0x05,0x03,0x00,0x00, // '\''
  0x00,0x1C,0x22,0x41,0x00, // '('
  0x00,0x41,0x22,0x1C,0x00, // ')'
  0x14,0x08,0x3E,0x08,0x14, // '*'
  0x08,0x08,0x3E,0x08,0x08, // '+'
  0x00,0x50,0x30,0x00,0x00, // ','
  0x08,0x08,0x08,0x08,0x08, // '-'
  0x00,0x60,0x60,0x00,0x00, // '.'
  0x20,0x10,0x08,0x04,0x02, // '/'
  0x3E,0x51,0x49,0x45,0x3E, // '0'
  0x00,0x42,0x7F,0x40,0x00, // '1'
  0x42,0x61,0x51,0x49,0x46, // '2'
  0x21,0x41,0x45,0x4B,0x31, // '3'
  0x18,0x14,0x12,0x7F,0x10, // '4'
  0x27,0x45,0x45,0x45,0x39, // '5'
  0x3C,0x4A,0x49,0x49,0x30, // '6'
  0x01,0x71,0x09,0x05,0x03, // '7'
  0x36,0x49,0x49,0x49,0x36, // '8'
  0x06,0x49,0x49,0x29,0x1E, // '9'
  0x00,0x36,0x36,0x00,0x00, // ':'
  0x00,0x56,0x36,0x00,0x00, // ';'
  0x08,0x14,0x22,0x41,0x00, // '<'
  0x14,0x14,0x14,0x14,0x14, // '='
  0x00,0x41,0x22,0x14,0x08, // '>'
  0x02,0x01,0x51,0x09,0x06, // '?'
  0x32,0x49,0x79,0x41,0x3E, // '@'
  0x7E,0x11,0x11,0x11,0x7E, // 'A'
  0x7F,0x49,0x49,0x49,0x36, // 'B'
  0x3E,0x41,0x41,0x41,0x22, // 'C'
  0x7F,0x41,0x41,0x22,0x1C, // 'D'
  0x7F,0x49,0x49,0x49,0x41, // 'E'
  0x7F,0x09,0x09,0x09,0x01, // 'F'
  0x3E,0x41,0x49,0x49,0x7A, // 'G'
  0x7F,0x08,0x08,0x08,0x7F, // 'H'
  0x00,0x41,0x7F,0x41,0x00, // 'I'
  0x20,0x40,0x41,0x3F,0x01, // 'J'
  0x7F,0x08,0x14,0x22,0x41, // 'K'
  0x7F,0x40,0x40,0x40,0x40, // 'L'
  0x7F,0x02,0x0C,0x02,0x7F, // 'M'
  0x7F,0x04,0x08,0x10,0x7F, // 'N'
  0x3E,0x41,0x41,0x41,0x3E, // 'O'
  0x7F,0x09,0x09,0x09,0x06, // 'P'
  0x3E,0x41,0x51,0x21,0x5E, // 'Q'
  0x7F,0x09,0x19,0x29,0x46, // 'R'
  0x46,0x49,0x49,0x49,0x31, // 'S'
  0x01,0x01,0x7F,0x01,0x01, // 'T'
  0x3F,0x40,0x40,0x40,0x3F, // 'U'
  0x1F,0x20,0x40,0x20,0x1F, // 'V'
  0x3F,0x40,0x38,0x40,0x3F, // 'W'
  0x63,0x14,0x08,0x14,0x63, // 'X'
  0x07,0x08,0x70,0x08,0x07, // 'Y'
  0x61,0x51,0x49,0x45,0x43, // 'Z'
  0x00,0x7F,0x41,0x41,0x00, // '['
  0x02,0x04,0x08,0x10,0x20, // '\'
  0x00,0x41,0x41,0x7F,0x00, // ']'
  0x04,0x02,0x01,0x02,0x04, // '^'
  0x40,0x40,0x40,0x40,0x40, // '_'
  0x00,0x01,0x02,0x04,0x00, // '`'
  0x20,0x54,0x54,0x54,0x78, // 'a'
  0x7F,0x48,0x44,0x44,0x38, // 'b'
  0x38,0x44,0x44,0x44,0x20, // 'c'
  0x38,0x44,0x44,0x48,0x7F, // 'd'
  0x38,0x54,0x54,0x54,0x18, // 'e'
  0x08,0x7E,0x09,0x01,0x02, // 'f'
  0x0C,0x52,0x52,0x52,0x3E, // 'g'
  0x7F,0x08,0x04,0x04,0x78, // 'h'
  0x00,0x44,0x7D,0x40,0x00, // 'i'
  0x20,0x40,0x44,0x3D,0x00, // 'j'
  0x7F,0x10,0x28,0x44,0x00, // 'k'
  0x00,0x41,0x7F,0x40,0x00, // 'l'
  0x7C,0x04,0x18,0x04,0x78, // 'm'
  0x7C,0x08,0x04,0x04,0x78, // 'n'
  0x38,0x44,0x44,0x44,0x38, // 'o'
  0x7C,0x14,0x14,0x14,0x08, // 'p'
  0x08,0x14,0x14,0x18,0x7C, // 'q'
  0x7C,0x08,0x04,0x04,0x08, // 'r'
  0x48,0x54,0x54,0x54,0x20, // 's'
  0x04,0x3F,0x44,0x40,0x20, // 't'
  0x3C,0x40,0x40,0x20,0x7C, // 'u'
  0x1C,0x20,0x40,0x20,0x1C, // 'v'
  0x3C,0x40,0x30,0x40,0x3C, // 'w'
  0x44,0x28,0x10,0x28,0x44, // 'x'
  0x0C,0x50,0x50,0x50,0x3C, // 'y'
  0x44,0x64,0x54,0x4C,0x44, // 'z'
  0x00,0x08,0x36,0x41,0x00, // '{'
  0x00,0x00,0x7F,0x00,0x00, // '|'
  0x00,0x41,0x36,0x08,0x00, // '}'
  0x10,0x08,0x10,0x20,0x10, // '~'
};

// ---- driver state -----------------------------------------------------
static bool _oled_present = false;

// ---- low-level I2C helpers --------------------------------------------

// Send a single command byte. Used only by oled_init() — runtime
// drawing batches commands via _oled_cmd2 / inline command lists.
static inline void _oled_cmd1(uint8_t c) {
  Wire.beginTransmission(I2C_ADDR_OLED);
  Wire.write(SSD_CMD);
  Wire.write(c);
  Wire.endTransmission();
}

// Send a command byte + one argument byte in a single I2C transaction.
static inline void _oled_cmd2(uint8_t c, uint8_t a) {
  Wire.beginTransmission(I2C_ADDR_OLED);
  Wire.write(SSD_CMD);
  Wire.write(c);
  Wire.write(a);
  Wire.endTransmission();
}

// Begin a data write — caller follows with Wire.write() bytes and
// must call Wire.endTransmission() when done.
//
// IMPORTANT: keep the per-transaction byte count under
// (BUFFER_LENGTH - 1) so the Wire library's TX buffer doesn't drop
// data. On stock Arduino BUFFER_LENGTH = 32, so up to 31 data bytes
// per call (we write columns in batches of 30 = 5 chars).
static inline void _oled_data_begin() {
  Wire.beginTransmission(I2C_ADDR_OLED);
  Wire.write(SSD_DATA);
}

// Forward declarations — oled_init() needs to call oled_clear(),
// which depends on oled_clearRow() / oled_setCursor() defined below.
inline void oled_clear();
inline void oled_clearRow(uint8_t row);
inline void oled_setCursor(uint8_t col, uint8_t row);

// ---- public API -------------------------------------------------------

inline bool oled_present() { return _oled_present; }

inline bool oled_init() {
  // ACK probe at 0x3C.
  Wire.beginTransmission(I2C_ADDR_OLED);
  if (Wire.endTransmission() != 0) {
    _oled_present = false;
    return false;
  }

  // Standard 128x64 SSD1306 init sequence (from the datasheet
  // application note). Order matters — the charge pump must be
  // enabled before the display is turned on, or the panel stays dark.
  _oled_cmd1(SSD_DISPLAY_OFF);
  _oled_cmd2(SSD_SET_CLOCK_DIV, 0x80);
  _oled_cmd2(SSD_SET_MULTIPLEX, _OLED_INIT_MUX);
  _oled_cmd2(SSD_SET_DISPLAY_OFFSET, 0x00);
  _oled_cmd1(SSD_SET_START_LINE | 0x00);
  _oled_cmd2(SSD_CHARGE_PUMP, 0x14);       // 0x14 = enable internal pump
  _oled_cmd2(SSD_SET_MEMORY_MODE, 0x00);   // horizontal addressing
  _oled_cmd1(SSD_SEG_REMAP | 0x01);
  _oled_cmd1(SSD_COM_SCAN_DEC);
  _oled_cmd2(SSD_COM_PIN_CFG, _OLED_INIT_COMPIN);
  _oled_cmd2(SSD_SET_CONTRAST, 0xCF);
  _oled_cmd2(SSD_PRECHARGE, 0xF1);
  _oled_cmd2(SSD_VCOMH, 0x40);
  _oled_cmd1(0xA4);                        // resume from RAM
  _oled_cmd1(SSD_NORMAL_DISPLAY);
  _oled_cmd1(SSD_DISPLAY_ON);

  _oled_present = true;
  oled_clear();
  return true;
}

// Move the write cursor to (col, row). row is 0..7 (page), col is
// 0..127 (pixel).  Subsequent oled_print() writes start here.
inline void oled_setCursor(uint8_t col, uint8_t row) {
  if (!_oled_present) return;
  if (row >= OLED_PAGES) row = OLED_PAGES - 1;
  if (col >= OLED_WIDTH) col = OLED_WIDTH - 1;

  // Set page range to a single row, column range to col..127.
  Wire.beginTransmission(I2C_ADDR_OLED);
  Wire.write(SSD_CMD);
  Wire.write(SSD_SET_PAGE_ADDR);
  Wire.write(row);
  Wire.write(row);
  Wire.write(SSD_SET_COLUMN_ADDR);
  Wire.write(col);
  Wire.write(OLED_WIDTH - 1);
  Wire.endTransmission();
}

// Blank one row (page) — much faster than oled_clear() when only one
// status line needs refreshing.
inline void oled_clearRow(uint8_t row) {
  if (!_oled_present) return;
  oled_setCursor(0, row);
  // 128 zero bytes, sent in 4 chunks of 32 (one control byte + 31
  // payload bytes per chunk fits in the default 32-byte Wire buffer).
  for (uint8_t chunk = 0; chunk < 5; ++chunk) {
    _oled_data_begin();
    uint8_t n = (chunk == 4) ? (OLED_WIDTH - 4 * 31) : 31;
    for (uint8_t i = 0; i < n; ++i) Wire.write((uint8_t)0);
    Wire.endTransmission();
  }
}

inline void oled_clear() {
  if (!_oled_present) return;
  for (uint8_t row = 0; row < OLED_PAGES; ++row) oled_clearRow(row);
  oled_setCursor(0, 0);
}

// Write one character at the current cursor position. Falls back to
// the '?' glyph for codes outside the ASCII 32..126 range.
inline void oled_print(char c) {
  if (!_oled_present) return;

  uint8_t idx = (uint8_t)c;
  if (idx < 32 || idx > 126) idx = '?';
  idx -= 32;

  const uint8_t* g = _oled_font5x7 + (uint16_t)idx * 5;

  // 5 glyph columns + 1 trailing blank column = one character cell.
  _oled_data_begin();
  for (uint8_t i = 0; i < 5; ++i) Wire.write(pgm_read_byte(g + i));
  Wire.write((uint8_t)0);
  Wire.endTransmission();
}

inline void oled_print(const char* s) {
  if (!_oled_present || !s) return;
  while (*s) oled_print(*s++);
}

inline void oled_print_P(const __FlashStringHelper* s) {
  if (!_oled_present || !s) return;
  const char* p = reinterpret_cast<const char*>(s);
  while (true) {
    char c = (char)pgm_read_byte(p++);
    if (!c) break;
    oled_print(c);
  }
}

inline void oled_print(int v) {
  char buf[8];
  itoa(v, buf, 10);
  oled_print((const char*)buf);
}

inline void oled_print(unsigned int v) {
  char buf[8];
  utoa(v, buf, 10);
  oled_print((const char*)buf);
}

inline void oled_print(long v) {
  char buf[12];
  ltoa(v, buf, 10);
  oled_print((const char*)buf);
}

inline void oled_print(float v, uint8_t decimals = 1) {
  char buf[16];
  dtostrf(v, 0, decimals, buf);
  oled_print((const char*)buf);
}

#else  // !HAS_OLED || !ENABLE_OLED_DISPLAY — stubs so callers compile.

inline bool oled_init()                                { return true; }
inline bool oled_present()                             { return false; }
inline void oled_clear()                               {}
inline void oled_clearRow(uint8_t)                     {}
inline void oled_setCursor(uint8_t, uint8_t)           {}
inline void oled_print(char)                           {}
inline void oled_print(const char*)                    {}
inline void oled_print_P(const __FlashStringHelper*)   {}
inline void oled_print(int)                            {}
inline void oled_print(unsigned int)                   {}
inline void oled_print(long)                           {}
inline void oled_print(float, uint8_t = 1)             {}

#endif // HAS_OLED && ENABLE_OLED_DISPLAY

#endif // OLED_H
