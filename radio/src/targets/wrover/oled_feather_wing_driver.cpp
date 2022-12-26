/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

extern pixel_t displayBuf[];

bool lcdInitFinished = false;
void lcdInitFinish();

#if !defined(LCD_DUAL_BUFFER)
void lcdRefreshWait()
{
}
#endif

void lcdRefresh(bool wait)
{
  display.clearDisplay();

  uint8_t * p = displayBuf;
  for (int y = 0; y < LCD_H; y+=8) {
    for (int x = 0; x < LCD_W; x+=1) {
      uint8_t bit = 1;
      for (int shift = 0; shift < 8; shift++) {
        display.drawPixel(x, y+shift, (0 != (*p & bit)) ? MONOOLED_WHITE : MONOOLED_BLACK);
        bit <<= 1;
      }
      p++;
    }
  }
  display.display();
}

/*
  Proper method for turning of LCD module. It must be used,
  otherwise we might damage LCD crystals in the long run!
*/
void lcdOff()
{
}

/*
  Starts LCD initialization routine. It should be called as
  soon as possible after the reset because LCD takes a lot of
  time to properly power-on.

  Make sure that delay_ms() is functional before calling this function!
*/
void lcdInit()
{
  delay(250); // wait for the OLED to power up
  display.begin(0x3C, true); // Address 0x3C default

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);
  display.setContrast(0x1);
}

/*
  Finishes LCD initialization. It is called auto-magically when first LCD command is
  issued by the other parts of the code.
*/

#if defined(RADIO_X9DP2019) || defined(RADIO_X7ACCESS)
  #define LCD_DELAY_NEEDED() true
#else
  #define LCD_DELAY_NEEDED() (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE())
#endif

void lcdInitFinish()
{
  lcdInitFinished = true;

  /*
    LCD needs longer time to initialize in low temperatures. The data-sheet
    mentions a time of at least 150 ms. The delay of 1300 ms was obtained
    experimentally. It was tested down to -10 deg Celsius.

    The longer initialization time seems to only be needed for regular Taranis,
    the Taranis Plus (9XE) has been tested to work without any problems at -18 deg Celsius.
    Therefore the delay for T+ is lower.

    If radio is reset by watchdog or boot-loader the wait is skipped, but the LCD
    is initialized in any case.

    This initialization is needed in case the user moved power switch to OFF and
    then immediately to ON position, because lcdOff() was called. In any case the LCD
    initialization (without reset) is also recommended by the data sheet.
  */
}

void lcdSetRefVolt(uint8_t val)
{
}

static bool bkl_enabled = false;
void backlightInit() {
  backlightEnable(BACKLIGHT_LEVEL_MAX);
}

void backlightEnable(uint8_t level) {
  static uint16_t prev_level = 0;
  const uint16_t offset = 200;
  uint16_t l = (((uint16_t)level) * (10 - offset) / BACKLIGHT_LEVEL_MAX) + offset;
  if (l != prev_level) {
    display.setContrast(l);
    prev_level = l;
  }
  bkl_enabled = true;
}

void backlightDisable() {
  display.setContrast(1);
  bkl_enabled = false;
}
uint8_t isBacklightEnabled() {return bkl_enabled;}
