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
#include "RTClib.h"

RTC_DS3231 rtc;

void rtcSetTime(const struct gtm * t)
{
    DateTime now = DateTime(t->tm_year + TM_YEAR_BASE, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    Serial.print("rtcSetTime ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println(); 
    rtc.adjust(now);
}

void rtcGetTime(struct gtm * t)
{
    DateTime now = rtc.now();
    Serial.print("rtcGetTime ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    t->tm_year = now.year() - TM_YEAR_BASE;
    t->tm_mon = now.month() - 1;
    t->tm_mday = now.day();
    t->tm_hour = now.hour();
    t->tm_min = now.minute();
    t->tm_sec = now.second();
}

void rtcInit()
{
    rtc.begin();
    struct gtm utm;
    rtcGetTime(&utm);
    g_rtcTime = gmktime(&utm);
}
