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
#include "mixer_scheduler.h"
#include "Arduino.h"
#include "FreeRTOS_entry.h"

static hw_timer_t *mixer_timer = NULL;

static void IRAM_ATTR interrupt_mixer() {
  timerAlarmDisable(mixer_timer);
  size_t usec = 2 * getMixerSchedulerPeriod() - 1;
  timerAlarmWrite(mixer_timer, usec, true);

  mixerSchedulerISRTrigger();
}

// Start scheduler with default period
void mixerSchedulerStart()
{
  size_t usec = 2 * getMixerSchedulerPeriod() - 1;
  mixer_timer = timerBegin(2, 80, true);
  timerAttachInterrupt(mixer_timer, &interrupt_mixer, true);
  timerAlarmWrite(mixer_timer, usec, true);
}

void mixerSchedulerStop()
{
  timerAlarmDisable(mixer_timer);
}

void mixerSchedulerResetTimer()
{
  timerAlarmDisable(mixer_timer);
  timerWrite(mixer_timer, 0);
  timerAlarmEnable(mixer_timer);
}

void mixerSchedulerEnableTrigger()
{
  timerAlarmEnable(mixer_timer);
}

void mixerSchedulerDisableTrigger()
{
  timerAlarmDisable(mixer_timer);
}
