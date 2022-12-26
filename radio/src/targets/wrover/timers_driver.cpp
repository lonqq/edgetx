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
#include "FreeRTOS_entry.h"

static hw_timer_t *MyTim5ms = NULL;
static hw_timer_t *MyTim2Mhz = NULL;
static SemaphoreHandle_t sem5ms;

static void IRAM_ATTR interrupt5ms()
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sem5ms, &pxHigherPriorityTaskWoken);
}

static void task5ms(void * pdata) {
  static uint8_t pre_scale;       // Used to get 10 Hz counter

  while (true) {
    if (xSemaphoreTake(sem5ms, portMAX_DELAY)) {
#if defined(COLORLCD)
      lv_tick_inc(5);
#endif

      ++pre_scale;

      if (pre_scale == 2) {
        pre_scale = 0;
        per10ms();
      }
    }
  }
}

// Start TIMER at 2000000Hz
void init2MhzTimer()
{
  MyTim2Mhz = timerBegin(1, 40, true); // 2MHz
  timerStart(MyTim2Mhz);
}

uint16_t getTmr2MHz() {
  return timerRead(MyTim2Mhz);
}

tmr10ms_t get_tmr10ms() {
  return (tmr10ms_t)(timerRead(MyTim2Mhz) / 20000); // 2MHz => 100Hz
}

#define TIM5MS_STACK_SIZE (1024 * 2)
RTOS_TASK_HANDLE taskId5ms;
RTOS_DEFINE_STACK(taskId5ms, task5ms_stack, TIM5MS_STACK_SIZE);
// Start TIMER at 200Hz
void init5msTimer()
{
  sem5ms = xSemaphoreCreateBinary();

  RTOS_CREATE_TASK_EX(taskId5ms,task5ms,"5ms timer",task5ms_stack,TIM5MS_STACK_SIZE,5,TMR_5MS_CORE);  // TODO-feather priority
  MyTim5ms = timerBegin(0, 80, true);
  timerAttachInterrupt(MyTim5ms, &interrupt5ms, true);
  timerAlarmWrite(MyTim5ms, 5000, true); // 200Hz
  timerAlarmEnable(MyTim5ms);
}

void stop5msTimer()
{
  timerAlarmDisable(MyTim5ms);
}

uint32_t ticksNow() {
   return xTaskGetTickCount();
}