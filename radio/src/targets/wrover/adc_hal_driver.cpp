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
#include "hal/adc_driver.h"
#include "Adafruit_ADS1X15.h"

static uint16_t mux[] = {ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
static uint8_t currentAdcPin = 0;

#ifndef DISABLE_I2C_DEVS
static Adafruit_ADS1015 ads1015;
#endif

static bool arduino_hal_adc_init()
{
#ifndef DISABLE_I2C_DEVS
  ads1015.begin();
  ads1015.startADCReading(mux[currentAdcPin], false);
#endif
  // TODO-feather: give those channels a default value, in case the FLySky Hall Gimbals were not connected
  adcValues[0] = 500;
  adcValues[1] = 500;
  adcValues[2] = 500;
  adcValues[3] = 500;
  adcValues[4] = 500;
  adcValues[5] = 500;
  return true;
}

static bool arduino_hal_adc_start_read()
{
  return true;
}

static void arduino_hal_adc_wait_completion() {
}

static const etx_hal_adc_driver_t arduino_hal_adc_driver = {
  arduino_hal_adc_init,
  arduino_hal_adc_start_read,
  arduino_hal_adc_wait_completion
};

static void task_i2c(void * pdata) {
  while (1) {
#ifndef DISABLE_I2C_DEVS
    if (ads1015.conversionComplete()) {
      adcValues[4 + currentAdcPin] = ads1015.getLastConversionResults();
      if ((2 == currentAdcPin) && (adcValues[4 + currentAdcPin] < 256)) {
        adcValues[6] = (1<<11); // fake batter voltage if batter not connected, to avoid alarm sound during dev
      }
      //TRACE("+++++ADS1015 pin %d reading %d", currentAdcPin, adcValues[4 + currentAdcPin]);
      currentAdcPin++;
      if (currentAdcPin >= sizeof(mux)/sizeof(mux[0])) {
        currentAdcPin = 0;
      }
      ads1015.startADCReading(mux[currentAdcPin], false);
    }
#endif
    RTOS_WAIT_MS(20);
  }
}

#define TASKI2C_STACK_SIZE (1024 * 2)
#define TASKI2C_PRIO 5

static RTOS_TASK_HANDLE taskIdI2C;
RTOS_DEFINE_STACK(taskIdI2C, taskI2C_stack, TASKI2C_STACK_SIZE);
void adruino_adc_init(void) {
  adcInit(&arduino_hal_adc_driver);

  // The stuff on ADC are not that critical, so start a task and read it in the background
  RTOS_CREATE_TASK_EX(taskIdI2C,task_i2c,"I2C timer",taskI2C_stack,TASKI2C_STACK_SIZE,TASKI2C_PRIO,MIXER_TASK_CORE);
}