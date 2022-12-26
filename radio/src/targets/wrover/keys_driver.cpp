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
#include <Adafruit_MCP23X17.h>

#define MCP1_SWITCHES_MASK 0xF8
#define MCP1_TRIM_MASK 0xFF

static uint8_t mcp1_trim = 0U;
  // ~0x02 thr trim down
  // ~0x01 thr trim up
  // ~0x08 ele trim down
static uint8_t switches = 0x0U;
  // ~0x08 swc up
  // ~0x80 swa down
  // ~0x40 swb down
  // ~0x10 swc down
  // ~0x20 swd down

#define MCP1_SW_PIN_A 0x80
#define MCP1_SW_PIN_B 0x40
#define MCP1_SW_PIN_D 0x20
#define MCP1_SW_PIN_C_H 0x08
#define MCP1_SW_PIN_C_L 0x10

#define ADD_2POS_CASE(x) \
  case SW_S ## x ## 0: \
    xxx = switches  & MCP1_SW_PIN_ ## x ; \
    break; \
  case SW_S ## x ## 2: \
    xxx = ~switches  & MCP1_SW_PIN_ ## x ; \
    break
#define ADD_INV_2POS_CASE(x) \
  case SW_S ## x ## 2: \
    xxx = switches  & MCP1_SW_PIN_ ## x ; \
    break; \
  case SW_S ## x ## 0: \
    xxx = ~switches  & MCP1_SW_PIN_ ## x ; \
    break
#define ADD_3POS_CASE(x, i) \
  case SW_S ## x ## 0: \
    xxx = (switches & MCP1_SW_PIN_ ## x ## _H); \
    if (IS_3POS(i)) { \
      xxx = xxx && (~switches & MCP1_SW_PIN_ ## x ## _L); \
    } \
    break; \
  case SW_S ## x ## 1: \
    xxx = (~switches & MCP1_SW_PIN_ ## x ## _H) && (~switches & MCP1_SW_PIN_ ## x ## _L); \
    break; \
  case SW_S ## x ## 2: \
    xxx = (~switches & MCP1_SW_PIN_ ## x ## _H); \
    if (IS_3POS(i)) { \
      xxx = xxx && (switches & MCP1_SW_PIN_ ## x ## _L); \
    } \
    break
#define ADD_INV_3POS_CASE(x, i) \
  case SW_S ## x ## 2: \
    xxx = (switches & MCP1_SW_PIN_ ## x ## _H); \
    if (IS_3POS(i)) { \
      xxx = xxx && (~switches & MCP1_SW_PIN_ ## x ## _L); \
    } \
    break; \
  case SW_S ## x ## 1: \
    xxx = (switches & MCP1_SW_PIN_ ## x ## _H) && (switches & MCP1_SW_PIN_ ## x ## _L); \
    break; \
  case SW_S ## x ## 0: \
    xxx = (~switches & MCP1_SW_PIN_ ## x ## _H); \
    if (IS_3POS(i)) { \
      xxx = xxx && (switches & MCP1_SW_PIN_ ## x ## _L); \
    } \
    break

#ifndef DISABLE_I2C_DEVS
static Adafruit_MCP23X17 mcp;
static Adafruit_MCP23X17 mcp1;
#endif
static RTOS_MUTEX_HANDLE keyMutex;
uint32_t readKeys()
{
  uint32_t result = 0;
#ifndef DISABLE_I2C_DEVS
  RTOS_LOCK_MUTEX(keyMutex);
  uint8_t mask = (1 << BUTTONS_ON_GPIOA) - 1;
  uint8_t gpioA = mcp.readGPIOA();
  result |= (gpioA ^ mask) & mask;

  gpioA = mcp1.readGPIOA();
  mcp1_trim = gpioA & MCP1_TRIM_MASK;
  uint8_t gpioB = mcp1.readGPIOB();
  switches = (gpioB & MCP1_SWITCHES_MASK) ^ MCP1_SWITCHES_MASK;
  RTOS_UNLOCK_MUTEX(keyMutex);
#endif
  return result;
}

uint32_t readTrims()
{
  uint32_t result = mcp1_trim ^ MCP1_TRIM_MASK;
  return result;
}

bool trimDown(uint8_t idx)
{
  return readTrims() & (1 << idx);
}

bool keyDown()
{
  return readKeys() || readTrims();
}

/* TODO common to ARM */
void readKeysAndTrims()
{
  uint8_t index = 0;
  uint32_t keys_input = readKeys();
  for (unsigned i = 1; i != unsigned(1 << TRM_BASE); i <<= 1) {
    keys[index++].input(keys_input & i);
  }

  uint32_t trims_input = readTrims();
  for (uint8_t i = 1; i != uint8_t(1 << 8); i <<= 1) {
    keys[index++].input(trims_input & i);
  }

#if defined(PWR_BUTTON_PRESS)
  if ((keys_input || trims_input || pwrPressed()) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#else
  if ((keys_input || trims_input) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#endif
    // on keypress turn the light on
    resetBacklightTimeout();
  }
}

#if !defined(BOOT)
uint32_t switchState(uint8_t index)
{
  uint32_t xxx = 0;

  switch (index) {
    ADD_INV_2POS_CASE(A);
    ADD_INV_2POS_CASE(B);
    ADD_3POS_CASE(C, 2);
    ADD_INV_2POS_CASE(D);
  }

  return xxx;
}
#endif

void keysInit()
{
  RTOS_CREATE_MUTEX(keyMutex);
#ifndef DISABLE_I2C_DEVS
  mcp.begin_I2C(MCP23XXX_ADDR, &Wire);
  mcp1.begin_I2C(MCP23XXX_ADDR + 1, &Wire);

  for (int i = 0; i < BUTTONS_ON_GPIOA; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }
  for (int i = 0; i < 8; i++) {
    mcp1.pinMode(i, INPUT_PULLUP);
  }
  for (int i = 11; i < 16; i++) {
    mcp1.pinMode(i, INPUT_PULLUP);
  }
#endif
}

void INTERNAL_MODULE_ON(void) {

}
void INTERNAL_MODULE_OFF(void) {
}

bool IS_INTERNAL_MODULE_ON(void) {
  return false;
}
