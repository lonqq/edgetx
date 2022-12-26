/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
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
#include "intmodule_serial_driver.h"

HardwareSerial *intmod_uart = &Serial1;

uint32_t get_ardu_serial_config(const etx_serial_init* params) {
  uint32_t rvalue = 0;
  if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_None) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8N1;
  } else if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_Even) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8E1;
  } else if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_Odd) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8O1;
  } else if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_None) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8N2;
  } else if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_Even) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8E2;
  } else if ((params->word_length == ETX_WordLength_8) &&
      (params->parity == ETX_Parity_Odd) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8O2;
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_None) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8N1;  // It seems Arduino does not support 9 bits?
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_Even) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8E1;
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_Odd) &&
      (params->stop_bits == ETX_StopBits_One)) {
    rvalue = SERIAL_8O1;
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_None) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8N2;
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_Even) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8E2;
  } else if ((params->word_length == ETX_WordLength_9) &&
      (params->parity == ETX_Parity_Odd) &&
      (params->stop_bits == ETX_StopBits_Two)) {
    rvalue = SERIAL_8O2;
  }
  return rvalue;
}
void* intmoduleSerialStart(const etx_serial_init* params)
{
  if (!params) return nullptr;
  intmod_uart->begin(params->baudrate, get_ardu_serial_config(params));
  return (void*)intmod_uart;
}

void intmoduleSendByte(void* ctx, uint8_t byte)
{
  HardwareSerial *uart = (HardwareSerial *)ctx;
  uart->write(byte);
}

void intmoduleSendBuffer(void* ctx, const uint8_t * data, uint8_t size)
{
  HardwareSerial *uart = (HardwareSerial *)ctx;
  uart->write(data, size);
}

void intmoduleWaitForTxCompleted(void* ctx)
{
}

static int intmoduleGetByte(void* ctx, uint8_t* data)
{
  HardwareSerial *uart = (HardwareSerial *)ctx;
  return (int)uart->read(data, 1);
}

static void intmoduleClearRxBuffer(void* ctx)
{
}

static void intmoduleSerialStop(void* ctx)
{
  HardwareSerial *uart = (HardwareSerial *)ctx;
  uart->end();
}

void intmoduleStop()
{
  intmoduleSerialStop(intmod_uart);
}

const etx_serial_driver_t IntmoduleSerialDriver = {
  .init = intmoduleSerialStart,
  .deinit = intmoduleSerialStop,
  .sendByte = intmoduleSendByte,
  .sendBuffer = intmoduleSendBuffer,
  .waitForTxCompleted = intmoduleWaitForTxCompleted,
  .getByte = intmoduleGetByte,
  .clearRxBuffer = intmoduleClearRxBuffer,
  .getBaudrate = nullptr,
  .setReceiveCb = nullptr,
  .setBaudrateCb = nullptr,
};
