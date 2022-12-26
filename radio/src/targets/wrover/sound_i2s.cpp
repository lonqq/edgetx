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
#include <driver/i2s.h>

#define ENABLE_SOUND

static uint32_t _sampleRate = AUDIO_SAMPLE_RATE;

void audioInit() {
#ifdef ENABLE_SOUND
  static const i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),   // The main controller can transmit data but not receive.
    .sample_rate = _sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,   // 16 bits per sample
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,   // 1-channels
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,   // I2S communication I2S Philips standard, data launch at second BCK
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,   // Interrupt level 2
    .dma_buf_count = 4,   // number of buffers, 128 max.
    .dma_buf_len = 400,   // size of each buffer, AVRC communication may be affected if the value is too high.
    .use_apll = false,   // For the application of a high precision clock, select the APLL_CLK clock source in the frequency range of 16 to 128 MHz. It's not the case here, so select false.
    .tx_desc_auto_clear = true
  };

  static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,   // Serial clock (SCK), aka bit clock (BCK)
    .ws_io_num = I2S_LRCLK,   // Word select (WS), i.e. command (channel) select, used to switch between left and right channel data
    .data_out_num = I2S_DOUT,   // Serial data signal (SD), used to transmit audio data in two's complement format
    .data_in_num = I2S_PIN_NO_CHANGE   // Not used
  };

  if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)){
    TRACE_ERROR("Install and start I2S driver failed !");
  } else if (i2s_set_pin(I2S_NUM_0, &pin_config)){
    TRACE_ERROR("Set I2S pin number failed !");
  }
#endif
}

static bool inited = false;
void setSampleRate(uint32_t frequency) {
  if (!inited) {
    audioInit();
    inited = true;
  }
#ifdef ENABLE_SOUND
  i2s_set_sample_rates(I2S_NUM_0, frequency);
#endif
}

uint8_t * currentBuffer = nullptr;
uint32_t currentSize = 0;
int16_t newVolume = -1;

void audioSetCurrentBuffer(const AudioBuffer * buffer)
{
  if (!inited) {
    audioInit();
    inited = true;
  }
  if (buffer) {
    currentBuffer = (uint8_t *)buffer->data;
    currentSize = buffer->size * 2;
  }
  else {
    currentBuffer = nullptr;
    currentSize = 0;
  }
}

void audioConsumeCurrentBuffer() {
  if (!currentBuffer) {
    audioSetCurrentBuffer(audioQueue.buffersFifo.getNextFilledBuffer());
  }

  while (currentBuffer && currentSize) {
    uint32_t written = 0U;
#ifdef ENABLE_SOUND
    i2s_write(I2S_NUM_0, currentBuffer, 4, &written, 100);
#else
    written = currentSize;
#endif
    if (written > currentSize) written = currentSize;
    currentBuffer += written;
    currentSize -= written;
    if (currentSize == 0) {
      audioQueue.buffersFifo.freeNextFilledBuffer();
      currentBuffer = nullptr;
      currentSize = 0;
    }
  }
}
