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
#include <driver/i2s_std.h>

#define ENABLE_SOUND

static uint32_t _sampleRate = AUDIO_SAMPLE_RATE;

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler

void audioInit() {
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(_sampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = (gpio_num_t)I2S_BCLK,
            .ws   = (gpio_num_t)I2S_LRCLK,
            .dout = (gpio_num_t)I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

void setSampleRate(uint32_t frequency) {
  const i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(frequency);

  i2s_channel_disable(tx_chan);
  i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
  i2s_channel_enable(tx_chan);
}

uint8_t * currentBuffer = nullptr;
uint32_t currentSize = 0;
int16_t newVolume = -1;

void audioSetCurrentBuffer(const AudioBuffer * buffer)
{
  if (buffer) {
    currentBuffer = (uint8_t *)buffer->data;
    currentSize = buffer->size * 2;
  }
  else {
    currentBuffer = nullptr;
    currentSize = 0;
  }
}


EXT_RAM_BSS_ATTR static uint8_t zero[128] = {0};

void audioConsumeCurrentBuffer() {
  if (!currentBuffer) {
    audioSetCurrentBuffer(audioQueue.buffersFifo.getNextFilledBuffer());
  }

  static size_t last = 0U;
  if ((NULL == currentBuffer) && (0U != last)) {
    size_t written = 0U;
    i2s_channel_write(tx_chan, zero, last * 2, &written, 1000); // TODO: to mute the speaker? Not sure why after tone playing the speaker keeps having noise even if not writing to the I2S
    last = 0U;
  }
  while (currentBuffer && currentSize) {
    size_t written = 0U;
#ifdef ENABLE_SOUND
    i2s_channel_write(tx_chan, currentBuffer, currentSize, &written, 1000);
    last = written;
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
