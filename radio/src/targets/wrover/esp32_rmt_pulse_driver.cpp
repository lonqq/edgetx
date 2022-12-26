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
#include "esp32_rmt_pulse.h"

static void esp32_rmt_ctx_free(rmt_ctx_t *ctx);

static void esp32_rmt_tx_task(void * pdata) {
    rmt_ctx_t *ctx = (rmt_ctx_t *)pdata;
    while(!ctx->exit) {
        size_t count = ctx->encoder(ctx);
        if (0U != count) {
            rmtWriteBlocking(ctx->rmt, ctx->data, count);
        }
    }
    esp32_rmt_ctx_free(ctx); // done with the ctx
    vTaskDelete(NULL);
}

rmt_ctx_t *esp32_rmt_tx_init(int pin, rmt_reserve_memsize_t memsize, float tick_in_ns, rmt_tx_encode_cb_t enc_fn, size_t pulse_in_frame) {
    rmt_ctx_t *rvalue = (rmt_ctx_t *)malloc(sizeof(rmt_ctx_t));

    if (NULL != rvalue) {
        rvalue->encoder = enc_fn;
        rvalue->exit = false;
        rvalue->rmt = rmtInit(pin, RMT_TX_MODE, memsize);
        rvalue->data = (rmt_data_t *)malloc(sizeof(rmt_data_t) * pulse_in_frame);
        if ((NULL != rvalue->rmt) && (NULL != rvalue->data)) {
            rvalue->tick_in_ns = rmtSetTick(rvalue->rmt, tick_in_ns);
            xTaskCreateStaticPinnedToCore(esp32_rmt_tx_task, "esp32_rmt_tx_task", sizeof(rvalue->rmt_tx_stack)/sizeof(rvalue->rmt_tx_stack[0]),
                    rvalue, 5, rvalue->rmt_tx_stack, &rvalue->task_struct, TRAINER_PPM_OUT_TASK_CORE); // TODO-feather priority
        } else {
            free(rvalue);
            rvalue = NULL;
        }
    }
    return rvalue;
}

static void esp32_rmt_rx_cb(uint32_t *data, size_t len, void *arg) {
    rmt_ctx_t *ctx = (rmt_ctx_t *)arg;
    ctx->decoder(ctx, data, len);
    if (ctx->exit) {
        rmtRead(ctx->rmt, NULL, ctx);
        esp32_rmt_ctx_free(ctx); // done with the ctx
    }
}

rmt_ctx_t *esp32_rmt_rx_init(int pin, rmt_reserve_memsize_t memsize, float tick_in_ns, rmt_rx_decode_cb_t dec_fn, size_t pulse_in_frame, size_t idle_threshold_in_ns, size_t min_pulse_in_ns) {
    rmt_ctx_t *rvalue = (rmt_ctx_t *)malloc(sizeof(rmt_ctx_t));

    if (NULL != rvalue) {
        rvalue->decoder = dec_fn;
        rvalue->exit = false;
        rvalue->rmt = rmtInit(pin, RMT_RX_MODE, memsize);
        if (NULL != rvalue->rmt) {
            rvalue->tick_in_ns = rmtSetTick(rvalue->rmt, tick_in_ns);
            rmtSetRxThreshold(rvalue->rmt, idle_threshold_in_ns / rvalue->tick_in_ns);
            if (0U != min_pulse_in_ns) {
                rmtSetFilter(rvalue->rmt, true, min_pulse_in_ns / rvalue->tick_in_ns);
            }
            rmtRead(rvalue->rmt, esp32_rmt_rx_cb, rvalue);
        } else {
            free(rvalue);
            rvalue = NULL;
        }
    }
    return rvalue;
}

void esp32_rmt_stop(rmt_ctx_t *ctx) {
    ctx->exit = true;
    if (NULL != ctx->decoder) { // RX
        rmtEnd(ctx->rmt); // force the read to end
    }
    // no thing special need to be done for TX. It will end after the last frame sends out
}

static void esp32_rmt_ctx_free(rmt_ctx_t *ctx) {
    if (NULL != ctx) {
        if (NULL != ctx->rmt) {
            rmtDeinit(ctx->rmt);
        }
        if (NULL != ctx->data) {
            free(ctx->data);
        }

        free(ctx);
    }
}

int rmt_ppm_decode_cb(rmt_ctx_t *ctx, uint32_t *rxdata, size_t rxdata_len, int16_t *ppm_decode_buf)
{
    int channel = 0;
    uint32_t high = 0U;
    for (size_t i = 0; i < rxdata_len; i++) {
        rmt_data_t d;
        d.val = rxdata[i];
        if (i != 0) {
            uint32_t val = (high + d.duration0) * (ctx->tick_in_ns / 1000); // result in us
            if (val > 800 && val < 2200) {
                ppm_decode_buf[channel++] =
                        (int16_t)(val - 1500) * (g_eeGeneral.PPM_Multiplier+10) / 10;  // +-500 != 512, but close enough.
            } else {
                channel = -1;
                break;
            }
        }
        high = d.duration1;
    }
    return channel;
}

void rmt_ppm_encode_cb(rmt_ctx_t *ctx, uint16_t *ppm, size_t len)
{
    // setupPulsesPPM() always setup data with 0.5us tick
    for (int i = 0; i < len; i++) {
        ctx->data[i].duration0 = ppm[i] - 600;
        ctx->data[i].level0 = 1;
        ctx->data[i].duration1 = 600;
        ctx->data[i].level1 = 0;
    }
    ctx->data[len].val = 0;
}