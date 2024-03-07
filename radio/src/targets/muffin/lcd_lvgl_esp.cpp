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

extern "C" {
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"
}

#if !defined(ESP_PLATFORM)
pixel_t LCD_FIRST_FRAME_BUFFER[DISPLAY_BUFFER_SIZE] __SDRAM;
pixel_t LCD_SECOND_FRAME_BUFFER[DISPLAY_BUFFER_SIZE] __SDRAM;
BitmapBuffer lcdBuffer1(BMP_RGB565, LCD_W, LCD_H, (uint16_t *)LCD_FIRST_FRAME_BUFFER);
BitmapBuffer lcdBuffer2(BMP_RGB565, LCD_W, LCD_H, (uint16_t *)LCD_SECOND_FRAME_BUFFER);
BitmapBuffer * lcdFront = &lcdBuffer1;
BitmapBuffer * lcd = &lcdBuffer2;
#else
extern uint32_t _ext_ram_bss_start;
pixel_t *LCD_FIRST_FRAME_BUFFER;
pixel_t *LCD_SECOND_FRAME_BUFFER;

BitmapBuffer * lcdFront = NULL;
BitmapBuffer * lcd = NULL;
#endif

static lv_area_t screen_area = {
    0, 0, LCD_W-1, LCD_H-1
};

extern BitmapBuffer * lcdFront;
extern BitmapBuffer * lcd;

lv_disp_drv_t disp_drv;
static lv_disp_draw_buf_t disp_buf;
static lv_disp_t* disp = nullptr;
static lv_indev_drv_t indev_drv;
extern lv_color_t* lcdbuf;

static lv_disp_drv_t* refr_disp = nullptr;

void lcdRefreshFull(lv_disp_drv_t* drv, uint16_t* buffer, const rect_t& areaExt);

static void flushLcd(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
#if !defined(LCD_VERTICAL_INVERT)
  // we're only interested in the last flush in direct mode
  if (!lv_disp_flush_is_last(disp_drv)) {
    lv_disp_flush_ready(disp_drv);
    return;
  }
#endif
  
#if defined(DEBUG_WINDOWS)
  if (area->x1 != 0 || area->x2 != LCD_W-1 || area->y1 != 0 ||
      area->y2 != LCD_H-1) {
    TRACE("partial refresh @ 0x%p {%d,%d,%d,%d}", color_p, area->x1,
          area->y1, area->x2, area->y2);
  } else {
    TRACE("full refresh @ 0x%p", color_p);
  }
#endif

  if (1) {
    refr_disp = disp_drv;

    rect_t copy_area = {area->x1, area->y1,
                        area->x2 - area->x1 + 1,
                        area->y2 - area->y1 + 1};

    lcdRefreshFull(disp_drv, (uint16_t*)color_p, copy_area);

#if !defined(LCD_VERTICAL_INVERT)
    uint16_t* src = (uint16_t*)color_p;
    uint16_t* dst = nullptr;
    if ((uint16_t*)color_p == LCD_FIRST_FRAME_BUFFER)
      dst = LCD_SECOND_FRAME_BUFFER;
    else
      dst = LCD_FIRST_FRAME_BUFFER;

    lv_disp_t* disp = _lv_refr_get_disp_refreshing();
    for(int i = 0; i < disp->inv_p; i++) {
      if(disp->inv_area_joined[i]) continue;

      const lv_area_t& refr_area = disp->inv_areas[i];

      auto area_w = refr_area.x2 - refr_area.x1 + 1;
      auto area_h = refr_area.y2 - refr_area.y1 + 1;

      DMACopyBitmap(dst, LCD_W, LCD_H, refr_area.x1, refr_area.y1,
                    src, LCD_W, LCD_H, refr_area.x1, refr_area.y1,
                    area_w, area_h);      
    }
    
    lv_disp_flush_ready(disp_drv);
#endif
  } else {
    lv_disp_flush_ready(disp_drv);
  }
}

void waitCb(_lv_disp_drv_t * disp_drv){
  //disp_wait_for_pending_transactions();
}

void lcdInitDisplayDriver()
{
#if defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488)
    lv_color_t* buf1 = (lv_color_t*)heap_caps_aligned_alloc(16, DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
#else
    lv_color_t* buf1 = lcdbuf;
#endif
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
#if defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488)
    lv_color_t* buf2 = (lv_color_t*)heap_caps_aligned_alloc(16, DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
#else
    lv_color_t* buf2 = &lcdbuf[DISP_BUF_SIZE];
#endif
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    memset(buf1, 0, DISPLAY_BUFFER_SIZE * sizeof(pixel_t));
    memset(buf2, 0, DISPLAY_BUFFER_SIZE * sizeof(pixel_t));

    uint32_t size_in_px = DISPLAY_BUFFER_SIZE;
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = flushLcd;
    //disp_drv.wait_cb = waitCb;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
#endif

    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    disp_drv.draw_buf = &disp_buf;
    disp = lv_disp_drv_register(&disp_drv);

    lv_disp_set_default(disp);

    // remove all styles on default screen (makes it transparent as well)
    lv_obj_remove_style_all(lv_scr_act());

    // transparent background:
    //  - this prevents LVGL overwritting things drawn directly into the bitmap buffer
    lv_disp_set_bg_opa(disp, LV_OPA_TRANSP);

    // allow drawing at any moment
    _lv_refr_set_disp_refreshing(disp);

  lcd = new BitmapBuffer(BMP_RGB565, LCD_W, LCD_H, (uint16_t *)buf1);
  lcdFront = new BitmapBuffer(BMP_RGB565, LCD_W, LCD_H, (uint16_t *)buf2);

  lv_draw_ctx_t * draw_ctx = disp->driver->draw_ctx;
  lcd->setDrawCtx(draw_ctx);
  lcdFront->setDrawCtx(draw_ctx);

  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = touch_driver_read;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);
}

void lcdRefreshFull(lv_disp_drv_t* drv, uint16_t* buffer, const rect_t& rect) {
  
  lv_color_t *p = (lv_color_t *)buffer;
  lv_coord_t maxLines = DISP_BUF_SIZE / rect.w;
  lv_area_t area = {.x1 = (lv_coord_t)rect.x, .y1 = (lv_coord_t)rect.y, .x2 = (lv_coord_t)(rect.right() - 1), .y2 = (lv_coord_t)(rect.y + maxLines - 1)};
  while (area.y1 < rect.bottom()) {
    int lines = rect.bottom() - area.y1;
    lines = LV_MIN(maxLines, lines);
    area.y2 = area.y1 + lines - 1;

    disp_driver_flush(drv, &area, p);
    area.y1 += lines;
    p += rect.w * lines; // OK to be added pass the end at the end of drawing
  }
}

static void _call_flush_cb(lv_disp_drv_t* drv, const lv_area_t* area,
                           lv_color_t* color_p)
{
  lv_area_t offset_area = {.x1 = (lv_coord_t)(area->x1 + drv->offset_x),
                           .y1 = (lv_coord_t)(area->y1 + drv->offset_y),
                           .x2 = (lv_coord_t)(area->x2 + drv->offset_x),
                           .y2 = (lv_coord_t)(area->y2 + drv->offset_y)};

  drv->flush_cb(drv, &offset_area, color_p);
}

static void _draw_buf_flush(lv_disp_t* disp)
{
  lv_disp_draw_buf_t* draw_buf = lv_disp_get_draw_buf(disp);

  /*Flush the rendered content to the display*/
  lv_draw_ctx_t* draw_ctx = disp->driver->draw_ctx;
  if (draw_ctx->wait_for_finish) draw_ctx->wait_for_finish(draw_ctx);

  /* In double buffered mode wait until the other buffer is freed
   * and driver is ready to receive the new buffer */
  if (draw_buf->buf1 && draw_buf->buf2) {
    while (draw_buf->flushing) {
      if (disp->driver->wait_cb)
        disp->driver->wait_cb(disp->driver);
    }
  }

  draw_buf->flushing = 1;
  draw_buf->flushing_last = 1;

  if (disp->driver->flush_cb) {
    _call_flush_cb(disp->driver, draw_ctx->buf_area,
                   (lv_color_t*)draw_ctx->buf);
  }

  /*If there are 2 buffers swap them. */
  if (draw_buf->buf1 && draw_buf->buf2) {
    if (draw_buf->buf_act == draw_buf->buf1)
      draw_buf->buf_act = draw_buf->buf2;
    else
      draw_buf->buf_act = draw_buf->buf1;
  }
}

void lcdRefresh()
{
  _draw_buf_flush(disp);
  lv_disp_flush_ready(disp->driver);
}

void lcdInitDirectDrawing() {
    lv_draw_ctx_t* draw_ctx = disp->driver->draw_ctx;
  draw_ctx->buf = disp->driver->draw_buf->buf_act;
  draw_ctx->buf_area = &screen_area;
  draw_ctx->clip_area = &screen_area;
  lcd->setData((pixel_t*)draw_ctx->buf);
  lcd->reset();
}

/*
  Starts LCD initialization routine. It should be called as
  soon as possible after the reset because LCD takes a lot of
  time to properly power-on.

  Make sure that delay_ms() is functional before calling this function!
*/
void lcdInit()
{
}

static TouchState internalTouchState = {0};
struct TouchState getInternalTouchState() {
  return internalTouchState;
}

static int state = 0;

struct TouchState touchPanelRead() {
  return internalTouchState;
}

bool touchPanelEventOccured() {
  bool ret = false;
  return ret;
}

bool touchPanelInit(void) {
    /* Register an input device when enabled on the menuconfig */
  //lv_indev_drv_t indev_drv;
  //lv_indev_drv_init(&indev_drv);
  //indev_drv.read_cb = touch_driver_read;
  //indev_drv.type = LV_INDEV_TYPE_POINTER;
  //lv_indev_drv_register(&indev_drv);

  return true;
}

void DMAWait()
{
}

void DMACopyBitmap(uint16_t *dest, uint16_t destw, uint16_t desth, uint16_t x,
                   uint16_t y, const uint16_t *src, uint16_t srcw,
                   uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w,
                   uint16_t h)
{
  for (int i = 0; i < h; i++) {
    memcpy(dest + (y + i) * destw + x, src + (srcy + i) * srcw + srcx, 2 * w);
  }
}

// 'src' has ARGB4444
// 'dest' has RGB565
void DMACopyAlphaBitmap(uint16_t *dest, uint16_t destw, uint16_t desth,
                        uint16_t x, uint16_t y, const uint16_t *src,
                        uint16_t srcw, uint16_t srch, uint16_t srcx,
                        uint16_t srcy, uint16_t w, uint16_t h)
{
  for (coord_t line = 0; line < h; line++) {
    uint16_t *p = dest + (y + line) * destw + x;
    const uint16_t *q = src + (srcy + line) * srcw + srcx;
    for (coord_t col = 0; col < w; col++) {
      uint8_t alpha = *q >> 12;
      uint8_t red =
          ((((*q >> 8) & 0x0f) << 1) * alpha + (*p >> 11) * (0x0f - alpha)) /
          0x0f;
      uint8_t green = ((((*q >> 4) & 0x0f) << 2) * alpha +
                       ((*p >> 5) & 0x3f) * (0x0f - alpha)) /
                      0x0f;
      uint8_t blue = ((((*q >> 0) & 0x0f) << 1) * alpha +
                      ((*p >> 0) & 0x1f) * (0x0f - alpha)) /
                     0x0f;
      *p = (red << 11) + (green << 5) + (blue << 0);
      p++;
      q++;
    }
  }
}

// 'src' has A8/L8?
// 'dest' has RGB565
void DMACopyAlphaMask(uint16_t *dest, uint16_t destw, uint16_t desth,
                      uint16_t x, uint16_t y, const uint8_t *src, uint16_t srcw,
                      uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w,
                      uint16_t h, uint16_t fg_color)
{
  RGB_SPLIT(fg_color, red, green, blue);

  for (coord_t line = 0; line < h; line++) {
    uint16_t *p = dest + (y + line) * destw + x;
    const uint8_t *q = src + (srcy + line) * srcw + srcx;
    for (coord_t col = 0; col < w; col++) {
      uint16_t opacity = *q >> 4;  // convert to 4 bits (stored in 8bit for DMA)
      uint8_t bgWeight = OPACITY_MAX - opacity;
      RGB_SPLIT(*p, bgRed, bgGreen, bgBlue);
      uint16_t r = (bgRed * bgWeight + red * opacity) / OPACITY_MAX;
      uint16_t g = (bgGreen * bgWeight + green * opacity) / OPACITY_MAX;
      uint16_t b = (bgBlue * bgWeight + blue * opacity) / OPACITY_MAX;
      *p = RGB_JOIN(r, g, b);
      p++;
      q++;
    }
  }
}

void lcdFlushed()
{
  // its possible to get here before flushLcd is ever called.
  // so check for nullptr first. (Race condition if you put breakpoints in startup code)
  if (refr_disp != nullptr)
    lv_disp_flush_ready(refr_disp);
}
