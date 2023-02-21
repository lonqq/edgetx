#include "opentx.h"

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

static disp_backlight_h bklite_handle = NULL;
static disp_backlight_config_t bklite_cfg = {
    .pwm_control = true,
    .output_invert = false,
    .gpio_num = BACKLITE_PIN,

    // Relevant only for PWM controlled backlight
    // Ignored for switch (ON/OFF) backlight control
    .timer_idx = 0,
    .channel_idx = 0
};

static bool bkl_enabled = false;
void backlightInit() {
    bklite_handle = disp_backlight_new(&bklite_cfg);
    disp_backlight_set(bklite_handle, 100);
}

void backlightEnable(uint8_t level) {
#if 0
  static uint8_t prev_level = 0;
  if (level != prev_level) {
    uint16_t l = 100;//(((uint16_t)level) * 100 / BACKLIGHT_LEVEL_MAX) + 80; // TODO
    prev_level = l;
    disp_backlight_set(bklite_handle, l);
  }
#endif
  bkl_enabled = true;
}

void backlightDisable() {
#if 0
  disp_backlight_set(bklite_handle, 0);
#endif
  bkl_enabled = false;
}
uint8_t isBacklightEnabled() {return bkl_enabled;}
