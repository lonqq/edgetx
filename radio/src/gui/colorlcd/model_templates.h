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

#pragma once

#include <functional>

#include "opentx.h"

class TemplatePage : public Page
{
 public:
  TemplatePage();

  void updateInfo();

#if defined(HARDWARE_KEYS)
  void onEvent(event_t event) override;
#endif

#if defined(DEBUG_WINDOWS)
  std::string getName() const { return "TemplatePage"; }
#endif

 protected:
  FormWindow* listWindow = nullptr;
  lv_obj_t* infoLabel = nullptr;

  static constexpr size_t LEN_INFO_TEXT = 300;
  static constexpr size_t LEN_PATH =
      sizeof(TEMPLATES_PATH) + TEXT_FILENAME_MAXLEN;
  static constexpr size_t LEN_BUFFER =
      sizeof(TEMPLATES_PATH) + 2 * TEXT_FILENAME_MAXLEN + 1;

  char buffer[LEN_BUFFER + 1] = "";
  char infoText[LEN_INFO_TEXT + 1] = "";
  static std::function<void(void)> update;
};

class SelectTemplateFolder : public TemplatePage
{
 public:
  SelectTemplateFolder(std::function<void(void)> update);
  ~SelectTemplateFolder();
};
