/*
   Joypad Interface Driver for URUS and Ardupilot.
   Copyright (c) 2017-2018 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "AP_Joypad_Backend.h"

AP_Joypad_Backend::AP_Joypad_Backend(AP_Joypad &joypad) :
    _joypad(joypad)
{}

void AP_Joypad_Backend::process(AP_Joypad::ProcessMode process_mode)
{}

void AP_Joypad_Backend::update()
{}

void AP_Joypad_Backend::set_button_array_data(uint8_t data_button[], uint8_t playerID)
{}

#endif

