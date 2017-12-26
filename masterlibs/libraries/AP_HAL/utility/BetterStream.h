/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
//
#pragma once

#include <stdarg.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Namespace.h>

#include "Stream.h"
#include <AP_Progmem/AP_Progmem.h>

class AP_HAL::BetterStream : public AP_HAL::Stream {
public:
    BetterStream(void) {}

    virtual void printf(const char *, ...) FMT_PRINTF(2, 3) = 0;
    virtual void vprintf(const char *, va_list) = 0;

    /* No format checking on printf_P: can't currently support that on AVR */
    virtual void _printf_P(const prog_char *, ...) = 0;
    virtual void print_P(const prog_char_t *) = 0;
    virtual void println_P(const prog_char_t *) = 0;
    virtual void vprintf_P(const prog_char *, va_list) = 0;
};
