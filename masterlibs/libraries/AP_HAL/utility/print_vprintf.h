#pragma once

#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

#if CONFIG_SHAL_CORE == SHAL_CORE_APM
void print_vprintf (AP_HAL::Print *s, unsigned char in_progmem, const char *fmt, va_list ap);
#else
void print_vprintf(AP_HAL::Print *s, const char *fmt, va_list ap);
#endif
