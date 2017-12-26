#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

void init();

#if CONFIG_SHAL_CORE == SHAL_CORE_APM
void panic(const prog_char_t *errormsg) NORETURN;
#else
void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;
#endif

uint32_t micros();
uint32_t millis();
uint64_t micros64();
uint64_t millis64();

} // namespace AP_HAL
