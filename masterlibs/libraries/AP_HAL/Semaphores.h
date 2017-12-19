#pragma once

#include "AP_HAL_Namespace.h"

#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
#define HAL_SEMAPHORE_BLOCK_FOREVER ((uint32_t) 0xFFFFFFFF)
#else
#define HAL_SEMAPHORE_BLOCK_FOREVER 0
#endif

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;
    virtual bool give() = 0;
    virtual ~Semaphore(void) {}
};
