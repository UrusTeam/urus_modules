#pragma once

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#if defined(SHAL_CORE_APM328)
#define ISR_REGISTRY_TIMER2_OVF  0
#define ISR_REGISTRY_TIMER1_CAPT 1
#define ISR_REGISTRY_NUM_SLOTS   2
#elif defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
#define ISR_REGISTRY_TIMER2_OVF  0
#define ISR_REGISTRY_TIMER4_CAPT 1
#define ISR_REGISTRY_TIMER5_CAPT 2
#define ISR_REGISTRY_NUM_SLOTS   3
#elif defined(SHAL_CORE_APM16U)
#define ISR_REGISTRY_TIMER0_OVF  0
#define ISR_REGISTRY_NUM_SLOTS   1
#else
#error "UNKNOWN ISR_REGISTRY MACROS CORE BOARD FOR PINS!"
#endif

class ISRRegistry {
public:
    int register_signal(int isr_number, AP_HAL::Proc proc);
    int unregister_signal(int isr_number);

    static AP_HAL::Proc _registry[ISR_REGISTRY_NUM_SLOTS];
};

#endif  // CONFIG_SHAL_CORE == SHAL_CORE_APM

