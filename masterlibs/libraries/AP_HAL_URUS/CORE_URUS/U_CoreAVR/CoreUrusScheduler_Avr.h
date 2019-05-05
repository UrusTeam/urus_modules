#pragma once

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if CONFIG_SHAL_CORE == SHAL_CORE_APM

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusScheduler.h"
#include "../CoreUrusTimers.h"

#include "CoreUrusTimers_Avr.h"
#include "CoreUrusScheduler_Avr.h"

#include "utility/ISRRegistry.h"

#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
#define URUS_SCHEDULER_MAX_TIMER_PROCS 5
#elif defined(SHAL_CORE_APM16U)
#define URUS_SCHEDULER_MAX_TIMER_PROCS 1
#else
#define URUS_SCHEDULER_MAX_TIMER_PROCS 3
#endif

/* Scheduler implementation: */
class CLCoreUrusScheduler_Avr : public NSCORE_URUS::CLCoreUrusScheduler {
public:
    CLCoreUrusScheduler_Avr();
    static CLCoreUrusScheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<CLCoreUrusScheduler_Avr*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    void init() override;
    void delay(uint16_t ms) override;
    void delay_microseconds(uint16_t us) override;
    void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

    void register_timer_process(AP_HAL::MemberProc);
    void register_io_process(AP_HAL::MemberProc);
    void suspend_timer_procs();
    void resume_timer_procs();

    void register_timer_failsafe(AP_HAL::Proc cb, uint32_t period_us);

    void system_initialized();

    void reboot(bool hold_in_bootloader);

    void timer_event() override {
        _run_timer_procs(true);
    }

    bool in_main_thread() const override;

private:

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static AP_HAL::Proc _failsafe;

    static void _run_timer_procs(bool called_from_isr);

    static void _fire_isr_sched();

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[URUS_SCHEDULER_MAX_TIMER_PROCS];

    static uint8_t _num_timer_procs;
    static bool _in_timer_proc;

    bool _initialized;

    uint32_t now_micros;
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    uint16_t dt_micros;
    uint16_t centinel_micros;
    uint16_t ms_cb;
#endif
    uint32_t start;
    volatile bool _in_delay_proc;

    static volatile uint8_t _timer_reset_value;

};
#endif // CONFIG_SHAL_CORE == SHAL_CORE_APM
