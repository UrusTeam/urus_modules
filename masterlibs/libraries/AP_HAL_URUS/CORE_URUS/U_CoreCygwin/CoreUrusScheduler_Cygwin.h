#pragma once

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN)

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusScheduler.h"

#define URUS_SCHEDULER_MAX_TIMER_PROCS 4
#include  <stdio.h>
#include <windows.h>

/* Scheduler implementation: */
class CLCoreUrusScheduler_Cygwin : public NSCORE_URUS::CLCoreUrusScheduler {
public:
    CLCoreUrusScheduler_Cygwin();
    static CLCoreUrusScheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<CLCoreUrusScheduler_Cygwin*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    void init() override;
    void delay(uint16_t ms);
    void delay_microseconds(uint16_t us);
    void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

    void register_timer_process(AP_HAL::MemberProc);
    void register_io_process(AP_HAL::MemberProc);
    void suspend_timer_procs();
    void resume_timer_procs();

    bool in_timerprocess();

    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void system_initialized();

    void reboot(bool hold_in_bootloader);

    bool interrupts_are_blocked(void) {
        return _nested_atomic_ctr != 0;
    }

    void sitl_begin_atomic() {
        _nested_atomic_ctr++;
    }
    void sitl_end_atomic();

    void timer_event() {
        _run_timer_procs(true);
        _run_io_procs(true);
    }

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    bool     in_main_thread() const override;

private:
    uint8_t _nested_atomic_ctr;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    static AP_HAL::Proc _failsafe;

    static void _run_timer_procs(bool called_from_isr);
    static void _run_io_procs(bool called_from_isr);

    static void *_fire_isr_timer(void *arg);
    static void *_fire_isr_sched(void *arg);

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[URUS_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::MemberProc _io_proc[URUS_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;
    static bool _in_timer_proc;
    static bool _in_io_proc;

    void stop_clock(uint64_t time_usec);
    static void usleep_win(DWORD waitTime);

    bool _initialized;
    uint64_t _stopped_clock_usec;

    static bool _isr_timer_running;
    static bool _isr_sched_running;

    uint32_t now_micros;
    uint16_t dt_micros;
    uint16_t centinel_micros;
    uint16_t ms_cb;
    uint32_t start;

};
#endif // __CYGWIN__
