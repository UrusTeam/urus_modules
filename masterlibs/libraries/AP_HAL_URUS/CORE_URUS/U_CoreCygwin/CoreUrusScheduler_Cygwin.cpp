#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN)

#include "../CORE_URUS_NAMESPACE.h"
#include "../CORE_URUS.h"

#include "../CoreUrusScheduler.h"
#include "CoreUrusScheduler_Cygwin.h"
#include "CoreUrusUARTDriver_Cygwin.h"

#include <sys/time.h>
#include <fenv.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;
static const NSCORE_URUS::CLCORE_URUS& _urus_core = NSCORE_URUS::get_CORE();

AP_HAL::Proc CLCoreUrusScheduler_Cygwin::_failsafe = nullptr;
volatile bool CLCoreUrusScheduler_Cygwin::_timer_suspended = false;
volatile bool CLCoreUrusScheduler_Cygwin::_timer_event_missed = false;

AP_HAL::MemberProc CLCoreUrusScheduler_Cygwin::_timer_proc[URUS_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t CLCoreUrusScheduler_Cygwin::_num_timer_procs = 0;
bool CLCoreUrusScheduler_Cygwin::_in_timer_proc = false;

AP_HAL::MemberProc CLCoreUrusScheduler_Cygwin::_io_proc[URUS_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t CLCoreUrusScheduler_Cygwin::_num_io_procs = 0;
bool CLCoreUrusScheduler_Cygwin::_in_io_proc = false;

bool CLCoreUrusScheduler_Cygwin::_isr_timer_running = false;
bool CLCoreUrusScheduler_Cygwin::_isr_sched_running = false;

CLCoreUrusScheduler_Cygwin::CLCoreUrusScheduler_Cygwin() :
    NSCORE_URUS::CLCoreUrusScheduler(),
    _stopped_clock_usec(0)
{
}

bool CLCoreUrusScheduler_Cygwin::in_main_thread() const
{
    return !_in_timer_proc && !_in_io_proc;
}

void CLCoreUrusScheduler_Cygwin::usleep_win(DWORD waitTime)
{
	LARGE_INTEGER perfCnt_time, start_time, now_time;

	QueryPerformanceFrequency(&perfCnt_time);
	QueryPerformanceCounter(&start_time);

	do {
		QueryPerformanceCounter((LARGE_INTEGER*) &now_time);
	} while ((now_time.QuadPart - start_time.QuadPart) / float(perfCnt_time.QuadPart) * 1000 * 1000 < waitTime);
}

void *CLCoreUrusScheduler_Cygwin::_fire_isr_timer(void *arg)
{

    if (_isr_timer_running) {
        return NULL;
    }

    _isr_timer_running = true;

    while(1) {
        /* this make frequency granulation for
         * shal isr timer with posix thread
         */
        usleep_win(clk_core_timers.isr_time);
        //hal.scheduler->delay_microseconds(clk_core_timers.isr_time);
        fire_isr_timer();
    }

    _isr_timer_running = false;

    return NULL;
}

void *CLCoreUrusScheduler_Cygwin::_fire_isr_sched(void *arg)
{

    if (_isr_sched_running) {
        return NULL;
    }

    _isr_sched_running = true;

    while(1) {
        /* this make frequency granulation for
         * shal isr timer with posix thread
         */
        usleep_win((clk_core_timers.isr_time / CORE_SPEED_FREQ_PERCENT));
        fire_isr_sched();
    }
    _isr_sched_running = false;

    return NULL;
}

void CLCoreUrusScheduler_Cygwin::init()
{

    clk_core_timers.clk_per_sec = CLOCKS_PER_SEC;
    start_sched();

    /* See CLCoreUrusScheduler::fire_isr_timer on the TOP SHAL. */
    pthread_t isr_timer_thread;
    pthread_attr_t thread_attr_timer;

    pthread_attr_init(&thread_attr_timer);
    pthread_attr_setstacksize(&thread_attr_timer, 2048);

    pthread_attr_setschedpolicy(&thread_attr_timer, SCHED_FIFO);

    pthread_create(&isr_timer_thread, &thread_attr_timer, &_fire_isr_timer, this);

    /* See CLCoreUrusScheduler::fire_isr_sched on the TOP SHAL. */
    pthread_t isr_sched_thread;
    pthread_attr_t thread_attr_sched;

    pthread_attr_init(&thread_attr_sched);
    pthread_attr_setstacksize(&thread_attr_sched, 2048);

    pthread_attr_setschedpolicy(&thread_attr_sched, SCHED_FIFO);

    pthread_create(&isr_sched_thread, &thread_attr_sched, &_fire_isr_sched, this);

#if 0
    printf("Cygwin Scheduler ok!\n");
#endif
}

void CLCoreUrusScheduler_Cygwin::delay_microseconds(uint16_t usec)
{
    uint32_t start_micros = AP_HAL::micros();
    while ((AP_HAL::micros() - start_micros) < usec);
}

void CLCoreUrusScheduler_Cygwin::delay(uint16_t ms)
{
    start = AP_HAL::millis();
    now_micros = AP_HAL::micros();
    dt_micros = 0;
    centinel_micros = URUS_MAGIC_TIME;
    ms_cb = ms;

    while ((AP_HAL::millis() - start) < ms) {
        dt_micros = AP_HAL::micros() - now_micros;
        now_micros = AP_HAL::micros();
        delay_microseconds(centinel_micros);

        if (dt_micros > centinel_micros) {
            centinel_micros = dt_micros - (dt_micros - centinel_micros);
        }

        if (dt_micros < centinel_micros) {
            centinel_micros = dt_micros + (centinel_micros - dt_micros);
        }

        ms_cb--;
        if (_min_delay_cb_ms < ms_cb) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}

void CLCoreUrusScheduler_Cygwin::register_delay_callback(AP_HAL::Proc proc,
        uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void CLCoreUrusScheduler_Cygwin::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < URUS_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }
}

void CLCoreUrusScheduler_Cygwin::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < URUS_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    }
}

void CLCoreUrusScheduler_Cygwin::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void CLCoreUrusScheduler_Cygwin::suspend_timer_procs() {
    _timer_suspended = true;
}

void CLCoreUrusScheduler_Cygwin::resume_timer_procs() {
    _timer_suspended = false;
    if (_timer_event_missed) {
        _timer_event_missed = false;
        _run_timer_procs(false);
    }
}

bool CLCoreUrusScheduler_Cygwin::in_timerprocess() {
    return _in_timer_proc || _in_io_proc;
}

void CLCoreUrusScheduler_Cygwin::system_initialized() {
    if (_initialized) {
        AP_HAL::panic(
            "PANIC: scheduler system initialized called more than once");
    }
    int exceptions = FE_OVERFLOW | FE_DIVBYZERO;
#ifndef __i386__
    // i386 with gcc doesn't work with FE_INVALID
    exceptions |= FE_INVALID;
#endif

    feclearexcept(exceptions);

    _initialized = true;
}

void CLCoreUrusScheduler_Cygwin::sitl_end_atomic() {
    if (_nested_atomic_ctr == 0) {
        hal.uartA->println("NESTED ATOMIC ERROR");
    } else {
        _nested_atomic_ctr--;
    }
}

void CLCoreUrusScheduler_Cygwin::reboot(bool hold_in_bootloader)
{
    hal.uartA->println("REBOOT NOT IMPLEMENTED\r\n");
}

void CLCoreUrusScheduler_Cygwin::_run_timer_procs(bool called_from_isr)
{
    if (_in_timer_proc) {
        // the timer calls took longer than the period of the
        // timer. This is bad, and may indicate a serious
        // driver failure. We can't just call the drivers
        // again, as we could run out of stack. So we only
        // call the _failsafe call. It's job is to detect if
        // the drivers or the main loop are indeed dead and to
        // activate whatever failsafe it thinks may help if
        // need be.  We assume the failsafe code can't
        // block. If it does then we will recurse and die when
        // we run out of stack
        if (_failsafe != nullptr) {
            _failsafe();
        }
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void CLCoreUrusScheduler_Cygwin::_run_io_procs(bool called_from_isr)
{

    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    _in_io_proc = false;

    if (!_urus_core.scheduler->get_timer_event_eval()) {
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartA)->_timer_tick();
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartB)->_timer_tick();
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartC)->_timer_tick();
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartD)->_timer_tick();
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartE)->_timer_tick();
        CLCoreUrusUARTDriver_Cygwin::from(hal.uartF)->_timer_tick();
    }

}

/*
  set simulation timestamp
 */
void CLCoreUrusScheduler_Cygwin::stop_clock(uint64_t time_usec)
{
    _stopped_clock_usec = time_usec;
    _run_io_procs(false);
}

#endif // __CYGWIN__
