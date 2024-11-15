#include <AP_HAL_URUS/AP_HAL_URUS.h>

#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusScheduler.h"
#include "../CoreUrusUARTDriver.h"
#include "../CoreUrusTimers.h"
#include "../CoreUrusUtil.h"

#include "CORE_URUS_AVR.h"
#include "CoreUrusScheduler_Avr.h"
#include "CoreUrusUARTDriver_Avr.h"
#include "CoreUrusTimers_Avr.h"
#include "CoreUrusUtil_Avr.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

extern const AP_HAL::HAL& hal;

AP_HAL::Proc CLCoreUrusScheduler_Avr::_failsafe = nullptr;
volatile bool CLCoreUrusScheduler_Avr::_timer_suspended = false;
volatile bool CLCoreUrusScheduler_Avr::_timer_event_missed = false;
//volatile uint32_t CLCoreUrusScheduler_Avr::_start_micros = 0;

AP_HAL::MemberProc CLCoreUrusScheduler_Avr::_timer_proc[URUS_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t CLCoreUrusScheduler_Avr::_num_timer_procs = 0;
bool CLCoreUrusScheduler_Avr::_in_timer_proc = false;

volatile uint8_t CLCoreUrusScheduler_Avr::_timer_reset_value = (256 - 124);

CLCoreUrusScheduler_Avr::CLCoreUrusScheduler_Avr() :
    NSCORE_URUS::CLCoreUrusScheduler(),
    _min_delay_cb_ms(0x7FFF)
{}

bool CLCoreUrusScheduler_Avr::in_main_thread() const
{
    const bool ret = _in_timer_proc;
    return ret;
}

void CLCoreUrusScheduler_Avr::init()
{
    ISRRegistry& isrregistry = CORE_AVR::isrregistry;
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02) || defined(SHAL_CORE_APM328)
    CLCoreUrusUtil_Avr* coreUtil = (CLCoreUrusUtil_Avr*)NSCORE_URUS::get_Util();
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    clk_core_timers.clk_per_sec = F_CPU;
    start_sched();
#endif
#endif

    CLCoreUrusTimers_Avr::avr_timer.init();
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_APM328) || defined(SHAL_CORE_MEGA02)
    /* TIMER2: Setup the overflow interrupt to occur at 1khz. */
    TIMSK2 = 0;                     /* Disable timer interrupt */
    TCCR2A = 0;                     /* Normal counting mode */
    TCCR2B = _BV(CS21) | _BV(CS22); /* Prescaler to clk/256 */
    TCNT2 = _timer_reset_value;     /* Set count to timer reset value  */
    TIFR2 = _BV(TOV2);              /* Clear pending interrupts */
    /* Register _fire_isr_sched to trigger on overflow */
    isrregistry.register_signal(ISR_REGISTRY_TIMER2_OVF, _fire_isr_sched);
    TIMSK2 = _BV(TOIE2);            /* Enable overflow interrupt*/

    /* Turn on global interrupt flag, AVR interupt system will start from this point */
#elif defined(SHAL_CORE_APM16U) || defined(SHAL_CORE_APM32U4)
    /* TIMER0: Setup the overflow interrupt to occur at 1khz. */
    TIMSK0 = 0;                     /* Disable timer interrupt */
    TCCR0A = 0;                     /* Normal counting mode */
    TCCR0B = _BV(CS02); /* Prescaler to clk/256 */
    TCNT0 = _timer_reset_value;     /* Set count to timer reset value  */
    TIFR0 = _BV(TOV0);              /* Clear pending interrupts */
    TIMSK0 = _BV(TOIE0);            /* Enable overflow interrupt*/
    /* Register _fire_isr_sched to trigger on overflow */
    isrregistry.register_signal(ISR_REGISTRY_TIMER0_OVF, _fire_isr_sched);
    /* Turn on global interrupt flag, AVR interupt system will start from this point */
#endif
    sei();
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02) || defined(SHAL_CORE_APM328)
    coreUtil->memcheck_init();
#endif
}

void CLCoreUrusScheduler_Avr::_fire_isr_sched()
{
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_APM328) || defined(SHAL_CORE_MEGA02)
    TCNT2 = _timer_reset_value;
#elif defined(SHAL_CORE_APM16U) || defined(SHAL_CORE_APM32U4)
    TCNT0 = _timer_reset_value;
#endif
    sei();
    _run_timer_procs(true);
}

void CLCoreUrusScheduler_Avr::delay_microseconds(uint16_t usec)
{

    if (--usec == 0) {
        return;
    }

    uint32_t _start_micros = AP_HAL::micros();
    while ((AP_HAL::micros() - _start_micros) < usec);
}

void CLCoreUrusScheduler_Avr::delay(uint16_t ms)
{
/*
    if (_in_delay_proc) {
        return;
    }

    _in_delay_proc = true;
*/
    uint32_t now_micros = AP_HAL::micros();
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    uint32_t start = AP_HAL::millis();
    uint16_t dt_micros = 0;
    uint16_t centinel_micros = (uint16_t)URUS_MAGIC_TIME;
    uint16_t ms_cb = ms;

    while ((AP_HAL::millis() - start) < (uint32_t)ms) {
        dt_micros = (uint16_t)(AP_HAL::micros() - now_micros);
        now_micros = AP_HAL::micros();
        delay_microseconds(centinel_micros);

        if (dt_micros > centinel_micros) {
            centinel_micros = centinel_micros + (dt_micros - centinel_micros);
        }

        if (dt_micros < centinel_micros) {
            centinel_micros = centinel_micros - dt_micros;
        }

        ms_cb--;
        if (_min_delay_cb_ms < ms_cb) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
#else
    while (ms > 0) {
        while ((AP_HAL::micros() - now_micros) >= 1000) {
            ms--;
            if (ms == 0) break;
            now_micros += 1000;
        }
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
#endif
    //_in_delay_proc = false;
}

void CLCoreUrusScheduler_Avr::register_delay_callback(AP_HAL::Proc proc,
        uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void CLCoreUrusScheduler_Avr::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < URUS_SCHEDULER_MAX_TIMER_PROCS) {
        /* this write to _timer_proc can be outside the critical section
         * because that memory won't be used until _num_timer_procs is
         * incremented. */
        _timer_proc[_num_timer_procs] = proc;
        /* _num_timer_procs is used from interrupt, and multiple bytes long. */
        uint8_t sreg = SREG;
        cli();
        _num_timer_procs++;
        SREG = sreg;
    }
}

void CLCoreUrusScheduler_Avr::register_io_process(AP_HAL::MemberProc proc)
{}

void CLCoreUrusScheduler_Avr::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void CLCoreUrusScheduler_Avr::suspend_timer_procs()
{
    _timer_suspended = true;
}

void CLCoreUrusScheduler_Avr::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed) {
        _timer_event_missed = false;
        _run_timer_procs(false);
    }
}

void CLCoreUrusScheduler_Avr::system_initialized()
{
    if (_initialized) {
#if !defined(SHAL_CORE_APM16U) && !defined(SHAL_CORE_APM32U4)
#if !HAL_MINIMIZE_FEATURES_AVR
        AP_HAL::panic(
            PSTR("PANIC: scheduler system initialized called more than once"));
#endif
#endif
        _initialized = false;
        return;
    }

    _initialized = true;
}

void CLCoreUrusScheduler_Avr::reboot(bool hold_in_bootloader)
{
#if !defined(SHAL_CORE_APM16U) && !defined(SHAL_CORE_APM32U4)
#if !HAL_MINIMIZE_FEATURES_AVR
    hal.uartA->printf_PS(PSTR("GOING DOWN FOR A REBOOT\r\n"));
#endif
    hal.scheduler->delay(100);
#endif
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02) || defined(SHAL_CORE_APM16U) || defined(SHAL_CORE_APM32U4)
    /* The APM2 bootloader will reset the watchdog shortly after
     * starting, so we can use the watchdog to force a reboot
     */
    cli();
    wdt_enable(WDTO_15MS);
    for(;;);
#else
    cli();
    /* Making a null pointer call will cause all AVRs to reboot
     * but they may not come back alive properly - we need to setup
     * the IO the way the bootloader would.
     */
    void (*fn)(void) = NULL;
    fn();
    for(;;);
#endif
}

void CLCoreUrusScheduler_Avr::_run_timer_procs(bool called_from_isr)
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

#endif // CONFIG_SHAL_CORE == SHAL_CORE_APM
