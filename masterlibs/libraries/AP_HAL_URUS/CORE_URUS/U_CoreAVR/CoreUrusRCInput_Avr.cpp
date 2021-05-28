
#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusScheduler.h"
#include "../CoreUrusRCInput.h"
#include "../CoreUrusTimers.h"

#include "CORE_URUS_AVR.h"
#include "CoreUrusScheduler_Avr.h"
#include "CoreUrusRCInput_Avr.h"
#include "CoreUrusTimers_Avr.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
#define  AVR_TIMER_OVF_VECT     TIMER5_OVF_vect
#define  AVR_TIMER_TCNT         TCNT5
#define  AVR_TIMER_TIFR         TIFR5
#define  AVR_TIMER_TCCRA        TCCR5A
#define  AVR_TIMER_TCCRB        TCCR5B
#define  AVR_TIMER_OCRA         OCR5A
#define  AVR_TIMER_TIMSK        TIMSK5
#define  AVR_TIMER_TOIE         TOIE5
#define  AVR_TIMER_WGM0         WGM50
#define  AVR_TIMER_WGM1         WGM51
#define  AVR_TIMER_WGM2         WGM52
#define  AVR_TIMER_WGM3         WGM53
#define  AVR_TIMER_CS1          CS51
#define  AVR_TIMER_ICR          ICR5
#define  AVR_TIMER_ICE          ICE5
#define  AVR_TIMER_ICES         ICES5
#define  AVR_TIMER_ICIE         ICIE1
#elif defined(SHAL_CORE_APM328)
#define  AVR_TIMER_OVF_VECT     TIMER1_OVF_vect
#define  AVR_TIMER_TCNT         TCNT1
#define  AVR_TIMER_TIFR         TIFR1
#define  AVR_TIMER_TCCRA        TCCR1A
#define  AVR_TIMER_TCCRB        TCCR1B
#define  AVR_TIMER_OCRA         OCR1A
#define  AVR_TIMER_TIMSK        TIMSK1
#define  AVR_TIMER_TOIE         TOIE1
#define  AVR_TIMER_WGM0         WGM10
#define  AVR_TIMER_WGM1         WGM11
#define  AVR_TIMER_WGM2         WGM12
#define  AVR_TIMER_WGM3         WGM13
#define  AVR_TIMER_CS1          CS11
#define  AVR_TIMER_ICR          ICR1
#define  AVR_TIMER_ICES         ICES1
#define  AVR_TIMER_ICIE         ICIE1
#endif

extern const AP_HAL::HAL& hal;

CLCoreUrusRCInput_Avr::CLCoreUrusRCInput_Avr() :
    NSCORE_URUS::CLCoreUrusRCInput()
{}

/* private variables to communicate with input capture isr */
volatile uint16_t CLCoreUrusRCInput_Avr::_pulse_capt[AVR_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t  CLCoreUrusRCInput_Avr::_num_channels = 0;
volatile bool     CLCoreUrusRCInput_Avr::_new_input = false;

/* private callback for input capture ISR */
void CLCoreUrusRCInput_Avr::_timer5_capt_cb(void) {
    static uint16_t icr5_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr5_current = AVR_TIMER_ICR;

    uint16_t pulse_width;
    if (icr5_current < icr5_prev) {
        /* ICR5 rolls over at TOP=40000 */
        pulse_width = icr5_current + 40000 - icr5_prev;
    } else {
        pulse_width = icr5_current - icr5_prev;
    }

    if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
        // sync pulse detected.  Pass through values if at least a minimum number of channels received
        if( channel_ctr >= AVR_RC_INPUT_MIN_CHANNELS ) {
            _num_channels = channel_ctr;
            _new_input = true;
        }
        channel_ctr = 0;
    } else {
        if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
                _num_channels = AVR_RC_INPUT_NUM_CHANNELS;
                _new_input = true;
            }
        }
    }
    icr5_prev = icr5_current;
}

void CLCoreUrusRCInput_Avr::init() {
    ISRRegistry& isrregistry = CORE_AVR::isrregistry;
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    isrregistry.register_signal(ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb);
#elif defined(SHAL_CORE_APM328)
    isrregistry.register_signal(ISR_REGISTRY_TIMER1_CAPT, _timer5_capt_cb);
#endif

    /* initialize overrides */
    clear_overrides();
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    /* Arduino pin 48 is ICP5 / PL1,  timer 5 input capture */
    hal.gpio->pinMode(48, HAL_GPIO_INPUT);
#else
    /* Arduino pin 8 is ICP1 / PB0,  timer 1 input capture */
    hal.gpio->pinMode(8, HAL_GPIO_INPUT);
#endif
    /**
     * WGM: 1 1 1 1. Fast WPM, TOP is in OCR5A (APM2) OCR1A (328P)
     * COM all disabled
     * CS51: prescale by 8 => 0.5us tick
     * ICES5: input capture on rising edge
     * OCR5A: 40000, 0.5us tick => 2ms period / 50hz freq for outbound
     * fast PWM.
     */

    uint8_t oldSREG = SREG;
    cli();

    /* Set timer 8x prescaler fast PWM mode toggle compare at OCRA with rising edge input capture */
    AVR_TIMER_TCCRB |= _BV(AVR_TIMER_WGM3) | _BV(AVR_TIMER_WGM2) | _BV(AVR_TIMER_CS1) | _BV(AVR_TIMER_ICES);

    /* Enable input capture interrupt */
    AVR_TIMER_TIMSK |= _BV(AVR_TIMER_ICIE);

    SREG = oldSREG;
}

bool CLCoreUrusRCInput_Avr::new_input()
{
    if (_new_input) {
        _new_input = false;
        return true;
    }
    return false;
}

uint8_t CLCoreUrusRCInput_Avr::num_channels() { return _num_channels; }

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}


uint16_t CLCoreUrusRCInput_Avr::read(uint8_t ch) {
    /* constrain ch */
    if (ch >= AVR_RC_INPUT_NUM_CHANNELS) return 0;
    /* grab channel from isr's memory in critical section*/
    uint8_t oldSREG = SREG;
    cli();
    uint16_t capt = _pulse_capt[ch];
    SREG = oldSREG;
    /* scale _pulse_capt from 0.5us units to 1us units. */
    uint16_t pulse = constrain_pulse(capt >> 1);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t CLCoreUrusRCInput_Avr::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > AVR_RC_INPUT_NUM_CHANNELS) { len = AVR_RC_INPUT_NUM_CHANNELS; }
    /* grab channels from isr's memory in critical section */
    uint8_t oldSREG = SREG;
    cli();
    for (int i = 0; i < len; i++) {
        periods[i] = _pulse_capt[i];
    }
    SREG = oldSREG;
    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (int i = 0; i < len; i++) {
        /* scale _pulse_capt from 0.5us units to 1us units. */
        periods[i] = constrain_pulse(periods[i] >> 1);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    return _num_channels;
}

bool CLCoreUrusRCInput_Avr::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool CLCoreUrusRCInput_Avr::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < AVR_RC_INPUT_NUM_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            _new_input = true;
            return true;
        }
    }
    return false;
}

void CLCoreUrusRCInput_Avr::clear_overrides() {
    for (int i = 0; i < AVR_RC_INPUT_NUM_CHANNELS; i++) {
        _override[i] = 0;
    }
}

#endif // CONFIG_SHAL_CORE == SHAL_CORE_APM
