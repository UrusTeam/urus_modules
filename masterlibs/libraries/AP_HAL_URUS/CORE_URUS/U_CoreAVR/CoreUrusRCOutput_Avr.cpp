
#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "../CORE_URUS_NAMESPACE.h"
#include "CoreUrusRCOutput_Avr.h"

#include <avr/interrupt.h>

#if defined(SHAL_CORE_APM328)
#define HIGH_GPIO(pin)  PORTD |= 1<<pin
#define LOW_GPIO(pin)   PORTD &= ~(1<<pin)

#define CPU_DIV_8       8
#define CPU_DIV_64      64
#define CPU_DIV_256     256
#define CPU_DIV_1024    1024

#define CPU_DIVIDER     CPU_DIV_8
#define TIMER_RESOLUTION_FACTOR 0x13
#define TIMER_SPEED_US  (((1000 / ((F_CPU / 1000000) / CPU_DIVIDER)) * TIMER_RESOLUTION_FACTOR) / 1000)

#define HZ_TO_US(hz)    1000000UL / hz
#define DEFAULT_HZ      50

static volatile uint16_t tick_freq = 0;
static volatile uint16_t pwm_dat_chan_1 = 0;
static volatile uint16_t pwm_dat_chan_2 = 0;
static volatile uint16_t pwm_dat_chan_3 = 0;
static volatile uint16_t pwm_dat_chan_4 = 0;
static volatile uint16_t pwm_cnt = 0;

#endif

extern const AP_HAL::HAL& hal;

CLCoreUrusRCOutput_Avr::CLCoreUrusRCOutput_Avr() :
    NSCORE_URUS::CLCoreUrusRCOutput()
{}

void CLCoreUrusRCOutput_Avr::init()
{
    uint8_t oldSREGinit = SREG;
    cli();

#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    hal.gpio->pinMode(12,HAL_GPIO_OUTPUT); // CH_1 (PB6/OC1B)
    hal.gpio->pinMode(11,HAL_GPIO_OUTPUT); // CH_2 (PB5/OC1A)

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick
    TCCR1A =((1<<WGM11));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = 40000; // 0.5us tick => 50hz freq
    OCR1A = 0xFFFF; // Init OCR registers to nil output signal
    OCR1B = 0xFFFF;

    // --------------- TIMER4: CH_3, CH_4, and CH_5 ---------------------
    hal.gpio->pinMode(8,HAL_GPIO_OUTPUT); // CH_3 (PH5/OC4C)
    hal.gpio->pinMode(7,HAL_GPIO_OUTPUT); // CH_4 (PH4/OC4B)
    hal.gpio->pinMode(6,HAL_GPIO_OUTPUT); // CH_5 (PH3/OC4A)

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
    // CS41: prescale by 8 => 0.5us tick
    TCCR4A =((1<<WGM41));
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    OCR4A = 0xFFFF; // Init OCR registers to nil output signal
    OCR4B = 0xFFFF;
    OCR4C = 0xFFFF;
    ICR4 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER3: CH_6, CH_7, and CH_8 ----------------------
    hal.gpio->pinMode(3,HAL_GPIO_OUTPUT); // CH_6 (PE5/OC3C)
    hal.gpio->pinMode(2,HAL_GPIO_OUTPUT); // CH_7 (PE4/OC3B)
    hal.gpio->pinMode(5,HAL_GPIO_OUTPUT); // CH_8 (PE3/OC3A)

    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
    // CS31: prescale by 8 => 0.5us tick
    TCCR3A =((1<<WGM31));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    OCR3A = 0xFFFF; // Init OCR registers to nil output signal
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    ICR3 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER5: CH_10, and CH_11 ---------------
    // NB TIMER5 is shared with PPM input from RCInput_APM2.cpp
    // The TIMER5 registers are assumed to be setup already.
    hal.gpio->pinMode(45, HAL_GPIO_OUTPUT); // CH_10 (PL4/OC5B)
    hal.gpio->pinMode(44, HAL_GPIO_OUTPUT); // CH_11 (PL5/OC5C)
#elif defined(SHAL_CORE_APM328)
    TCCR0A = (1<<WGM01);
#if CPU_DIVIDER == CPU_DIV_8
    TCCR0B = (1<<CS01);
#endif // CPU_DIVIDER

    OCR0A = TIMER_RESOLUTION_FACTOR; // Init OCR registers

    TIFR0 = _BV(TOV0) | _BV(OCF0B) | _BV(OCF0A);      //Clear pending interrupts
    TIMSK0 = _BV(OCIE0A);    //Enable overflow interrupt

    // --------------- TIMER1: CH_5 ---------------------
    hal.gpio->pinMode(10,HAL_GPIO_OUTPUT); // CH_5 (PB2/OC1B)

    tick_freq = (uint16_t)(HZ_TO_US(DEFAULT_HZ) / TIMER_SPEED_US);
#endif
    SREG = oldSREGinit;
    sei();
}

#if defined(SHAL_CORE_APM328)
ISR(TIMER0_COMPA_vect)
{
    if (pwm_cnt >= tick_freq) {
        pwm_cnt = 0;
    }
    if (pwm_dat_chan_1 > 0) {
        pwm_cnt < pwm_dat_chan_1 ? HIGH_GPIO(2) : LOW_GPIO(2);
    }

    if (pwm_dat_chan_2 > 0) {
        pwm_cnt < pwm_dat_chan_2 ? HIGH_GPIO(3) : LOW_GPIO(3);
    }

    if (pwm_dat_chan_3 > 0) {
        pwm_cnt < pwm_dat_chan_3 ? HIGH_GPIO(4) : LOW_GPIO(4);
    }

    if (pwm_dat_chan_4 > 0) {
        pwm_cnt < pwm_dat_chan_4 ? HIGH_GPIO(5) : LOW_GPIO(5);
    }

    pwm_cnt++;
}
#endif // defined

void CLCoreUrusRCOutput_Avr::set_freq(uint32_t chmask, uint16_t freq_hz)
{
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    uint16_t icr = _timer_period(freq_hz);
    if ((chmask & ( _BV(CH_1) | _BV(CH_2))) != 0) {
        ICR1 = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_5))) != 0) {
        ICR4 = icr;
    }

    if ((chmask & ( _BV(CH_6) | _BV(CH_7) | _BV(CH_8))) != 0) {
        ICR3 = icr;
    }
#elif defined(SHAL_CORE_APM328)
    if ((chmask & (_BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4))) != 0) {
        tick_freq = (uint16_t)(HZ_TO_US(freq_hz) / TIMER_SPEED_US);
    }
#endif
}

uint16_t CLCoreUrusRCOutput_Avr::get_freq(uint8_t ch)
{
    uint16_t icr;
    switch (ch) {
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
        case CH_1:
        case CH_2:
            icr = ICR1;
            break;
        case CH_3:
        case CH_4:
        case CH_5:
            icr = ICR4;
            break;
        case CH_6:
        case CH_7:
        case CH_8:
            icr = ICR3;
            break;
        /* CH_10 and CH_11 share TIMER5 with input capture.
         * The period is specified in OCR5A rater than the ICR. */
        case CH_10:
        case CH_11:
            icr = OCR5A;
            break;
#elif defined(SHAL_CORE_APM328)
        case CH_1:
        case CH_2:
        case CH_3:
        case CH_4:
            icr = (tick_freq * TIMER_SPEED_US) << 1;
            break;
        /* CH_5 share TIMER1 with input capture.
         * The period is specified in OCR1A rater than the ICR. */
        case CH_5:
            icr = OCR1A;
            break;
#endif
        default:
            return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
    return (2000000UL / icr);
}

void CLCoreUrusRCOutput_Avr::enable_ch(uint8_t ch)
{
    switch(ch) {
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    case 0: TCCR1A |= (1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A |= (1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A |= (1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A |= (1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A |= (1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A |= (1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A |= (1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A |= (1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A |= (1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A |= (1<<COM5C1); break; // CH_11 : OC5C
#elif defined(SHAL_CORE_APM328)
    case 0:
        hal.gpio->pinMode(2, HAL_GPIO_OUTPUT);  // CH_1
        chans_status |= _BV(CH_1);
        break;
    case 1:
        chans_status |= _BV(CH_2);
        hal.gpio->pinMode(3, HAL_GPIO_OUTPUT);  // CH_2
        break;
    case 2:
        chans_status |= _BV(CH_3);
        hal.gpio->pinMode(4, HAL_GPIO_OUTPUT);  // CH_3
        break;
    case 3:
        chans_status |= _BV(CH_4);
        hal.gpio->pinMode(5, HAL_GPIO_OUTPUT);  // CH_4
        break;
    case 4: TCCR1A |= (1<<COM1B1); break; // CH_5 : OC1B
#endif
    }
}

void CLCoreUrusRCOutput_Avr::disable_ch(uint8_t ch)
{
    switch(ch) {
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    case 0: TCCR1A &= ~(1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A &= ~(1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A &= ~(1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A &= ~(1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A &= ~(1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A &= ~(1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A &= ~(1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A &= ~(1<<COM3A1); break; // CH_8 : OC3A
    case 9: TCCR5A &= ~(1<<COM5B1); break; // CH_10 : OC5B
    case 10: TCCR5A &= ~(1<<COM5C1); break; // CH_11 : OC5C
#elif defined(SHAL_CORE_APM328)
    case 0:
        chans_status &= ~_BV(CH_1);
        pwm_dat_chan_1 = 0;
        LOW_GPIO(2);  // CH_1
        break;
    case 1:
        chans_status &= ~_BV(CH_2);
        pwm_dat_chan_2 = 0;
        LOW_GPIO(3);  // CH_2
        break;
    case 2:
        chans_status &= ~_BV(CH_3);
        pwm_dat_chan_3 = 0;
        LOW_GPIO(4);  // CH_3
        break;
    case 3:
        pwm_dat_chan_4 = 0;
        chans_status &= ~_BV(CH_4);
        LOW_GPIO(5);  // CH_4
        break;
    case 4: TCCR1A &= ~(1<<COM1B1); break; // CH_5 : OC1B
#endif
    }
}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_OUTPUT_MAX_PULSEWIDTH) return RC_OUTPUT_MAX_PULSEWIDTH;
    if (p < RC_OUTPUT_MIN_PULSEWIDTH) return RC_OUTPUT_MIN_PULSEWIDTH;
    return p;
}

void CLCoreUrusRCOutput_Avr::write(uint8_t ch, uint16_t period_us)
{
    /* constrain, then scale from 1us resolution (input units)
     * to 0.5us (timer units) */
    uint16_t pwm = constrain_period(period_us) << 1;
#if defined(SHAL_CORE_APM328)
    uint16_t pwm_dat = (pwm >> 1) / TIMER_SPEED_US;
#endif // defined
    switch(ch)
    {
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    case 0:  OCR1B=pwm; break;  // out1
    case 1:  OCR1A=pwm; break;  // out2
    case 2:  OCR4C=pwm; break;  // out3
    case 3:  OCR4B=pwm; break;  // out4
    case 4:  OCR4A=pwm; break;  // out5
    case 5:  OCR3C=pwm; break;  // out6
    case 6:  OCR3B=pwm; break;  // out7
    case 7:  OCR3A=pwm; break;  // out8
    case 9:  OCR5B=pwm; break;  // out10
    case 10: OCR5C=pwm; break;  // out11
#elif defined(SHAL_CORE_APM328)
    case 0:
        if (((chans_status >> CH_1) & 0x01) == 1) {
            pwm_dat_chan_1 = pwm_dat;
        }
        break;
    case 1:
        if (((chans_status >> CH_2) & 0x01) == 1) {
            pwm_dat_chan_2 = pwm_dat;
        }
        break;
    case 2:
        if (((chans_status >> CH_3) & 0x01) == 1) {
            pwm_dat_chan_3 = pwm_dat;
        }
        break;
    case 3:
        if (((chans_status >> CH_4) & 0x01) == 1) {
            pwm_dat_chan_4 = pwm_dat;
        }
        break;
    case 4:
        OCR1B = pwm;
        break;
#endif
    }
}

void CLCoreUrusRCOutput_Avr::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t CLCoreUrusRCOutput_Avr::read(uint8_t ch)
{
    uint16_t pwm=0;
    switch(ch) {
#if defined(SHAL_CORE_APM2) || defined(SHAL_CORE_MEGA02)
    case 0:  pwm=OCR1B; break;      // out1
    case 1:  pwm=OCR1A; break;      // out2
    case 2:  pwm=OCR4C; break;      // out3
    case 3:  pwm=OCR4B; break;      // out4
    case 4:  pwm=OCR4A; break;      // out5
    case 5:  pwm=OCR3C; break;      // out6
    case 6:  pwm=OCR3B; break;      // out7
    case 7:  pwm=OCR3A; break;      // out8
    case 9:  pwm=OCR5B; break;      // out10
    case 10: pwm=OCR5C; break;      // out11
#elif defined(SHAL_CORE_APM328)
    case 0:  pwm=(pwm_dat_chan_1 * TIMER_SPEED_US) << 1; break; // out1
    case 1:  pwm=(pwm_dat_chan_2 * TIMER_SPEED_US) << 1; break; // out2
    case 2:  pwm=(pwm_dat_chan_3 * TIMER_SPEED_US) << 1; break; // out3
    case 3:  pwm=(pwm_dat_chan_4 * TIMER_SPEED_US) << 1; break; // out4
    case 4:  pwm=OCR1B; break;                                  // out5
#endif
    }
    /* scale from 0.5us resolution (timer units) to 1us units */
    return pwm>>1;
}

void CLCoreUrusRCOutput_Avr::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

uint16_t CLCoreUrusRCOutput_Avr::_timer_period(uint16_t speed_hz)
{
    return 2000000UL / speed_hz;
}


#endif // __SHAL_CORE_APM__
