#pragma once

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "../CORE_URUS_NAMESPACE.h"
#include "../CoreUrusRCOutput.h"

class CLCoreUrusRCOutput_Avr : public NSCORE_URUS::CLCoreUrusRCOutput {
public:
    CLCoreUrusRCOutput_Avr();
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);
    void write(uint8_t ch, uint16_t period_us);
    void write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void read(uint16_t* period_us, uint8_t len);

#if defined(SHAL_CORE_APM328)
    static volatile uint16_t tick_freq;
    static volatile uint16_t pwm_dat_chan_1;
    static volatile uint16_t pwm_dat_chan_2;
    static volatile uint16_t pwm_dat_chan_3;
    static volatile uint16_t pwm_dat_chan_4;
    static volatile uint16_t pwm_cnt;
#endif // defined

private:
    uint16_t _timer_period(uint16_t speed_hz);
#if defined(SHAL_CORE_APM328)
    uint8_t chans_status;
#endif // defined
};

#endif // __SHAL_CORE_APM__
