#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include <assert.h>

#include "AP_HAL_URUS.h"
#include "HAL_URUS_Class.h"
#include "CORE_URUS/CORE_URUS.h"

extern const NSCORE_URUS::CLCORE_URUS& _urus_core;

HAL_URUS::HAL_URUS() :
    AP_HAL::HAL(
        nullptr,  /* uartA */
        nullptr,  /* uartB */
        nullptr,  /* uartC */
        nullptr,  /* uartD */
        nullptr,  /* uartE */
        nullptr,  /* uartF */
        nullptr,
        nullptr, /* spi */
        nullptr, /* analogin */
        nullptr, /* storage */
        nullptr, /* console */
        nullptr, /* gpio */
        nullptr,  /* rcinput */
        nullptr, /* rcoutput */
        nullptr, /* scheduler */
        nullptr, /* util */
#ifndef HAL_MINIMIZE_FEATURES_AVR
        nullptr, /* onboard optical flow */
        nullptr) /* can */
#else
        nullptr) /* onboard optical flow */
#endif // HAL_MINIMIZE_FEATURES_AVR
{
    scheduler = NSCORE_URUS::get_scheduler();
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_MEGA02) \
    || defined(SHAL_CORE_APM328)) \
    || defined(SHAL_CORE_APM32U4)
#if CONFIG_SHAL_CORE_UARTDriverA == ENABLED
    uartA = NSCORE_URUS::get_uartA_Driver();
    console = NSCORE_URUS::get_uartA_Driver();
#endif // CONFIG_SHAL_CORE_UARTDriverA
#endif

#if CONFIG_SHAL_CORE == SHAL_CORE_APM && defined(SHAL_CORE_APM2) \
    || defined(SHAL_CORE_MEGA02) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_APM32U4)

    uartB = NSCORE_URUS::get_uartB_Driver();
#if !defined(SHAL_CORE_APM32U4)
    uartC = NSCORE_URUS::get_uartC_Driver();
#endif
#if CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN
    uartD = NSCORE_URUS::get_uartD_Driver();
    uartE = NSCORE_URUS::get_uartE_Driver();
    uartF = NSCORE_URUS::get_uartF_Driver();
#endif
#endif

#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_MEGA02) \
    || defined(SHAL_CORE_APM328))

#if CONFIG_SHAL_CORE_I2C == ENABLED
    i2c_mgr = NSCORE_URUS::get_I2CDeviceManager();
#endif // CONFIG_SHAL_CORE_I2C
#if CONFIG_SHAL_CORE_SPI == ENABLED
    spi = NSCORE_URUS::get_SPIDeviceManager();
#endif // CONFIG_SHAL_CORE_SPI
#if CONFIG_SHAL_CORE_ANALOGIN == ENABLED
    analogin = NSCORE_URUS::get_AnalogIn();
#endif // CONFIG_SHAL_CORE_ANALOGIN
#if CONFIG_SHAL_CORE_UTIL == ENABLED
    util = NSCORE_URUS::get_Util();
#endif
#if CONFIG_SHAL_CORE_STORAGE == ENABLED
    storage = NSCORE_URUS::get_Storage();
#endif // CONFIG_SHAL_CORE_STORAGE
#if CONFIG_SHAL_CORE_RCINPUT == ENABLED
    rcin = NSCORE_URUS::get_RCInput();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
#if CONFIG_SHAL_CORE_RCOUTPUT == ENABLED
    rcout = NSCORE_URUS::get_RCOutput();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
#endif
    gpio = NSCORE_URUS::get_GPIO();
}

void HAL_URUS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_MEGA02)

    assert(callbacks);

#endif

    _urus_core.init_core();

    gpio->init();

#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_MEGA02) \
    || defined(SHAL_CORE_APM328))

#if CONFIG_SHAL_CORE_RCOUTPUT == ENABLED
    rcout->init();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
#if CONFIG_SHAL_CORE_RCINPUT == ENABLED
    rcin->init();
#endif // CONFIG_SHAL_CORE_RCINPUT
#if CONFIG_SHAL_CORE_ANALOGIN == ENABLED
    analogin->init();
#endif

#endif
    callbacks->setup();

    for (;;) {
        callbacks->loop();
    }
}

void HAL_URUS::rcout_init()
{
#if CONFIG_SHAL_CORE_RCOUTPUT == ENABLED
#if !defined(SHAL_CORE_APM32U4)
    rcout = NSCORE_URUS::get_RCOutput();
    rcout->init();
#endif
#endif // CONFIG_SHAL_CORE_RCOUTPUT
}

void HAL_URUS::rcin_init()
{
#if CONFIG_SHAL_CORE_RCINPUT == ENABLED
#if !defined(SHAL_CORE_APM32U4)
    rcin = NSCORE_URUS::get_RCInput();
    rcin->init();
#endif
#endif // CONFIG_SHAL_CORE_RCINPUT
}

void HAL_URUS::analogin_init()
{
#if CONFIG_SHAL_CORE_ANALOGIN == ENABLED
#if !defined(SHAL_CORE_APM32U4)
    analogin = NSCORE_URUS::get_AnalogIn();
    analogin->init();
#endif
#endif // CONFIG_SHAL_CORE_ANALOGIN
}

void HAL_URUS::i2c_init()
{
#if CONFIG_SHAL_CORE_I2C == ENABLED
#if !defined(SHAL_CORE_APM32U4)
    util = NSCORE_URUS::get_Util();
    i2c_mgr = NSCORE_URUS::get_I2CDeviceManager();
#endif
#endif // CONFIG_SHAL_CORE_I2C
}

static const HAL_URUS shal;

void NSCORE_URUS::rcout_init()
{
#if CONFIG_SHAL_CORE_RCOUTPUT != ENABLED
    HAL_URUS &shaltmp = (HAL_URUS&)shal;
    shaltmp.rcout_init();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
}

void NSCORE_URUS::rcin_init()
{
#if CONFIG_SHAL_CORE_RCINPUT != ENABLED
    HAL_URUS &shaltmp = (HAL_URUS&)shal;
    shaltmp.rcin_init();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
}

void NSCORE_URUS::analogin_init()
{
#if CONFIG_SHAL_CORE_ANALOGIN != ENABLED
    HAL_URUS &shaltmp = (HAL_URUS&)shal;
    shaltmp.analogin_init();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
}

void NSCORE_URUS::i2c_init()
{
#if CONFIG_SHAL_CORE_I2C != ENABLED
    HAL_URUS &shaltmp = (HAL_URUS&)shal;
    shaltmp.i2c_init();
#endif // CONFIG_SHAL_CORE_RCOUTPUT
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return shal;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
