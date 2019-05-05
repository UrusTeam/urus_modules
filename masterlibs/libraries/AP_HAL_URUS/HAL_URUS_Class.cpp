#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include <assert.h>

#include "AP_HAL_URUS.h"
#include "HAL_URUS_Class.h"
#include "CORE_URUS/CORE_URUS.h"

static const NSCORE_URUS::CLCORE_URUS& _urus_core = NSCORE_URUS::get_CORE();

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
        nullptr, /* onboard optical flow */
        nullptr) /* can */
{
    scheduler = NSCORE_URUS::get_scheduler();
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_MEGA02) \
    || defined(SHAL_CORE_APM328))

    uartA = NSCORE_URUS::get_uartA_Driver();
    console = NSCORE_URUS::get_uartA_Driver();
#endif

#if CONFIG_SHAL_CORE == SHAL_CORE_APM && defined(SHAL_CORE_APM2) \
    || defined(SHAL_CORE_MEGA02) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN

    uartB = NSCORE_URUS::get_uartB_Driver();
    uartC = NSCORE_URUS::get_uartC_Driver();
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
    analogin = NSCORE_URUS::get_AnalogIn();
#if CONFIG_SHAL_CORE_UTIL == ENABLED
    util = NSCORE_URUS::get_Util();
#endif
    storage = NSCORE_URUS::get_Storage();
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

    uartA->printf_PS(PSTR("\n\n\n\n\n\n\nInit URUS System!\n\n"));

#endif
    callbacks->setup();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_URUS hal;
    return hal;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
