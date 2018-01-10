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
    || defined(SHAL_CORE_APM328))

    uartA = NSCORE_URUS::get_uartA_Driver();
    console = uartA;
#endif

#if CONFIG_SHAL_CORE == SHAL_CORE_APM && defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN

    uartB = NSCORE_URUS::get_uartB_Driver();
    uartC = NSCORE_URUS::get_uartC_Driver();
    uartD = NSCORE_URUS::get_uartD_Driver();
    uartE = NSCORE_URUS::get_uartE_Driver();
    uartF = NSCORE_URUS::get_uartF_Driver();

#endif
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_APM328))

    i2c_mgr = NSCORE_URUS::get_I2CDeviceManager();
    spi = NSCORE_URUS::get_SPIDeviceManager();
    analogin = NSCORE_URUS::get_AnalogIn();
    util = NSCORE_URUS::get_Util();
    storage = NSCORE_URUS::get_Storage();
    rcin = NSCORE_URUS::get_RCInput();
    rcout = NSCORE_URUS::get_RCOutput();
#endif
    gpio = NSCORE_URUS::get_GPIO();
}

void HAL_URUS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN

    assert(callbacks);

#endif

    _urus_core.init_core();

    gpio->init();
#if CONFIG_SHAL_CORE == SHAL_CORE_APM && (defined(SHAL_CORE_APM2) \
    || CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN \
    || defined(SHAL_CORE_APM328))

    rcout->init();
    rcin->init();

    analogin->init();

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
