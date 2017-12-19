#pragma once

#include <stdint.h>

namespace NSCORE_URUS {

    class CLCORE_URUS;
    class CLCoreUrusTimers;
    class CLCoreUrusScheduler;
    class CLCoreUrusUARTDriver;
    class CLCoreUrusI2CDevice;
    class CLCoreUrusI2CDeviceManager;
    class CLCoreUrusSPIDevice;
    class CLCoreUrusSPIDeviceManager;
    class CLCoreUrusSemaphore;
    class CLCoreUrusAnalogSource;
    class CLCoreUrusAnalogIn;
    class CLCoreUrusUtil;
    class CLCoreUrusEEStorage;
    class CLCoreUrusGPIO;
    class CLCoreUrusDigitalSource;
    class CLCoreUrusRCInput;
    class CLCoreUrusRCOutput;

    const CLCORE_URUS& get_CORE();
    CLCoreUrusScheduler* get_scheduler();
    CLCoreUrusUARTDriver* get_uartA_Driver();
    CLCoreUrusUARTDriver* get_uartB_Driver();
    CLCoreUrusUARTDriver* get_uartC_Driver();
    CLCoreUrusUARTDriver* get_uartD_Driver();
    CLCoreUrusUARTDriver* get_uartE_Driver();
    CLCoreUrusUARTDriver* get_uartF_Driver();
    CLCoreUrusI2CDeviceManager* get_I2CDeviceManager();
    CLCoreUrusSPIDeviceManager* get_SPIDeviceManager();
    CLCoreUrusAnalogSource* get_AnalogSource();
    CLCoreUrusAnalogIn* get_AnalogIn();
    CLCoreUrusUtil* get_Util();
    CLCoreUrusEEStorage* get_Storage();
    CLCoreUrusGPIO* get_GPIO();
    CLCoreUrusRCInput* get_RCInput();
    CLCoreUrusRCOutput* get_RCOutput();

}
