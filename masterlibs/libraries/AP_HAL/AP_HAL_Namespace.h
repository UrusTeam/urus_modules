#pragma once

#include "string.h"
#include "utility/functor.h"

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDevice;
    class I2CDeviceManager;
    class Device;

    class SPIDevice;
    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    class OpticalFlow;
#ifndef HAL_MINIMIZE_FEATURES_AVR
    class CANManager;
    class CAN;
#endif // HAL_MINIMIZE_FEATURES_AVR
    class Util;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Member Procedure)

       For member functions we use the FastDelegate delegates class
       which allows us to encapculate a member function as a type
     */
    typedef void(*Proc)(void);
    FUNCTOR_TYPEDEF(MemberProc, void);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDeviceType {
        // Devices using AP_HAL::SPIDevice abstraction
        SPIDevice_Type              = -1,
    };

    // Must be implemented by the concrete HALs.
    const HAL& get_HAL();
}
