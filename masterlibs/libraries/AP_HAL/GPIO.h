#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ALT    2
#define HAL_GPIO_INTERRUPT_LOW 0
#define HAL_GPIO_INTERRUPT_HIGH 1
#define HAL_GPIO_INTERRUPT_FALLING 2
#define HAL_GPIO_INTERRUPT_RISING 3

enum GPIO_INT {
    GPIO_INT1 = 8,
    GPIO_INT2,
    GPIO_INT3,
    GPIO_INT4,
    GPIO_INT5,
    GPIO_INT6,
    GPIO_INT7,
    GPIO_INT8
};

class AP_HAL::DigitalSource {
public:
    virtual void    mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void    write(uint8_t value) = 0;
    virtual void    toggle() = 0;
};

class AP_HAL::GPIO {
public:
    GPIO() {}

    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;

    // optional interface on some boards
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    toggle(uint8_t pin) = 0;
    virtual int8_t  analogPinToDigitalPin(uint8_t pin) = 0;

    virtual void    write_port(uint8_t portnr, uint8_t value) = 0;
    virtual uint8_t read_port(uint8_t portnr) = 0;

    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    /* Interrupt interface: */
    virtual bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode) = 0;

    /* return true if USB cable is connected */
    virtual bool    usb_connected(void) = 0;
};
