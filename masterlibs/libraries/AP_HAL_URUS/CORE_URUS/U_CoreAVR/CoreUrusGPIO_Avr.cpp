
#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)

#include "../CORE_URUS_NAMESPACE.h"
#include "../CoreUrusGPIO.h"
#include "CoreUrusGPIO_Avr.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "utility/pins_arduino_mega.h"

#define analogInPinToBit(P) (P)


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

extern const AP_HAL::HAL& hal;

AP_HAL::Proc CLCoreUrusGPIO_Avr::_interrupt_6[AVR_INT_NUM_PINS_MAX] = {nullptr};
static volatile uint8_t PCintLast = 0;
static volatile bool PCintInProc = false;

#if defined(SHAL_CORE_APM2)
SIGNAL(INT6_vect) {
    if (CLCoreUrusGPIO_Avr::_interrupt_6[0]) {
        CLCoreUrusGPIO_Avr::_interrupt_6[0]();
    }
}

ISR(PCINT2_vect) {

    if (PCintInProc) {
        return;
    }

    PCintInProc = true;
	uint8_t mask;
	uint8_t pin;

	pin = PINK;                 // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;     // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;            // we memorize the current state of all PINs.

	if (mask & (1 << PCINT16)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[1]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[1]();
        }
	}

	if (mask & (1 << PCINT17)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[2]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[2]();
        }
	}

	if (mask & (1 << PCINT18)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[3]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[3]();
        }
	}

	if (mask & (1 << PCINT19)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[4]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[4]();
        }
	}

	if (mask & (1 << PCINT20)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[5]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[5]();
        }
	}

	if (mask & (1 << PCINT21)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[6]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[6]();
        }
	}

	if (mask & (1 << PCINT22)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[7]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[7]();
        }
	}

	if (mask & (1 << PCINT23)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[8]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[8]();
        }
	}

    PCintInProc = false;
}

#elif defined(SHAL_CORE_APM328)
ISR(PCINT2_vect) {
    if (PCintInProc) {
        return;
    }

    PCintInProc = true;
	uint8_t mask;
	uint8_t pin;

	pin = PIND;                 // PIND indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;     // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;            // we memorize the current state of all PINs.

	if (mask & (1 << PCINT18)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[0]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[0]();
        }
	}

	if (mask & (1 << PCINT19)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[1]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[1]();
        }
	}

	if (mask & (1 << PCINT20)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[2]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[2]();
        }
	}

	if (mask & (1 << PCINT21)) {
        if (CLCoreUrusGPIO_Avr::_interrupt_6[3]) {
            CLCoreUrusGPIO_Avr::_interrupt_6[3]();
        }
	}
    PCintInProc = false;
}
#endif

//CLCoreUrusGPIO_Avr Constructor
CLCoreUrusGPIO_Avr::CLCoreUrusGPIO_Avr()
{}

void CLCoreUrusGPIO_Avr::init()
{}

void CLCoreUrusGPIO_Avr::pinMode(uint8_t pin, uint8_t output)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg;

    if (port == NOT_A_PIN) return;

    // JWS: can I let the optimizer do this?
    reg = portModeRegister(port);

    if (output == HAL_GPIO_INPUT)
    {
        uint8_t oldSREG = SREG;
        cli();
        *reg &= ~bit;
        SREG = oldSREG;
    }
    else
    {
        uint8_t oldSREG = SREG;
        cli();
        *reg |= bit;
        SREG = oldSREG;
    }

}

int8_t CLCoreUrusGPIO_Avr::analogPinToDigitalPin(uint8_t pin)
{
    return analogInputToDigitalPin(pin);
}

uint8_t CLCoreUrusGPIO_Avr::read(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN) return 0;

    if (*portInputRegister(port) & bit) return 1;
    return 0;
}

void CLCoreUrusGPIO_Avr::write(uint8_t pin, uint8_t value)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (value == 0)
    {
        *out &= ~bit;
    }
    else
    {
        *out |= bit;
    }

    SREG = oldSREG;
}

void CLCoreUrusGPIO_Avr::toggle(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    *out ^= bit;

    SREG = oldSREG;
}

/* Alternative interface: */
AP_HAL::DigitalSource* CLCoreUrusGPIO_Avr::channel(uint16_t n)
{
    return new CLCoreDigitalSource_Avr(n);
}

/* Interrupt interface: */
bool CLCoreUrusGPIO_Avr::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode)
{
    /* Mode is to set the ISCn0 and ISCn1 bits.
     * These correspond to the GPIO_INTERRUPT_ defs in AP_HAL.h */
    if (!((mode == HAL_GPIO_INTERRUPT_LOW)||
          (mode == HAL_GPIO_INTERRUPT_HIGH)||
          (mode == HAL_GPIO_INTERRUPT_FALLING)||
          (mode == HAL_GPIO_INTERRUPT_RISING))) return false;
#if defined(SHAL_CORE_APM2)
    bool ret = false;
    uint8_t oldSREG = SREG;
    cli();
    switch (interrupt_num)
    {
    case 6:
            _interrupt_6[0] = p;
            /* Set the ISC60 and ICS61 bits in EICRB according to the value
             * of mode. */
            EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
            EIMSK |= (1 << INT6);
            ret = true;
            break;
    case GPIO_INT1:
            _interrupt_6[1] = p;
            PORTK   |= (1 << PCINT16);
            PCMSK2  |= (1 << PCINT16);
            ret = true;
            break;
    case GPIO_INT2:
            _interrupt_6[2] = p;
            PORTK   |= (1 << PCINT17);
            PCMSK2  |= (1 << PCINT17);
            ret = true;
            break;
    case GPIO_INT3:
            _interrupt_6[3] = p;
            PORTK   |= (1 << PCINT18);
            PCMSK2  |= (1 << PCINT18);
            ret = true;
            break;
    case GPIO_INT4:
            _interrupt_6[4] = p;
            PORTK   |= (1 << PCINT19);
            PCMSK2  |= (1 << PCINT19);
            ret = true;
            break;
    case GPIO_INT5:
            _interrupt_6[5] = p;
            PORTK   |= (1 << PCINT20);
            PCMSK2  |= (1 << PCINT20);
            ret = true;
            break;
    case GPIO_INT6:
            _interrupt_6[6] = p;
            PORTK   |= (1 << PCINT21);
            PCMSK2  |= (1 << PCINT21);
            ret = true;
            break;
    case GPIO_INT7:
            _interrupt_6[7] = p;
            PORTK   |= (1 << PCINT22);
            PCMSK2  |= (1 << PCINT22);
            ret = true;
            break;
    case GPIO_INT8:
            _interrupt_6[8] = p;
            PORTK   |= (1 << PCINT23);
            PCMSK2  |= (1 << PCINT23);
            ret = true;
            break;
    default:
        ret = false;
        break;
    }
    PCICR   |= (1 << PCIE2); // PCINT2 Interrupt enable
    SREG = oldSREG;

    return ret;
#elif defined(SHAL_CORE_APM328)
    if ((interrupt_num == 2) || (interrupt_num == 3) ||
        (interrupt_num == 4) || (interrupt_num == 5)){

        uint8_t oldSREG = SREG;
        cli();

        if (interrupt_num == 2) {
            _interrupt_6[0] = p;
            PORTD   |= (1 << PCINT18);
            PCMSK2  |= (1 << PCINT18);
        }

        if (interrupt_num == 3) {
            _interrupt_6[1] = p;
            PORTD   |= (1 << PCINT19);
            PCMSK2  |= (1 << PCINT19);
        }

        if (interrupt_num == 4) {
            _interrupt_6[2] = p;
            PORTD   |= (1 << PCINT20);
            PCMSK2  |= (1 << PCINT20);
        }

        if (interrupt_num == 5) {
            _interrupt_6[3] = p;
            PORTD   |= (1 << PCINT21);
            PCMSK2  |= (1 << PCINT21);
        }

        PCICR   |= (1 << PCIE2); // PCINT2 Interrupt enable
        SREG = oldSREG;
        return true;
    } else {
        return false;
    }
#endif
}

bool CLCoreUrusGPIO_Avr::usb_connected(void)
{
#if HAL_GPIO_USB_MUX_PIN != -1
    pinMode(HAL_GPIO_USB_MUX_PIN, HAL_GPIO_INPUT);
    return !read(HAL_GPIO_USB_MUX_PIN);
#else
    return false;
#endif
}

// CLCoreDigitalSource_Avr Constructor
CLCoreDigitalSource_Avr::CLCoreDigitalSource_Avr(uint8_t v) :
    _v(v)
{}

void CLCoreDigitalSource_Avr::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t CLCoreDigitalSource_Avr::read()
{
    return hal.gpio->read(_v);
}

void CLCoreDigitalSource_Avr::write(uint8_t value)
{
    return hal.gpio->write(_v,value);
}

void CLCoreDigitalSource_Avr::toggle()
{
    write(!read());
}

#endif // CONFIG_SHAL_CORE == SHAL_CORE_APM
