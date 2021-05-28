
#include "CoreUrusUsbUARTDriver_Avr.h"

#if (CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM32U4)

void CLCoreUrusUsbUARTDriver_Avr::begin(uint32_t b)
{
    usb_dev.attach();
	peek_buffer = -1;
	begin(b, 0, 0);
}

void CLCoreUrusUsbUARTDriver_Avr::end(void)
{}

void CLCoreUrusUsbUARTDriver_Avr::_timer_tick(void)
{}

uint32_t CLCoreUrusUsbUARTDriver_Avr::available(void)
{
	if (peek_buffer >= 0) {
		return 1 + USB_Available(CDC_RX);
	}
	return USB_Available(CDC_RX);
}

int CLCoreUrusUsbUARTDriver_Avr::read(void)
{
	if (peek_buffer >= 0) {
		int c = peek_buffer;
		peek_buffer = -1;
		return c;
	}
	return USB_Recv(CDC_RX);
}

int CLCoreUrusUsbUARTDriver_Avr::availableForWrite(void)
{
	return USB_SendSpace(CDC_TX);
}

void CLCoreUrusUsbUARTDriver_Avr::flush(void)
{
	USB_Flush(CDC_TX);
}

size_t CLCoreUrusUsbUARTDriver_Avr::write(uint8_t c)
{
	return write(&c, 1);
}

size_t CLCoreUrusUsbUARTDriver_Avr::write(const uint8_t *buffer, size_t size)
{
	if (availableForWrite() <= 0) {
        return size;
	}

	/* only try to send bytes if the high-level CDC connection itself
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */

	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)

	if (_usbLineInfo.lineState > 0)	{
		int r = USB_Send(CDC_TX, buffer, size);
		if (r > 0) {
			return r;
		} else {
			return 0;
		}
	}

	return 0;
}

#endif
