#pragma once

#if (CONFIG_HAL_BOARD == HAL_BOARD_URUS) && (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM32U4)

#include "../CORE_URUS_NAMESPACE.h"
#include "../CoreUrusUARTDriver.h"

#include "USBAPI.h"

//================================================================================
//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class CLCoreUrusUsbUARTDriver_Avr : public NSCORE_URUS::CLCoreUrusUARTDriver
{
public:
	CLCoreUrusUsbUARTDriver_Avr() { peek_buffer = -1; };

    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) { peek_buffer = -1; };
	void end(void);

	void _timer_tick(void) override;

	uint32_t available(void);
    bool is_initialized() {
        return true;
    }

    void set_blocking_writes(bool blocking)
    {}

    bool tx_pending() {
        return false;
    }

    /* Implementations of Stream virtual methods */
    uint32_t txspace() override { return 0; };

	int read(void);
	int availableForWrite(void);
	void flush(void);
	size_t write(uint8_t);
	size_t write(const uint8_t*, size_t);

private:
	int peek_buffer;
};

#endif
