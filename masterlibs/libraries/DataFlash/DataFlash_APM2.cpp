/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       DataFlash_APM2.cpp - DataFlash log library for AT45DB321D
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *       This code works only on ATMega2560. It uses Serial port 3 in SPI MSPI mdoe.
 *
 *       Dataflash library for AT45DB321D flash memory
 *       Memory organization : 8192 pages of 512 bytes or 528 bytes
 *
 *       Maximun write bandwidth : 512 bytes in 14ms
 *       This code is written so the master never has to wait to write the data on the eeprom
 *
 *       Methods:
 *               Init() : Library initialization (SPI initialization)
 *               StartWrite(page) : Start a write session. page=start page.
 *               StartRead(page) : Start a read on (page)
 *               GetWritePage() : Returns the last page written to
 *               GetPage() : Returns the last page read
 *
 *       Properties:
 *
 */

#include <AP_HAL/AP_HAL.h>               // for removing conflict with optical flow sensor on SPI3 bus
#include "DataFlash_APM2.h"
#include <utility>

extern const AP_HAL::HAL& hal;
/*
 * #define ENABLE_FASTSERIAL_DEBUG
 *
 * #ifdef ENABLE_FASTSERIAL_DEBUG
 # define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
 ##else
 # define serialDebug(fmt, args...)
 ##endif
 #  //*/

#define DF_RESET 41             // RESET  (PG0)
#define DF_CARDDETECT 33        // PC4

// AT45DB321D Commands (from Datasheet)
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A

void DataFlash_APM2::cs_release()
{
    //hal.gpio->write(28,1);
    _spi->cs_release();
}

void DataFlash_APM2::cs_assert()
{
    _spi->cs_assert();
    //hal.gpio->write(28,0);
}

/*
  try to take a semaphore safely from both in a timer and outside
 */
bool DataFlash_APM2::_sem_take(uint8_t timeout)
{
    //if (hal.scheduler->in_main_thread()) {
        //return _spi_sem->take_nonblocking();
    //}
    return _spi_sem->take(timeout);
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM2::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Backend::Init(structure, num_types);
    // init to zero
    df_NumPages = 0;

    hal.gpio->pinMode(DF_RESET, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(DF_CARDDETECT, HAL_GPIO_INPUT);

    // Reset the chip
    hal.gpio->write(DF_RESET,0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET,1);

    _spi = hal.spi->get_device("FLASH_SPI3");

    hal.gpio->pinMode(28, HAL_GPIO_OUTPUT);
    hal.gpio->write(28,1);

    if (!_spi) {
        //AP_HAL::panic("PANIC: DataFlash SPIDeviceDriver not found");
        return;
    }
    _spi_sem = _spi->get_semaphore();
    if (!_spi_sem) {
        //AP_HAL::panic("PANIC: DataFlash SPIDeviceDriver semaphore is null");
        return; /* never reached */
    }

    if (!_sem_take(5))
        return;
    // get page size: 512 or 528  (by default: 528)
    df_PageSize=PageSize();

    ReadManufacturerID();

    _spi_sem->give();

    // see page 22 of the spec for the density code
    uint8_t density_code = (df_device >> 8) & 0x1F;

    // note that we set df_NumPages to one lower than the highest, as
    // the last page is reserved for a config page
    if (density_code == 0x7) {
        // 32 Mbit
        df_NumPages = 8191;
    } else if (density_code == 0x6) {
        // 16 Mbit
        df_NumPages = 4095;
    }

    //hal.console->printf("numpage: %u density: %u pagesize: %u\n", df_NumPages, density_code, df_PageSize);
    //serialDebug("density_code %d pages %d, size %d\n", density_code, df_NumPages, df_PageSize);
}

// This function is mainly to test the device
void DataFlash_APM2::ReadManufacturerID()
{
    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    // Read manufacturer and ID command...
    uint8_t data[1];
    uint8_t rcv_data[4] = {0xff, 0xff, 0xff, 0xff};
    data[0] = DF_READ_MANUFACTURER_AND_DEVICE_ID;
    _spi->transfer(data, 1, nullptr, 0);
    _spi->transfer(nullptr, 0, rcv_data, 4);
    //_register_write(DF_READ_MANUFACTURER_AND_DEVICE_ID, 0x00);

    //data[0] = 0xff;
    //df_manufacturer = _register_read(0xff);
    df_manufacturer = rcv_data[0];
    //df_device = _register_read(0xff);
    df_device = rcv_data[1];
    //df_device = (df_device<<8) | _register_read(0xff);
    df_device = (df_device<<8) | rcv_data[2];
    //_register_write(0xff, 0);
    //_register_write(0xff, 0);
    //hal.console->printf("manuf: 0x%x\n", df_manufacturer);

    // release SPI bus for use by other sensors
    //_spi->cs_release();
    cs_release();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_APM2::CardInserted()
{
    //serialDebug("df_NumPages %d, detect:%d\n", df_NumPages, tmp);
    //return (df_NumPages >= 4095 && digitalRead(DF_CARDDETECT) == 0);
    return (df_NumPages >= 4095);
}

// Read the status register
uint8_t DataFlash_APM2::ReadStatusReg()
{
    uint8_t tmp;

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    // Read status command
    uint8_t data[1];
    data[0] = DF_STATUS_REGISTER_READ;
    //_register_write(DF_STATUS_REGISTER_READ, 0x00);

    //data[1] = 0x00;
    uint8_t data_rcv[1];
    data_rcv[0] = 0x00;
    _spi->transfer(data, 1, nullptr, 0);      // We only want to extract the READY/BUSY bit
    _spi->transfer(nullptr, 0, data_rcv, 1);
    //tmp = _register_read(0x00);
    tmp = data_rcv[0];
    //tmp = data[0];

    // release SPI bus for use by other sensors
    //_spi->cs_release();
    cs_release();

    return tmp;
}

// Read the status of the DataFlash
inline
uint8_t DataFlash_APM2::ReadStatus()
{
    return(ReadStatusReg()&0x80);      // We only want to extract the READY/BUSY bit
}

inline
uint16_t DataFlash_APM2::PageSize()
{
    return(528-((ReadStatusReg()&0x01)<<4));      // if first bit 1 trhen 512 else 528 bytes
}

// Wait until DataFlash is in ready state...
void DataFlash_APM2::WaitReady()
{
    while(!ReadStatus()) ;
}

void DataFlash_APM2::PageToBuffer(uint8_t BufferNum, uint16_t PageAdr)
{
    if (!_sem_take(1))
        return;
    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    uint8_t cmd[4];
    cmd[0] = BufferNum?DF_TRANSFER_PAGE_TO_BUFFER_2:DF_TRANSFER_PAGE_TO_BUFFER_1;
    //cmd[1] = 0;
    cmd[3] = 0x00;

    if(df_PageSize==512) {
        cmd[1] = (uint8_t)(PageAdr >> 7);
        cmd[2] = (uint8_t)(PageAdr << 1);
    }else{
        cmd[1] = (uint8_t)(PageAdr >> 6);
        cmd[2] = (uint8_t)(PageAdr << 2);
    }
    //cmd[3] = 0;
    _spi->transfer(cmd, sizeof(cmd), nullptr, 0);

    //initiate the transfer
    //_spi->cs_release();
    cs_release();
    cs_assert();
    //_spi->cs_assert();

    while(!ReadStatus()) ;     //monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    //_spi->cs_release();
    cs_release();
    _spi_sem->give();
}

void DataFlash_APM2::BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait)
{
    if (!_sem_take(1))
        return;
    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    uint8_t cmd[4];
    cmd[0] = BufferNum?DF_BUFFER_2_TO_PAGE_WITH_ERASE:DF_BUFFER_1_TO_PAGE_WITH_ERASE;
    //cmd[1] = 0;
    cmd[3] = 0x00;

    if(df_PageSize==512) {
        cmd[1] = (uint8_t)(PageAdr >> 7);
        cmd[2] = (uint8_t)(PageAdr << 1);
    }else{
        cmd[1] = (uint8_t)(PageAdr >> 6);
        cmd[2] = (uint8_t)(PageAdr << 2);
    }
    //cmd[3] = 0;
    _spi->transfer(cmd, sizeof(cmd), nullptr, 0);

    //initiate the transfer
    //_spi->cs_release();
    cs_release();

    // Check if we need to wait to write the buffer to memory or we can continue...
    if (wait)
        while(!ReadStatus());
            //hal.scheduler->delay(1);//monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    _spi_sem->give();
}

void DataFlash_APM2::BlockWrite (uint8_t BufferNum, uint16_t IntPageAdr,
                                 const void *pHeader, uint8_t hdr_size,
                                 const void *pBuffer, uint16_t size)
{
    if (!_sem_take(1))
        return;

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    uint8_t cmd[] = {
        (uint8_t)(BufferNum?DF_BUFFER_2_WRITE:DF_BUFFER_1_WRITE),
        0,
        (uint8_t)(IntPageAdr>>8),
        (uint8_t)(IntPageAdr)
    };
    _spi->transfer(cmd, sizeof(cmd), nullptr, 0);

    // transfer header, if any
    if (hdr_size != 0) {
        _spi->transfer((const uint8_t *)pHeader, hdr_size, nullptr, 0);
    }

    // transfer data
    _spi->transfer((const uint8_t *)pBuffer, size, nullptr, 0);

    //while(!ReadStatus())
        //hal.scheduler->delay(1);
    // release SPI bus for use by other sensors
    //_spi->cs_release();
    cs_release();
    _spi_sem->give();
}

bool DataFlash_APM2::BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size)
{
    if (!_sem_take(1))
        return false;

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    uint8_t data[5];
    data[1] = 0x00;
    data[2] = (uint8_t)(IntPageAdr>>8);
    data[3] = (uint8_t)(IntPageAdr);
    data[4] = 0x00;

    if (BufferNum==0) {
        data[0] = DF_BUFFER_1_READ;
        _spi->transfer(data, 5, nullptr, 0);
    } else {
        data[0] = DF_BUFFER_2_READ;
        _spi->transfer(data, 5, nullptr, 0);
    }

    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);
    //data[0] = (uint8_t)(IntPageAdr>>8);
    //_spi->transfer(data, 1, nullptr, 0);       //upper part of internal buffer address
    //data[0] = (uint8_t)(IntPageAdr);
    //_spi->transfer(data, 1, nullptr, 0);                  //lower part of internal buffer address
    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);                                                                 //don't cares
    uint8_t *pData = (uint8_t *)pBuffer;
    //memset(pData, 0, size);
    uint8_t val;

    while (size--) {
        //data[0] = 0x00;
        val = 0x00;
        _spi->transfer(nullptr, 0, &val, 1);
        *pData++ = val;
    }
    //while(!ReadStatus());
    // release SPI bus for use by other sensors
    //_spi->cs_release();
    cs_release();

    _spi_sem->give();
    return true;
}

// *** END OF INTERNAL FUNCTIONS ***
void DataFlash_APM2::PageErase (uint16_t PageAdr)
{
    if (!_sem_take(1))
        return;

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    // Send page erase command
    uint8_t data[4];
    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);

    data[0] = DF_PAGE_ERASE;
    //data[1] = 0x00;
    data[3] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);

    if(df_PageSize==512) {
        data[1] = (uint8_t)(PageAdr >> 7);
        //_spi->transfer(data, 1, nullptr, 0);
        data[2] = (uint8_t)(PageAdr << 1);
        //data[3] = 0x00;
        _spi->transfer(data, 4, nullptr, 0);
    }else{
        data[1] = (uint8_t)(PageAdr >> 6);
        //_spi->transfer(data, 1, nullptr, 0);
        data[2] = (uint8_t)(PageAdr << 2);
        //data[3] = 0x00;
        _spi->transfer(data, 4, nullptr, 0);
    }

    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);

    //initiate flash page erase
    //_spi->cs_release();
    cs_release();
    while(!ReadStatus());
        //hal.scheduler->delay(1);

    // release SPI bus for use by other sensors
    _spi_sem->give();
}

// erase a block of 8 pages.
void DataFlash_APM2::BlockErase(uint16_t BlockAdr)
{
    if (!_sem_take(1))
        return;

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    // Send block erase command

    uint8_t data[4];
    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);

    data[0] = DF_BLOCK_ERASE;
    //data[1] = 0x00;
    data[3] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);

    if (df_PageSize==512) {
        data[1] = (uint8_t)(BlockAdr >> 4);
        //_spi->transfer(data, 1, nullptr, 0);
        data[2] = (uint8_t)(BlockAdr << 4);
        //data[1] = 0x00;
        _spi->transfer(data, 4, nullptr, 0);
    } else {
        data[1] = (uint8_t)(BlockAdr >> 3);
        //_spi->transfer(data, 1, nullptr, 0);
        data[2] = (uint8_t)(BlockAdr << 5);
        //data[1] = 0x00;
        _spi->transfer(data, 4, nullptr, 0);
    }
    //data[0] = 0x00;
    //_spi->transfer(data, 1, nullptr, 0);
    //serialDebug("BL Erase, %d\n", BlockAdr);

    //initiate flash page erase
    //_spi->cs_release();
    cs_release();
    while(!ReadStatus());

    // release SPI bus for use by other sensors
    _spi_sem->give();
}

void DataFlash_APM2::ChipErase()
{
    if (!_sem_take(1))
        return;

    //serialDebug("Chip Erase\n");

    // activate dataflash command decoder
    //_spi->cs_assert();
    cs_assert();

    // opcodes for chip erase
    uint8_t data[4];
    data[0] = DF_CHIP_ERASE_0;
    //_spi->transfer(data, 1, nullptr, 0);
    data[1] = DF_CHIP_ERASE_1;
    //_spi->transfer(data, 1, nullptr, 0);
    data[2] = DF_CHIP_ERASE_2;
    //_spi->transfer(data, 1, nullptr, 0);
    data[3] = DF_CHIP_ERASE_3;
    _spi->transfer(data, 4, nullptr, 0);

    //initiate flash page erase
    //_spi->cs_release();
    cs_release();

    while(!ReadStatus()) {
        hal.scheduler->delay(1);
    }

    // release SPI bus for use by other sensors
    _spi_sem->give();
}
