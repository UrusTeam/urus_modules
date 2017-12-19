#include "DataFlash.h"

DataFlash_Class *DataFlash_Class::_instance;
/*
const AP_Param::GroupInfo DataFlash_Class::var_info[] PROGMEM = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: DataFlash Backend Storage type
    // @Description: 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
    // @Values: 0:None,1:File,2:MAVLink,3:BothFileAndMAVLink
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, DataFlash_Class, _params.backend_types,       DATAFLASH_BACKEND_FILE),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: Maximum DataFlash File Backend buffer size (in kilobytes)
    // @Description: The DataFlash_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
    // @User: Standard
    AP_GROUPINFO("_FILE_BUFSIZE",  1, DataFlash_Class, _params.file_bufsize,       16),

    // @Param: _DISARMED
    // @DisplayName: Enable logging while disarmed
    // @Description: If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DISARMED",  2, DataFlash_Class, _params.log_disarmed,       0),

    // @Param: _REPLAY
    // @DisplayName: Enable logging of information needed for Replay
    // @Description: If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_REPLAY",  3, DataFlash_Class, _params.log_replay,       0),

    // @Param: _FILE_DSRMROT
    // @DisplayName: Stop logging to current file on disarm
    // @Description: When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, DataFlash_Class, _params.file_disarm_rot,       0),

    AP_GROUPEND
};
*/
// start functions pass straight through to backend:
void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    backend->WriteBlock(pBuffer, size);
}
uint16_t DataFlash_Class::start_new_log() {
    return backend->start_new_log();
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    backend->EraseAll();
}
// change me to "LoggingAvailable"?
bool DataFlash_Class::CardInserted(void) {
    return backend->CardInserted();
}
// remove me in favour of calling "DoTimeConsumingPreparations" all the time?
bool DataFlash_Class::NeedErase(void) {
    return backend->NeedErase();
}

uint16_t DataFlash_Class::find_last_log(void) {
    return backend->find_last_log();
}
void DataFlash_Class::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {
    backend->get_log_boundaries(log_num, start_page, end_page);
}
void DataFlash_Class::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    backend->get_log_info(log_num, size, time_utc);
}
int16_t DataFlash_Class::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    return backend->get_log_data(log_num, page, offset, len, data);
}
uint16_t DataFlash_Class::get_num_logs(void) {
    return backend->get_num_logs();
}
void DataFlash_Class::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt) {
    backend->Log_Fill_Format(s, pkt);
}


void DataFlash_Class::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     print_mode_fn printMode,
                                     AP_HAL::BetterStream *port) {
    backend->LogReadProcess(log_num, start_page, end_page, printMode, port);
}

void DataFlash_Class::ShowDeviceInfo(AP_HAL::BetterStream *port) {
    backend->ShowDeviceInfo(port);
}

#ifndef DATAFLASH_NO_CLI
void DataFlash_Class::DumpPageInfo(AP_HAL::BetterStream *port) {
    backend->DumpPageInfo(port);
}

void DataFlash_Class::ListAvailableLogs(AP_HAL::BetterStream *port) {
    backend->ListAvailableLogs(port);
}
#endif // DATAFLASH_NO_CLI

bool DataFlash_Class::logging_started(void) {
    return backend->log_write_started;
}

void DataFlash_Class::EnableWrites(bool enable) {
    backend->EnableWrites(enable);
}

bool DataFlash_Class::should_log(const uint32_t mask) const
{
    /*
    if (!(mask & _log_bitmask)) {
        return false;
    }
    if (!vehicle_is_armed() && !log_while_disarmed()) {
        return false;
    }
    if (in_log_download()) {
        return false;
    }
    if (_next_backend == 0) {
        return false;
    }
    */
    return true;
}
/*
#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)
*/

void DataFlash_Class::StopLogging()
{
    //FOR_EACH_BACKEND(stop_logging());
}

/* Log_Write support */
void DataFlash_Class::Log_Write(const char *name, const char *labels, const char *fmt, ...)
{

}

void DataFlash_Class::set_vehicle_armed(const bool armed_state)
{

}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
void DataFlash_Class::flush(void) {
    backend->flush();
}
#endif

// end functions pass straight through to backend
