
#include <AP_HAL/AP_HAL.h>
#include "DataFlash.h"
#include <stdlib.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>

extern const AP_HAL::HAL& hal;

#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)

void DataFlash_Class::Init(const struct LogStructure *structure, uint8_t num_types)
{
    _num_types = num_types;
    _structures = structure;

    // DataFlash
#if defined(SHAL_CORE_APM1)
    backend = new DataFlash_APM1(*this);
#elif defined(SHAL_CORE_APM2)
    backend = new DataFlash_APM2(*this);
#elif defined(HAL_BOARD_LOG_DIRECTORY)
    backend = new DataFlash_File(*this, HAL_BOARD_LOG_DIRECTORY);
#else
    // no dataflash driver
    backend = new DataFlash_Empty(*this);
#endif
    if (backend == NULL) {
        AP_HAL::panic("Unable to open dataflash");
    }
    backend->Init(structure, num_types);
}

/*
  read and print a log entry using the format strings from the given structure
 */
void DataFlash_Backend::_print_log_entry(uint8_t msg_type,
                                         print_mode_fn printMode,
                                         AP_HAL::BetterStream *port)
{

    struct log_Format pkt_tmp;
    memset(&pkt_tmp, 0, sizeof(pkt_tmp));

    uint8_t i;
    for (i=0; i<_num_types; i++) {
        if (msg_type == PGM_UINT8(&_structures[i].msg_type)) {
            break;
        }
    }
    if (i == _num_types) {
        port->printf("UNKN, %u\n", (unsigned)msg_type);
        return;
    }

    uint8_t msg_len = PGM_UINT8(&_structures[i].msg_len) - 3;
    uint8_t pkt[msg_len];
    if (!ReadBlock(pkt, msg_len)) {
        return;
    }

    strncpy_P(pkt_tmp.name, &_structures[i].name[0], sizeof(pkt_tmp.name));
    port->printf("%s, ", pkt_tmp.name);
    for (uint8_t ofs=0, fmt_ofs=0; ofs<msg_len; fmt_ofs++) {
        char fmt = PGM_UINT8(&_structures[i].format[fmt_ofs]);
        switch (fmt) {
        case 'b': {
            port->printf("%d", (int)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'B': {
            port->printf("%u", (unsigned)pkt[ofs]);
            ofs += 1;
            break;
        }
        case 'h': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%d", (int)v);
            ofs += sizeof(v);
            break;
        }
        case 'H': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%u", (unsigned)v);
            ofs += sizeof(v);
            break;
        }
        case 'i': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%ld", (long)v);
            ofs += sizeof(v);
            break;
        }
        case 'I': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%lu", (unsigned long)v);
            ofs += sizeof(v);
            break;
        }
        case 'q': {
            int64_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%lld", (long long)v);
            ofs += sizeof(v);
            break;
        }
        case 'Q': {
            uint64_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%llu", (unsigned long long)v);
            ofs += sizeof(v);
            break;
        }
        case 'f': {
            float v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%f", (double)v);
            ofs += sizeof(v);
            break;
        }
        case 'd': {
            double v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            // note that %f here *really* means a single-precision
            // float, so we lose precision printing this double out
            // dtoa_engine needed....
            port->printf("%f", (double)v);
            ofs += sizeof(v);
            break;
        }
        case 'c': {
            int16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'C': {
            uint16_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'e': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'E': {
            uint32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            port->printf("%.2f", (double)(0.01f*v));
            ofs += sizeof(v);
            break;
        }
        case 'L': {
            int32_t v;
            memcpy(&v, &pkt[ofs], sizeof(v));
            print_latlon(port, v);
            ofs += sizeof(v);
            break;
        }
        case 'n': {
            char v[5];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'N': {
            char v[17];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'Z': {
            char v[65];
            memcpy(&v, &pkt[ofs], sizeof(v));
            v[sizeof(v)-1] = 0;
            port->printf("%s", v);
            ofs += sizeof(v)-1;
            break;
        }
        case 'M': {
            printMode(port, pkt[ofs]);
            ofs += 1;
            break;
        }
        default:
            ofs = msg_len;
            break;
        }
        if (ofs < msg_len) {
            port->printf(", ");
        }
    }
    port->printf("\n");

}

/*
  write a structure format to the log
 */
void DataFlash_Class::Log_Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Log_Fill_Format(s, pkt);
    WriteBlock(&pkt, sizeof(pkt));
}

// This function starts a new log file in the DataFlash, and writes
// the format of supported messages in the log
uint16_t DataFlash_Class::StartNewLog(void)
{
    uint16_t ret;
    ret = start_new_log();
    if (ret == 0xFFFF) {
        // don't write out formats if we fail to open the log
        return ret;
    }
    // write log formats so the log is self-describing
    for (uint8_t i=0; i<_num_types; i++) {
        Log_Write_Format(&_structures[i]);
        // avoid corrupting the APM1/APM2 dataflash by writing too fast
        hal.scheduler->delay(10);
    }

    return ret;
}

/*
  print FMT specifiers for log dumps where we have wrapped in the
  dataflash and so have no formats. This assumes the log being dumped
  using the same log formats as the current formats, but it is better
  than falling back to old defaults in the GCS
 */
void DataFlash_Block::_print_log_formats(AP_HAL::BetterStream *port)
{
    struct log_Format pkt;
    for (uint8_t i=0; i<_num_types; i++) {
        const struct LogStructure *s = &_structures[i];
        strncpy_P(pkt.name, &s->name[0], sizeof(pkt.name));
        strncpy_P(pkt.format, s->format, sizeof(pkt.format));
        strncpy_P(pkt.labels, s->labels, sizeof(pkt.labels));
        port->printf("FMT, %u, %u, %s, %s, %s\n",
                        (unsigned)PGM_UINT8(&s->msg_type),
                        (unsigned)PGM_UINT8(&s->msg_len),
                        pkt.name, pkt.format, pkt.labels);
    }
}

/*
  show information about the device
 */
void DataFlash_Block::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    if (!CardInserted()) {
        port->println("No dataflash inserted");
        return;
    }
    ReadManufacturerID();
    port->printf("Manufacturer: 0x%02x   Device: 0x%04x\n",
                    (unsigned)df_manufacturer,
                    (unsigned)df_device);
    port->printf("NumPages: %u  PageSize: %u\n",
                   (unsigned)df_NumPages+1,
                   (unsigned)df_PageSize);
}

/*
  write a structure format to the log - should be in frontend
 */
void DataFlash_Backend::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = PGM_UINT8(&s->msg_type);
    pkt.length = PGM_UINT8(&s->msg_len);
    strncpy_P(pkt.name, s->name, sizeof(pkt.name)+1);
    strncpy_P(pkt.format, s->format, sizeof(pkt.format));
    strncpy_P(pkt.labels, s->labels, sizeof(pkt.labels));
}

// Write a text message to the log
void DataFlash_Class::Log_Write_Message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };
    strncpy(pkt.msg, message, sizeof(pkt.msg));
    WriteBlock(&pkt, sizeof(pkt));
}

// This funciton finds the last log number
uint16_t DataFlash_Block::find_last_log(void)
{
    uint16_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
void DataFlash_Block::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    uint16_t num = get_num_logs();
    uint16_t look;

    if (df_BufferIdx != 0) {
        FinishWrite();
        hal.scheduler->delay(100);
    }

    if(num == 1)
    {
        StartRead(df_NumPages);
        if (GetFileNumber() == 0xFFFF)
        {
            start_page = 1;
            end_page = find_last_page_of_log((uint16_t)log_num);
        } else {
            end_page = find_last_page_of_log((uint16_t)log_num);
            start_page = end_page + 1;
        }

    } else {
        if(log_num==1) {
            StartRead(df_NumPages);
            if(GetFileNumber() == 0xFFFF) {
                start_page = 1;
            } else {
                start_page = find_last_page() + 1;
            }
        } else {
            if(log_num == find_last_log() - num + 1) {
                start_page = find_last_page() + 1;
            } else {
                look = log_num-1;
                do {
                    start_page = find_last_page_of_log(look) + 1;
                    look--;
                } while (start_page <= 0 && look >=1);
            }
        }
    }
    if (start_page == df_NumPages+1 || start_page == 0) {
        start_page = 1;
    }
    end_page = find_last_page_of_log(log_num);
    if (end_page == 0) {
        end_page = start_page;
    }
}

// find log size and time
void DataFlash_Block::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    uint16_t start, end;
    get_log_boundaries(log_num, start, end);
    if (end >= start) {
        size = (end + 1 - start) * (uint32_t)df_PageSize;
    } else {
        size = (df_NumPages + end - start) * (uint32_t)df_PageSize;
    }
    time_utc = 0;
}

// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint16_t DataFlash_Block::get_num_logs(void)
{
    uint16_t lastpage;
    uint16_t last;
    uint16_t first;

    if (find_last_page() == 1) {
        return 0;
    }

    StartRead(1);

    if (GetFileNumber() == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    StartRead(lastpage);
    last = GetFileNumber();
    StartRead(lastpage + 2);
    first = GetFileNumber();
    if(first > last) {
        StartRead(1);
        first = GetFileNumber();
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}

// This function starts a new log file in the DataFlash
uint16_t DataFlash_Block::start_new_log(void)
{
    uint16_t last_page = find_last_page();

    StartRead(last_page);
    //hal.console->printf("last page: %u\n", last_page);	//Serial.println(last_page);
    //hal.console->printf("file #: %u\n", GetFileNumber());
    //hal.console->printf("file page: %u\n", GetFilePage());
    //Serial.print("file #: ");	Serial.println(GetFileNumber());
    //Serial.print("file page: ");	Serial.println(GetFilePage());

    if(find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        SetFileNumber(1);
        StartWrite(1);
        //Serial.println("start log from 0");
        log_write_started = true;
        return 1;
    }

    uint16_t new_log_num;

    // Check for log of length 1 page and suppress
    if(GetFilePage() <= 1) {
        new_log_num = GetFileNumber();
        // Last log too short, reuse its number
        // and overwrite it
        SetFileNumber(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        SetFileNumber(new_log_num);
        StartWrite(last_page + 1);
    }
    log_write_started = true;
    return new_log_num;
}

// This function finds the last page of the last file
uint16_t DataFlash_Block::find_last_page(void)
{
    uint16_t look;
    uint16_t bottom = 1;
    uint16_t top = df_NumPages;
    uint32_t look_hash;
    uint32_t bottom_hash;
    uint32_t top_hash;

    StartRead(bottom);
    bottom_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();

    while(top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash < bottom_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// This function finds the last page of a particular log file
uint16_t DataFlash_Block::find_last_page_of_log(uint16_t log_number)
{
    uint16_t look;
    uint16_t bottom;
    uint16_t top;
    uint32_t look_hash;
    uint32_t check_hash;

    if(check_wrapped())
    {
        StartRead(1);
        bottom = GetFileNumber();
        if (bottom > log_number)
        {
            bottom = find_last_page();
            top = df_NumPages;
        } else {
            bottom = 1;
            top = find_last_page();
        }
    } else {
        bottom = 1;
        top = find_last_page();
    }

    check_hash = (int32_t)log_number<<16 | 0xFFFF;

    while(top-bottom > 1)
    {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash > check_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
        }
    }

    StartRead(top);
    if (GetFileNumber() == log_number) return top;

    StartRead(bottom);
    if (GetFileNumber() == log_number) return bottom;

    return -1;
}

bool DataFlash_Block::check_wrapped(void)
{
    StartRead(df_NumPages);
    if(GetFileNumber() == 0xFFFF)
        return 0;
    else
        return 1;
}

