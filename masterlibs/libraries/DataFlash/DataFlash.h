/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Rally/AP_Rally.h>
#include <stdint.h>
#include "DataFlash_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <uORB/topics/esc_status.h>
#endif


class DataFlash_Backend;

enum DataFlash_Backend_Type {
    DATAFLASH_BACKEND_NONE = 0,
    DATAFLASH_BACKEND_FILE = 1,
    DATAFLASH_BACKEND_MAVLINK = 2,
    DATAFLASH_BACKEND_BOTH = 3,
};


class DataFlash_Class
{
public:
    FUNCTOR_TYPEDEF(print_mode_fn, void, AP_HAL::BetterStream*, uint8_t);

    DataFlash_Class(const char *firmware_string, const AP_Int32 &log_bitmask) :
        _firmware_string(firmware_string),
        _log_bitmask(log_bitmask)
        {
            //AP_Param::setup_object_defaults(this, var_info);
            if (_instance != nullptr) {
                AP_HAL::panic("DataFlash must be singleton");
            }
            _instance = this;
        }

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);

    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);

    void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page,
                                print_mode_fn printMode,
                                AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
#ifndef DATAFLASH_NO_CLI
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);
#endif // DATAFLASH_NO_CLI

    /* logging methods common to all vehicles */
    uint16_t StartNewLog(void);
    void AddLogFormats(const struct LogStructure *structures, uint8_t num_types);
    void EnableWrites(bool enable);
    void Log_Write_SysInfo(const prog_char_t *firmware_string);
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const AP_GPS &gps, uint8_t instance, int32_t relative_alt);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_IMUDT(const AP_InertialSensor &ins);
    void Log_Write_Vibration(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
    void Log_Write_POS(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs, bool optFlowEnabled);
#endif
    void Log_Write_MavCmd(uint16_t cmd_total, const mavlink_mission_item_t& mav_cmd);
    void Log_Write_Radio(const mavlink_radio_t &packet);
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);
    void Log_Write_Camera(const AP_AHRS &ahrs, const AP_GPS &gps, const Location &current_loc);
    void Log_Write_ESC(void);
    void Log_Write_Airspeed(AP_Airspeed &airspeed);
    void Log_Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets);
    void Log_Write_Current(const AP_BattMonitor &battery, int16_t throttle);
    void Log_Write_Compass(const Compass &compass);
    void Log_Write_Mode(uint8_t mode);
    void Log_Write_Parameters(void);

    void Log_Write_EntireMission(const AP_Mission &mission);
    void Log_Write_Mission_Cmd(const AP_Mission &mission,
                               const AP_Mission::Mission_Command &cmd);
    void Log_Write_Origin(uint8_t origin_type, const Location &loc);
    void Log_Write_RPM(const AP_RPM &rpm_sensor);

    // This structure provides information on the internal member data of a PID for logging purposes
    struct PID_Info {
        float desired;
        float P;
        float I;
        float D;
        float FF;
        float AFF;
    };

    void Log_Write_PID(uint8_t msg_type, const PID_Info &info);

    bool logging_started(void);

    // get singleton instance
    static DataFlash_Class *instance(void) {
        return _instance;
    }


    // returns true if logging of a message should be attempted
    bool should_log(uint32_t mask) const;
/*
    // parameter support
    static const struct AP_Param::GroupInfo        var_info[];
    struct {
        AP_Int8 backend_types;
        AP_Int8 file_bufsize; // in kilobytes
        AP_Int8 file_disarm_rot;
        AP_Int8 log_disarmed;
        AP_Int8 log_replay;
    } _params;
*/
    void StopLogging();

    /* Log_Write support */
    void Log_Write(const char *name, const char *labels, const char *fmt, ...);
    void set_vehicle_armed(const bool armed_state);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only DataFlash_File support this:
    void flush(void);
#endif

protected:
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Log_Write_Parameter(const AP_Param *ap, const AP_Param::ParamToken &token,
                             enum ap_var_type type);
    uint16_t start_new_log(void);

    const struct LogStructure *_structures;
    uint8_t _num_types;

private:
    DataFlash_Backend *backend;
    static DataFlash_Class *_instance;
    const char *_firmware_string;
    const AP_Int32 &_log_bitmask;
    uint8_t _next_backend;
};

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char name[5];
    const char format[16];
    const char labels[64];
};

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char name[16];
    float value;
};

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint8_t  used;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    uint32_t gyro_error, accel_error;
    float temperature;
    uint8_t gyro_health, accel_health;
};

struct PACKED log_IMUDT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float delta_time, delta_vel_dt;
    float delta_ang_x, delta_ang_y, delta_ang_z;
    float delta_vel_x, delta_vel_y, delta_vel_z;
};

struct PACKED log_Vibe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vibe_x, vibe_y, vibe_z;
    uint32_t clipping_0, clipping_1, clipping_2;
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
};

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   altitude;
    float   pressure;
    int16_t temperature;
    float   climbrate;
};

struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
};

struct PACKED log_POS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_alt;
};

struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t Vcc;
    uint16_t Vservo;
    uint16_t flags;
};

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int8_t Ratio;
    int8_t AZ1bias;
    int8_t AZ2bias;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovVT;
};

struct PACKED log_EKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarMX;
    int16_t sqrtvarMY;
    int16_t sqrtvarMZ;
    int16_t sqrtvarVT;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint8_t faults;
    uint8_t timeouts;
    uint16_t solution;
};

struct PACKED log_EKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
};

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    float latitude;
    float longitude;
    float altitude;
};

struct PACKED log_Radio {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint8_t noise;
    uint8_t remnoise;
    uint16_t rxerrors;
    uint16_t fixed;
};

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

struct PACKED log_PID {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   desired;
    float   P;
    float   I;
    float   D;
    float   FF;
    float   AFF;
};

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  throttle;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
    int16_t  battery2_voltage;
};

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
    uint8_t  health;
};

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t mode;
    uint8_t mode_num;
};

/*
  terrain log structure
 */
struct PACKED log_TERRAIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t status;
    int32_t lat;
    int32_t lng;
    uint16_t spacing;
    float terrain_height;
    float current_height;
    uint16_t pending;
    uint16_t loaded;
};

/*
  UBlox logging
 */
struct PACKED log_Ubx1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t noisePerMS;
    uint8_t  jamInd;
    uint8_t  aPower;
    uint16_t agcCnt;
};

struct PACKED log_Ubx2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
};

struct PACKED log_Ubx3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float hAcc;
    float vAcc;
    float sAcc;
};

struct PACKED log_GPS_RAW {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t iTOW;
    int16_t week;
    uint8_t numSV;
    uint8_t sv;
    double cpMes;
    double prMes;
    float doMes;
    int8_t mesQI;
    int8_t cno;
    uint8_t lli;
};

struct PACKED log_GPS_RAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
};

struct PACKED log_GPS_RAWS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double prMes;
    double cpMes;
    float doMes;
    uint8_t gnssId;
    uint8_t svId;
    uint8_t freqId;
    uint16_t locktime;
    uint8_t cno;
    uint8_t prStdev;
    uint8_t cpStdev;
    uint8_t doStdev;
    uint8_t trkStat;
};

struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t rpm;
    int16_t voltage;
    int16_t current;
    int16_t temperature;
};

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
    float   rawpressure;
    float   offset;
};

struct PACKED log_ACCEL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float AccX, AccY, AccZ;
};

struct PACKED log_GYRO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float GyrX, GyrY, GyrZ;
};

struct PACKED log_ORGN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t origin_type;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float rpm1;
    float rpm2;
};

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
  q   : int64_t
  Q   : uint64_t
 */

// messages for all boards
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "QNf",        "TimeUS,Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "QBIHBcLLeeEefB", "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,RAlt,Alt,Spd,GCrs,VZ,U" }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "Qhhhhhhhhhhhhhh",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "Qhhhhhhhhhhhh",     "TimeUS,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7,Ch8,Ch9,Ch10,Ch11,Ch12" }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  "Qffcf", "TimeUS,Alt,Press,Temp,CRt" }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QCCH","TimeUS,Vcc,VServo,Flags" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHfffffff","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,Roll,Pitch,Yaw" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), \
      "ARSP",  "Qffcff",   "TimeUS,Airspeed,DiffPress,Temp,RawPress,Offset" }, \
    { LOG_CURRENT_MSG, sizeof(log_Current), \
      "CURR", "QhhhHfh","TimeUS,Throttle,Volt,Curr,Vcc,CurrTot,Volt2" },\
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "QccccCCCC", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", "QhhhhhhhhhB",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMB",         "TimeUS,Mode,ModeNum" }

// messages for more advanced boards
#define LOG_EXTRA_STRUCTURES \
    { LOG_GPS2_MSG, sizeof(log_GPS), \
      "GPS2",  "QBIHBcLLeeEefB", "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,RAlt,Alt,Spd,GCrs,VZ,U" }, \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  "QffffffIIfBB",     "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,ErrG,ErrA,Temp,GyHlt,AcHlt" }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","QccCfLL","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
      "POS","QLLff","TimeUS,Lat,Lng,Alt,RelAlt" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLL","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_EKF1_MSG, sizeof(log_EKF1), \
      "EKF1","QccCffffffccc","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_EKF2_MSG, sizeof(log_EKF2), \
      "EKF2","Qbbbcchhhhhh","TimeUS,Ratio,AZ1bias,AZ2bias,VWN,VWE,MN,ME,MD,MX,MY,MZ" }, \
    { LOG_EKF3_MSG, sizeof(log_EKF3), \
      "EKF3","Qcccccchhhc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IVT" }, \
    { LOG_EKF4_MSG, sizeof(log_EKF4), \
      "EKF4","QcccccccbbBBH","TimeUS,SV,SP,SH,SMX,SMY,SMZ,SVT,OFN,EFE,FS,TS,SS" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded" }, \
    { LOG_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_UBX3_MSG, sizeof(log_Ubx3), \
      "UBX3", "QBfff", "TimeUS,Instance,hAcc,vAcc,sAcc" }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli" }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat" }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk" }, \
    { LOG_ESC1_MSG, sizeof(log_Esc), \
      "ESC1",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC2_MSG, sizeof(log_Esc), \
      "ESC2",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC3_MSG, sizeof(log_Esc), \
      "ESC3",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC4_MSG, sizeof(log_Esc), \
      "ESC4",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC5_MSG, sizeof(log_Esc), \
      "ESC5",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC6_MSG, sizeof(log_Esc), \
      "ESC6",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC7_MSG, sizeof(log_Esc), \
      "ESC7",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_ESC8_MSG, sizeof(log_Esc), \
      "ESC8",  "Qcccc", "TimeUS,RPM,Volt,Curr,Temp" }, \
    { LOG_EKF5_MSG, sizeof(log_EKF5), \
      "EKF5","QBhhhcccCC","TimeUS,normInnov,FIX,FIY,AFI,HAGL,offset,RI,meaRng,errHAGL" }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2","QhhhhhhhhhB",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3","QhhhhhhhhhB",    "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health" }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", "QQfff",        "TimeUS,SampleUS,AccX,AccY,AccZ" }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", "QQfff",        "TimeUS,SampleUS,GyrX,GyrY,GyrZ" }, \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  "Qffcf", "TimeUS,Alt,Press,Temp,CRt" }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QfffIII",     "TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2" }, \
    { LOG_IMUDT_MSG, sizeof(log_IMUDT), \
      "IMT","Qffffffff","TimeUS,DelT,DelvT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_IMUDT2_MSG, sizeof(log_IMUDT), \
      "IMT2","Qffffffff","TimeUS,DelT,DelvT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_IMUDT3_MSG, sizeof(log_IMUDT), \
      "IMT3","Qffffffff","TimeUS,DelT,DelvT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ" }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
      "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2" }

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES, LOG_EXTRA_STRUCTURES
#else
#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES
#endif

// message types 0 to 128 reversed for vehicle specific use

// message types for common messages
enum LogMessages {
    LOG_FORMAT_MSG = 128,   //000
    LOG_PARAMETER_MSG,      //001
    LOG_GPS_MSG,            //002
    LOG_GPS2_MSG,           //003
    LOG_IMU_MSG,            //004
    LOG_MESSAGE_MSG,        //005
    LOG_RCIN_MSG,           //006
    LOG_RCOUT_MSG,          //007
    LOG_IMU2_MSG,           //008
    LOG_BARO_MSG,           //009
    LOG_POWR_MSG,           //010
    LOG_AHR2_MSG,           //011
    LOG_SIMSTATE_MSG,       //012
    LOG_EKF1_MSG,           //013
    LOG_EKF2_MSG,           //014
    LOG_EKF3_MSG,           //015
    LOG_EKF4_MSG,           //016
    LOG_CMD_MSG,            //017
    LOG_RADIO_MSG,          //018
    LOG_ATRP_MSG,           //019
    LOG_CAMERA_MSG,         //020
    LOG_IMU3_MSG,           //021
    LOG_TERRAIN_MSG,        //022
    LOG_UBX1_MSG,           //023
    LOG_UBX2_MSG,           //024
    LOG_UBX3_MSG,           //025
    LOG_ESC1_MSG,           //026
    LOG_ESC2_MSG,           //027
    LOG_ESC3_MSG,           //028
    LOG_ESC4_MSG,           //029
    LOG_ESC5_MSG,           //030
    LOG_ESC6_MSG,           //031
    LOG_ESC7_MSG,           //032
    LOG_ESC8_MSG,           //033
    LOG_EKF5_MSG,           //034
    LOG_BAR2_MSG,           //035
    LOG_ARSP_MSG,           //036
    LOG_ATTITUDE_MSG,       //037
    LOG_CURRENT_MSG,        //038
    LOG_COMPASS_MSG,        //039
    LOG_COMPASS2_MSG,       //040
    LOG_COMPASS3_MSG,       //041
    LOG_MODE_MSG,           //042
    LOG_GPS_RAW_MSG,        //043
    LOG_GPS_RAWH_MSG,       //043
    LOG_GPS_RAWS_MSG,       //044
    LOG_ACC1_MSG,           //045
    LOG_ACC2_MSG,           //046
    LOG_ACC3_MSG,           //047
    LOG_GYR1_MSG,           //048
    LOG_GYR2_MSG,           //049
    LOG_GYR3_MSG,           //050
    LOG_POS_MSG,            //051
    LOG_PIDR_MSG,           //052
    LOG_PIDP_MSG,           //053
    LOG_PIDY_MSG,           //054
    LOG_PIDA_MSG,           //055
    LOG_PIDS_MSG,           //056
    LOG_VIBE_MSG,           //057
    LOG_IMUDT_MSG,          //058
    LOG_IMUDT2_MSG,         //059
    LOG_IMUDT3_MSG,         //060
    LOG_ORGN_MSG,           //061
    LOG_RPM_MSG             //062
};

enum LogOriginType {
    ekf_origin = 0,
    ahrs_home = 1
};

// message types 200 to 210 reversed for GPS driver use
// message types 211 to 220 reversed for autotune use

#include "DataFlash_Block.h"
#include "DataFlash_File.h"

#endif
