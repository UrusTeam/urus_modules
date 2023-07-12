#include <AP_HAL/AP_HAL.h>

#include <assert.h>
#include <utility>
#include <stdio.h>

#include "AP_InertialSensor_ITG3200.h"

extern const AP_HAL::HAL& hal;

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);

//#define ITG3200_ADDRESS  	0x68 // 0xD0

#define GYRO_SMPLRT_50HZ 19 // 1KHz/(divider+1)
#define GYRO_SMPLRT_100HZ 9 // 1KHz/(divider+1)
#define GYRO_SMPLRT_200HZ 4 // 1KHz/(divider+1)

#define GYRO_DLPF_CFG_5HZ 6
#define GYRO_DLPF_CFG_10HZ 5
#define GYRO_DLPF_CFG_20HZ 4
#define GYRO_DLPF_CFG_42HZ 3
#define GYRO_DLPF_CFG_98HZ 2

// ITG-3200 14.375 LSB/degree/s
float AP_InertialSensor_ITG3200::_gyro_scale = 0.0012141421; // ToRad(1/14.375)
float AP_InertialSensor_ITG3200::_accel_scale_1G = (GRAVITY_MSS / 2730.0f); // 2730 LSB = 1G



uint8_t AP_InertialSensor_ITG3200::_gyro_data_index[3] = { 1, 2, 0 };
uint8_t AP_InertialSensor_ITG3200::_accel_data_index[3] = { 4, 5, 6 };
int8_t AP_InertialSensor_ITG3200::_gyro_data_sign[3] = { 1, 1, -1 };
int8_t AP_InertialSensor_ITG3200::_accel_data_sign[3] = { 1, 1, -1 };

const uint8_t AP_InertialSensor_ITG3200::_temp_data_index = 3;

//uint8_t AP_InertialSensor_ITG3200::_accel_addr = 0x40;
//uint8_t AP_InertialSensor_ITG3200::_board_type = 0;
uint16_t AP_InertialSensor_ITG3200::_micros_per_sample = 9500; // 100Hz

/* Static I2C device driver */
AP_HAL::Semaphore* AP_InertialSensor_ITG3200::_i2c_sem = NULL;

static volatile uint32_t _ins_timer = 0;

AP_InertialSensor_ITG3200::AP_InertialSensor_ITG3200(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel)
    : AP_InertialSensor_Backend(imu),
    _dev_gyro(std::move(dev_gyro)),
    _dev_accel(std::move(dev_accel))
{
    _initialised = false;
/*
    _board_type = HK_RED_MULTIWII_PRO;

    if (_board_type == BLACK_VORTEX) {
        _accel_addr = 0x41;
        _accel_scale_1G = (GRAVITY_MSS / 1024.0f); // BMA280 - 1024 LSB = 1G
    } else if (_board_type == PARIS_V5_OSD) {
        _accel_addr = 0x18;
        _gyro_scale = 0.0010642251536551; // ITG3050 - 16.4 LSB/degree/s = ToRad(1/16.4)
        _accel_scale_1G = (GRAVITY_MSS / 1024.0f); // BMA280 - 1024 LSB = 1G
        _gyro_data_index[0] = 2;
        _gyro_data_index[1] = 1;
        _gyro_data_index[2] = 0;
        _accel_data_index[0] = 4;
        _accel_data_index[1] = 5;
        _accel_data_index[2] = 6;
        _gyro_data_sign[0] = -1;
        _gyro_data_sign[1] = 1;
        _gyro_data_sign[2] = -1;
        _accel_data_sign[0] = 1;
        _accel_data_sign[1] = 1;
        _accel_data_sign[2] = -1;
    }
*/
}

uint16_t AP_InertialSensor_ITG3200::_init_sensor(Sample_rate sample_rate)
{
    if (_initialised) return 1;
    _initialised = true;

    _i2c_sem = _dev_gyro->get_semaphore();

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;
    do {
        bool success = hardware_init(sample_rate);
        if (success) {
            break;
        } else {
            hal.scheduler->delay(50); // delay for 50ms
            hal.console->println_P(
                PSTR("ITG E-04"));
        }
        if (tries++ > 5) {
            hal.console->printf_PS(PSTR("ITG E-01"));
        }
    } while (1);

    hal.scheduler->resume_timer_procs();


    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
    _ins_timer = AP_HAL::micros();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ITG3200::_poll_data, void));
    return 1;

}

void AP_InertialSensor_ITG3200::start()
{
    _init_sensor(RATE_200HZ);
    /*
    _imu.register_gyro(200, 0);
    _imu.register_accel(200, 1);
    */
    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_ITG3200::wait_for_sample(uint16_t timeout_ms)
{
    if (sample_available()) {
        return true;
    }
    uint32_t start = AP_HAL::millis();
    while ((AP_HAL::millis() - start) < timeout_ms) {
        hal.scheduler->delay_microseconds(100);
        if (sample_available()) {
            return true;
        }
    }
    return false;
}

bool AP_InertialSensor_ITG3200::update( void )
{
    int32_t sum[7];
    float count_scale;
    Vector3f accel_scale = _imu.get_accel_scale();

    // wait for at least 1 sample
    if (!wait_for_sample(1000)) {
        return false;
    }

    // disable interrupts for mininum time
    hal.scheduler->suspend_timer_procs();
    /** ATOMIC SECTION w/r/t TIMER PROCESS */
    {
        for (int i=0; i<7; i++) {
            sum[i] = _sum[i];
            _sum[i] = 0;
        }

        _num_samples = _count;
        _count = 0;
    }
    hal.scheduler->resume_timer_procs();

    count_scale = 1.0f / _num_samples;

    _imu.set_gyro(_imu.get_primary_gyro(), Vector3f(_gyro_data_sign[0] * sum[_gyro_data_index[0]],
                     _gyro_data_sign[1] * sum[_gyro_data_index[1]],
                     _gyro_data_sign[2] * sum[_gyro_data_index[2]]));
    Vector3f _vgyro = _imu.get_gyro();
    _vgyro.rotate(ROTATION_NONE);
    _vgyro *= _gyro_scale * count_scale;
    _vgyro -= _imu.get_gyro_offsets();
    _imu.set_gyro(_imu.get_primary_gyro(), _vgyro);

    _imu.set_accel(_imu.get_primary_accel(), Vector3f(_accel_data_sign[0] * sum[_accel_data_index[0]],
                      _accel_data_sign[1] * sum[_accel_data_index[1]],
                      _accel_data_sign[2] * sum[_accel_data_index[2]]));

    Vector3f _accel = _imu.get_accel();
    _accel.rotate(ROTATION_NONE);
    _accel *= count_scale * _accel_scale_1G;
    _accel.x *= accel_scale.x;
    _accel.y *= accel_scale.y;
    _accel.z *= accel_scale.z;
    _accel -= _imu.get_accel_offsets();

    //_notify_new_accel_raw_sample(_imu.get_primary_accel(), _accel, 0, false);
    //_notify_new_gyro_raw_sample(_imu.get_primary_gyro(), _vgyro);
    _publish_accel(_imu.get_primary_accel(), _accel);
    _publish_gyro(_imu.get_primary_gyro(), _vgyro);

    _temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

    _publish_temperature(0, _temp);

    return true;
}


// get_delta_time returns the time period in seconds overwhich the sensor data was collected
float AP_InertialSensor_ITG3200::get_delta_time()
{
    return _delta_time;
}

float AP_InertialSensor_ITG3200::_temp_to_celsius ( uint16_t regval )
{
    return 20.0;
}

void AP_InertialSensor_ITG3200::_poll_data(void)
{
    uint32_t now = AP_HAL::micros();
    if (now - _ins_timer > _micros_per_sample) {
        _ins_timer = now;

        if (hal.scheduler->in_main_thread()) {
            _read_data_from_timerprocess();
        } else {
            /* Synchronous read - take semaphore */
            bool got = _i2c_sem->take(10);
            if (got) {
                _read_data_transaction();
                _i2c_sem->give();
            } else {
                hal.console->printf_PS(
                    PSTR("ITG E-02"));
            }
        }
    }
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_ITG3200::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// get number of samples read from the sensors
bool AP_InertialSensor_ITG3200::sample_available()
{
    return _count > 0;
}




/*================ HARDWARE FUNCTIONS ==================== */
void AP_InertialSensor_ITG3200::_read_data_from_timerprocess()
{
    if (!_i2c_sem->take_nonblocking()) {
        /*
          the semaphore being busy is an expected condition when the
          mainline code is calling sample_available() which will
          grab the semaphore. We return now and rely on the mainline
          code grabbing the latest sample.
         */
        return;
    }

    _read_data_transaction();

    _i2c_sem->give();
}

void AP_InertialSensor_ITG3200::_read_data_transaction()
{
    // now read the data
    uint8_t raw[6];

    memset(raw,0,6);
    _dev_gyro->read_registers(0X1D, raw, 6);
    _sum[0] += (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    _sum[1] += (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    _sum[2] += (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);

    memset(raw,0,6);
    _dev_accel->read_registers(0x02, raw, 6);
    _sum[4] += (int16_t)(((uint16_t)raw[3] << 8) | raw[2]) >> 2; // Lower 2 bits has no data, we just cut it
    _sum[5] += (int16_t)(((uint16_t)raw[1] << 8) | raw[0]) >> 2;
    _sum[6] += (int16_t)(((uint16_t)raw[5] << 8) | raw[4]) >> 2;


    _count++;
    if (_count == 0) {
        // rollover - v unlikely
        memset((void*)_sum, 0, sizeof(_sum));
    }
}

bool AP_InertialSensor_ITG3200::hardware_init(Sample_rate sample_rate)
{

    if (!_i2c_sem->take(100)) {
        hal.console->printf_PS(PSTR("ITG E-03"));
    }

    // Chip reset
    hal.scheduler->delay(10);
    _dev_gyro->write_register(0x3E, 0x80);
    hal.scheduler->delay(5);

    // sample rate and filtering
    uint8_t filter, default_filter;

    // to minimise the effects of aliasing we choose a filter
    // that is less than half of the sample rate
    switch (sample_rate) {
    case RATE_50HZ:
        _dev_gyro->write_register(0x15, GYRO_SMPLRT_50HZ);
        default_filter = GYRO_DLPF_CFG_20HZ;
        _micros_per_sample = 19500;
        _delta_time = 0.02;
        break;
    case RATE_100HZ:
        _dev_gyro->write_register(0x15, GYRO_SMPLRT_100HZ);
        default_filter = GYRO_DLPF_CFG_20HZ;
        _micros_per_sample = 9500;
        _delta_time = 0.01;
        break;
    case RATE_200HZ:
    default:
        _dev_gyro->write_register(0x15, GYRO_SMPLRT_200HZ);
        default_filter = GYRO_DLPF_CFG_42HZ;
        _micros_per_sample = 4500;
        _delta_time = 0.005;
        break;
    }
    hal.scheduler->delay(5);

    // choose filtering frequency
    switch (_mpu6000_filter) {
    case 5:
        filter = GYRO_DLPF_CFG_5HZ;
        break;
    case 10:
        filter = GYRO_DLPF_CFG_10HZ;
        break;
    case 20:
        filter = GYRO_DLPF_CFG_20HZ;
        break;
    case 42:
        filter = GYRO_DLPF_CFG_42HZ;
        break;
    case 98:
        filter = GYRO_DLPF_CFG_98HZ;
        break;
    case 0:
    default:
        // the user hasn't specified a specific frequency,
        // use the default value for the given sample rate
        filter = default_filter;
    }
    _dev_gyro->write_register(0x16, 0x18+filter);

    hal.scheduler->delay(5);
    _dev_gyro->write_register(0x3E, 0x03);
    hal.scheduler->delay(10);

    /*
    if (_board_type == PARIS_V5_OSD) {
        _dev_accel->write_register(0x10, 0x09); // acceleration data filter bandwidth = 15,63Hz
        hal.scheduler->delay(5);
        _dev_accel->write_register(0x0F, 0x08); // range = 8G
    } else {
    */
        _dev_accel->write_register(0x0D, 1<<4);
        hal.scheduler->delay(1);
        _dev_accel->write_register(0x35, 3<<1); // range = 8G
        hal.scheduler->delay(1);
        _dev_accel->write_register(0x20, 0<<4);
    //}
    hal.scheduler->delay(10);

    _i2c_sem->give();

    return true;
}
