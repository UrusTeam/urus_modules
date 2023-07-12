#ifndef __AP_INERTIAL_SENSOR_ITG3200_H__
#define __AP_INERTIAL_SENSOR_ITG3200_H__

#include <string.h>
#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

// MPNG board types
#define CRIUS_V1  1
#define RCTIMER_CRIUS_V2 2
#define HK_RED_MULTIWII_PRO 3
#define BLACK_VORTEX 4
#define MULTIWII_PRO_EZ3_BLACK 5
#define PARIS_V5_OSD 6

class AP_InertialSensor_ITG3200 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_ITG3200(AP_InertialSensor &imu, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_gyro,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_accel);

    enum Sample_rate {
        RATE_50HZ,
        RATE_100HZ,
        RATE_200HZ
    };

    /* Concrete implementation of AP_InertialSensor functions: */
    bool 					update();
    float 				get_gyro_drift_rate();

    bool				sample_available();
    bool                        wait_for_sample(uint16_t timeout_ms);
    void           _poll_data(void);

    // get_delta_time returns the time period in seconds overwhich the sensor data was collected
    float					get_delta_time();

    // Init I2C Bypass mode
    void                  hardware_init_i2c_bypass();
    void    start() override;

protected:
    uint16_t 			_init_sensor( Sample_rate sample_rate );
    float         _delta_time;
private:
    bool read(uint32_t);

    void           _read_data_from_timerprocess();
    void           _read_data_transaction();
    bool 					hardware_init(Sample_rate sample_rate);

    static AP_HAL::Semaphore *_i2c_sem;

    uint16_t				_num_samples;

    float                 _temp;

    float 				_temp_to_celsius( uint16_t );

    static float 	_gyro_scale;
    static float 	_accel_scale_1G;

    static uint8_t 		_gyro_data_index[3];
    static int8_t 		_gyro_data_sign[3];

    static uint8_t 		_accel_data_index[3];
    static int8_t 		_accel_data_sign[3];

    static const uint8_t 		_temp_data_index;

    //static uint8_t _accel_addr;
    //static uint8_t _board_type;
    static uint16_t _micros_per_sample;

// ensure we can't initialise twice
    unsigned _initialised:1;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_gyro;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_accel;

    uint8_t _mpu6000_filter = 20;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;
};

#endif // __AP_INERTIAL_SENSOR_ITG3200_H__
