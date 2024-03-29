#pragma once
/*
  driver for the invensense range of IMUs, including:

  MPU6000
  MPU9250
  ICM-20608
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#if !HAL_MINIMIZE_FEATURES_AVR
#include "AuxiliaryBus.h"

class AP_Invensense_AuxiliaryBus;
class AP_Invensense_AuxiliaryBusSlave;
#endif

class AP_InertialSensor_Invensense : public AP_InertialSensor_Backend
{
#if !HAL_MINIMIZE_FEATURES_AVR
    friend AP_Invensense_AuxiliaryBus;
    friend AP_Invensense_AuxiliaryBusSlave;
#endif

public:
    virtual ~AP_InertialSensor_Invensense();

    static AP_InertialSensor_Invensense &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Invensense&>(backend);
    }

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    /* update accel and gyro state */
    bool update() override;
    void accumulate() override;

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
#if !HAL_MINIMIZE_FEATURES_AVR
    AuxiliaryBus *get_auxiliary_bus() override;
#endif

    void start() override;

    enum Invensense_Type {
        Invensense_MPU6000=0,
        Invensense_MPU6500,
        Invensense_MPU9250,
        Invensense_ICM20608,
        Invensense_ICM20602,
    };

private:
    AP_InertialSensor_Invensense(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /* Initialize sensor*/
    bool _init();
    bool _hardware_init();
    bool _check_whoami();

    void _set_filter_register(void);
    void _fifo_reset();
    bool _has_auxiliary_bus();

    /* Read samples from FIFO (FIFO enabled) */
    void _read_fifo();

    /* Check if there's data available by either reading DRDY pin or register */
    bool _data_ready();

    /* Poll for new data (non-blocking) */
    void _poll_data();

    /* Read and write functions taking the differences between buses into
     * account */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    bool _accumulate(uint8_t *samples, uint8_t n_samples);
    bool _accumulate_fast_sampling(uint8_t *samples, uint8_t n_samples);

    bool _check_raw_temp(int16_t t2);

    int16_t _raw_temp;

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    uint16_t _error_count;

    float temp_sensitivity = 1.0/340; // degC/LSB
    float temp_zero = 36.53; // degC

    float _temp_filtered;
    float _accel_scale;
    LowPassFilter2pFloat _temp_filter;

    enum Rotation _rotation;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
#if !HAL_MINIMIZE_FEATURES_AVR
    AP_Invensense_AuxiliaryBus *_auxiliary_bus;
#endif

    // which sensor type this is
    enum Invensense_Type _mpu_type;

    // are we doing more than 1kHz sampling?
    bool _fast_sampling;

    // Last status from register user control
    uint8_t _last_stat_user_ctrl;

    // buffer for fifo read
    uint8_t *_fifo_buffer;

    volatile bool need_reset = false;

    /*
      accumulators for fast sampling
      See description in _accumulate_fast_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t count;
        LowPassFilterVector3f accel_filter{4000, 188};
        LowPassFilterVector3f gyro_filter{8000, 188};
    } _accum;
};

#if !HAL_MINIMIZE_FEATURES_AVR
class AP_Invensense_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_Invensense_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_Invensense_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu_addr;
    const uint8_t _mpu_reg;
    const uint8_t _mpu_ctrl;
    const uint8_t _mpu_do;

    uint8_t _ext_sens_data = 0;
};

class AP_Invensense_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_Invensense;

public:
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    AP_Invensense_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};
#endif
