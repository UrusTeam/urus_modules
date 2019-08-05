#pragma once

#include "AP_Baro.h"

class AP_Baro_Backend
{
public:
    AP_Baro_Backend(AP_Baro &baro);
    virtual ~AP_Baro_Backend(void) {};

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

    // callback for UAVCAN messages
    virtual void handle_baro_msg(float pressure, float temperature) {}

    void backend_update(uint8_t instance);

    virtual float calculate_qnh(float alt_qnh, float pressure_qnh, float temp, uint8_t alt_qnh_unit) { return 0; };
    virtual float calculate_qfe(float alt_qfe, float pressure_qfe, float temp, uint8_t alt_qfe_unit) { return 0; };
    virtual float get_altitude_difference(float base_pressure, float pressure, float temp) const { return 0; };

protected:
    // reference to frontend object
    AP_Baro &_frontend;

    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;

    virtual void update_healthy_flag(uint8_t instance);

};
