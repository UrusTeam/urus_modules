#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper.h"
#include "UR_Stepper_Backend.h"
#include "UR_Stepper_Generic.h"

#define UR_STEPPER_GENERIC_TYPE 1

extern const AP_HAL::HAL& hal;

UR_Stepper::UR_Stepper()
{}

UR_Stepper::UR_Stepper(UR_STEPPER_NAMESPACE::stepper_profile_t profile) :
    _profile(profile)
{}

void UR_Stepper::configure(ProcessMode process_mode)
{
    _configure_backends(process_mode);
}

void UR_Stepper::update()
{
    if (!_backends) {
        return;
    }

    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->update();
    }
}

void UR_Stepper::_configure_backends()
{
    if (_backends_configuring) {
        return;
    }

    _backends_configuring = true;
#if defined(UR_STEPPER_GENERIC_TYPE)
    _add_backend(UR_Stepper_Generic::configure(*this));
#endif
    _backends_configuring = false;
}

void UR_Stepper::_configure_backends(ProcessMode process_mode)
{
    _configure_backends();

    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->setup_process(process_mode);
    }
}

bool UR_Stepper::_add_backend(UR_Stepper_Backend *backend)
{
    if (!backend) {
        return false;
    }

    if (_backend_count == UR_STEPPER_MAX_BACKENDS) {
        hal.console->printf("UR_Stepper: MAX BACKEND REACHED!\n");
    }

    _backends[_backend_count++] = backend;

    return true;
}

void UR_Stepper::move_degree(int64_t deg)
{
    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->move_degree(deg);
    }
}

void UR_Stepper::move_steps(int64_t steps)
{
    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->move_steps(steps);
    }
}

void UR_Stepper::set_profile(UR_STEPPER_NAMESPACE::stepper_profile_t profile)
{
    _profile = profile;
}

UR_STEPPER_NAMESPACE::State UR_Stepper::get_current_state(uint8_t backend)
{
    return _backends[backend]->_get_current_state();
}

#endif
