#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper_Generic.h"
#include <math.h>
#if CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN
#include <unistd.h>
#endif // CONFIG_SHAL_CORE

#define STEP_PULSE(steps, microsteps, rpm) ((((60.0f * 1000000.0f) / steps) / microsteps) / rpm)

#define LINEAR_SPEED_ENABLED 0

extern const AP_HAL::HAL& hal;

UR_Stepper_Generic::UR_Stepper_Generic(UR_Stepper &ur_stepper) :
    UR_Stepper_Backend(ur_stepper)
{
}

UR_Stepper_Generic::~UR_Stepper_Generic()
{}

void UR_Stepper_Generic::setup_process(UR_Stepper::ProcessMode process_mode)
{
    switch (process_mode) {
    case UR_Stepper::LoopProcess:
        _auto_process = false;
        break;
    case UR_Stepper::AutoProcess:
        _auto_process = true;
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&UR_Stepper_Generic::_steps_loop, void));
        break;
    default:
        break;
    }

    hal.scheduler->delay(100);
}

void UR_Stepper_Generic::update()
{
    if (_auto_process) {
        return;
    }

    _steps_loop();
}

UR_Stepper_Backend *UR_Stepper_Generic::configure(UR_Stepper &ur_stepper)
{
    UR_Stepper_Generic *stepper_gen = new UR_Stepper_Generic(ur_stepper);

    if (!stepper_gen || !stepper_gen->_configure()) {
        delete stepper_gen;
        return nullptr;
    }

    return stepper_gen;
}

bool UR_Stepper_Generic::_configure()
{
    hal.gpio->pinMode(_profile.step_pin, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(_profile.dir_pin, HAL_GPIO_OUTPUT);

    hal.gpio->write(_profile.step_pin, LOW);
    hal.gpio->write(_profile.dir_pin, LOW);

    _set_step_dir(STEP_DIR::DIR_CW);

    return true;
}

void UR_Stepper_Generic::move_steps(int32_t steps)
{
    STEP_DIR dir_state = (steps > 0) ? STEP_DIR::DIR_CW : STEP_DIR::DIR_CCW;
    if (_steps_remaining <= 0) {
        _set_step_dir(dir_state);
        _steps = steps;
    }
    //if (_steps_remaining > 0) {
        //_steps = _steps + (int64_t)_steps_remaining;
    //}
    //_steps_remaining = 0;
}

void UR_Stepper_Generic::move_degree(float deg)
{
    move_steps((int32_t)_calc_steps_for_rotation(deg));
}

float UR_Stepper_Generic::_calc_steps_for_rotation(float deg)
{
    return deg * _profile.steps * (float)_profile.microsteps / 360.0f;
}

void UR_Stepper_Generic::_steps_loop(void)
{
    _next_action();

    if (_steps_remaining <= 0) {
        if (_steps != 0) {
            _start_move(_steps);
            _steps = 0;
        }
        //_steps = 0;
    } //else {
        //_steps = 0;
    //}
    //_next_action();
}

void UR_Stepper_Generic::_set_step_dir(STEP_DIR dir)
{
    hal.gpio->write(_profile.dir_pin, dir);
}

void UR_Stepper_Generic::_start_move(int32_t steps)
{
/*
    if (steps == 0) {
        return;
    }
*/
    _steps_remaining = (int32_t)labs(steps);
#if LINEAR_SPEED_ENABLED == 1
    _last_action_end = 0;
    _step_count = 0.0f;
    _rest = 0.0f;
#endif // LINEAR_SPEED_ENABLED
    switch (_profile.mode) {
#if LINEAR_SPEED_ENABLED == 1
    case LINEAR_SPEED:
        float speed;
        // speed is in [steps/s]
        speed = _profile.rpm * _profile.steps / 60.0f;
        // how many microsteps from 0 to target speed
        _steps_to_cruise = (float)_profile.microsteps * (speed * speed / (2.0f * _profile.accel));
        // how many microsteps are needed from cruise speed to a full stop
        _steps_to_brake = _steps_to_cruise * _profile.accel / _profile.decel;
        if (_steps_remaining < (_steps_to_cruise + _steps_to_brake)) {
            // cannot reach max speed, will need to brake early
            _steps_to_cruise = _steps_remaining * _profile.decel / (_profile.accel + _profile.decel);
            _steps_to_brake = _steps_remaining - _steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        _step_pulse = 1e+6 * 0.676f * sqrt(2.0f / _profile.accel / (float)_profile.microsteps);
        // Save cruise timing since we will no longer have the calculated target speed later
        _cruise_step_pulse = 1e+6 / speed / (float)_profile.microsteps;
        break;
#endif // LINEAR_SPEED_ENABLED
    case CONSTANT_SPEED:
    default:
#if LINEAR_SPEED_ENABLED == 1
        _steps_to_cruise = 0.0f;
        _steps_to_brake = 0.0f;
#endif // LINEAR_SPEED_ENABLED
        _cruise_step_pulse = (float)STEP_PULSE(_profile.steps, (float)_profile.microsteps, _profile.rpm);
        _step_pulse = _cruise_step_pulse;
    }
}

void UR_Stepper_Generic::_next_action(void)
{
    if (_steps_remaining > 0) {
#if LINEAR_SPEED_ENABLED == 1
        uint32_t ms = AP_HAL::micros();
#endif // LINEAR_SPEED_ENABLED
        /*
        uint32_t start_us = (uint32_t)_last_action_end;
        if (!start_us) {
            start_us = AP_HAL::micros();
        }
        */
#if LINEAR_SPEED_ENABLED == 1
        if ((ms - _last_action_end) < _next_action_interval) {
            //hal.console->printf_PS(PSTR("B %ld*%ld*%ld*%ld\n"), ms, _last_action_end, (uint32_t)_step_pulse, (uint32_t)_steps_remaining);
            return 0;
        }
#endif // LINEAR_SPEED_ENABLED
        //hal.console->printf_PS(PSTR("A %ld*%ld*%ld\n"), ms, _last_action_end, _next_action_interval);

#if LINEAR_SPEED_ENABLED == 1
        uint32_t pulse = (uint32_t)_step_pulse; // save value because _calc_step_pulse() will overwrite it
#endif // LINEAR_SPEED_ENABLED
        _calc_step_pulse();
        //hal.scheduler->delay_microseconds(1500);
        hal.gpio->write(_profile.step_pin, HIGH);
        // We should pull HIGH for at least 1-2us (step_high_min)
        _delay_micros(1);
        //hal.scheduler->delay_microseconds(1);
        hal.gpio->write(_profile.step_pin, LOW);
        //_delay_micros(100);
        // account for _calc_step_pulse() execution time; sets ceiling for max rpm on slower MCUs
#if LINEAR_SPEED_ENABLED == 1
        _last_action_end = AP_HAL::micros();
        uint32_t resms = _last_action_end - ms;
        _next_action_interval = (pulse > resms) ? (uint32_t)(pulse - resms): 1;
#endif // LINEAR_SPEED_ENABLED
    } //else {
        // end of move
        //_last_action_end = 0;
        //_next_action_interval = 0;
        //_steps_remaining = 0;
    //}

    //return _next_action_interval;
}

void UR_Stepper_Generic::_calc_step_pulse(void)
{
    if (_steps_remaining <= 0) {  // this should not happen, but avoids strange calculations
        return;
    }

    _steps_remaining = _steps_remaining - 1;
#if LINEAR_SPEED_ENABLED == 1
    _step_count = _step_count + 1.0f;
    if (_profile.mode == LINEAR_SPEED) {
        switch (_get_current_state()) {
        case ACCELERATING:
            if (_step_count < _steps_to_cruise){
                _step_pulse = (_step_pulse - (2.0f * _step_pulse + _rest) / (4.0f * _step_count + 1.0f));
                _rest = (_step_count < _steps_to_cruise) ? (int32_t)(2.0f * _step_pulse + _rest) % (int32_t)(4.0f * _step_count + 1.0f) : 0.0f;
            } else {
                // The series approximates target, set the final value to what it should be instead
                _step_pulse = _cruise_step_pulse;
            }
            break;

        case DECELERATING:
            _step_pulse = _step_pulse - (2.0f * _step_pulse + _rest) / (-4.0f * _steps_remaining + 1.0f);
            _rest = (int32_t)(2.0f * _step_pulse + _rest) % (int32_t)(-4.0f * _steps_remaining + 1.0f);
            break;

        default:
            break; // no speed changes
        }
    }
#endif // LINEAR_SPEED_ENABLED
}

UR_STEPPER_NAMESPACE::State UR_Stepper_Generic::_get_current_state(void)
{
    enum State state = CRUISING;
    if (_steps_remaining <= 0) {
        state = STOPPED;
    } else {
#if LINEAR_SPEED_ENABLED == 1
        if (_steps_remaining < (int32_t)_steps_to_brake){
            state = DECELERATING;
        } else if (_step_count <= _steps_to_cruise){
            state = ACCELERATING;
        }// else {
            //state = CRUISING;
        //}
#endif // LINEAR_SPEED_ENABLED
    }
    return state;
}

inline void UR_Stepper_Generic::_delay_micros(uint32_t delay_us, uint32_t start_us) {
    if (delay_us) {
        if (!start_us) {
            start_us = AP_HAL::micros();
        }
#if CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN
        usleep(delay_us);
#else
        while ((AP_HAL::micros() - start_us) < delay_us);
#endif // CONFIG_SHAL_CORE
    }
}

#endif
