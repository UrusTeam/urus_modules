#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper_Generic.h"
#include <math.h>
#if CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN
#include <unistd.h>
#endif // CONFIG_SHAL_CORE

#define STEP_PULSE(steps, microsteps, rpm) (60.0*1000000L/steps/microsteps/rpm)

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
    _set_step_dir(STEP_DIR::DIR_CW);

    return true;
}

void UR_Stepper_Generic::move_steps(int64_t steps)
{
    STEP_DIR dir_state = (steps >= 0) ? STEP_DIR::DIR_CW : STEP_DIR::DIR_CCW;
    _set_step_dir(dir_state);
    _steps = steps;
}

void UR_Stepper_Generic::move_degree(int64_t deg)
{
    move_steps(_calc_steps_for_rotation(deg));
}

int64_t UR_Stepper_Generic::_calc_steps_for_rotation(int64_t deg)
{
    return (int64_t)(deg * _profile.steps * _profile.microsteps / 360);
}

void UR_Stepper_Generic::_steps_loop(void)
{
    _next_action();

    if (_steps_remaining <= 0) {
        _start_move(_steps);
    } else {
        _steps = 0;
    }
}

void UR_Stepper_Generic::_set_step_dir(STEP_DIR dir)
{
    hal.gpio->write(_profile.dir_pin, dir);
}

void UR_Stepper_Generic::_start_move(int64_t steps)
{
    float speed;
    _last_action_end = 0;
    _steps_remaining = labs(steps);
    _step_count = 0;
    _rest = 0;
    switch (_profile.mode) {
    case LINEAR_SPEED:
        // speed is in [steps/s]
        speed = _profile.rpm * _profile.steps / 60.0;
        // how many microsteps from 0 to target speed
        _steps_to_cruise = _profile.microsteps * (speed * speed / (2.0 * _profile.accel));
        // how many microsteps are needed from cruise speed to a full stop
        _steps_to_brake = _steps_to_cruise * _profile.accel / _profile.decel;
        if (_steps_remaining < _steps_to_cruise + _steps_to_brake){
            // cannot reach max speed, will need to brake early
            _steps_to_cruise = _steps_remaining * _profile.decel / (_profile.accel + _profile.decel);
            _steps_to_brake = _steps_remaining - _steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        _step_pulse = 1e+6 * 0.676 * sqrt(2.0 / _profile.accel / _profile.microsteps);
        // Save cruise timing since we will no longer have the calculated target speed later
        _cruise_step_pulse = 1e+6 / speed / _profile.microsteps;
        break;

    case CONSTANT_SPEED:
    default:
        _steps_to_cruise = 0;
        _steps_to_brake = 0;
        _cruise_step_pulse = STEP_PULSE(_profile.steps, _profile.microsteps, _profile.rpm);
        _step_pulse = _cruise_step_pulse;
    }
}

int64_t UR_Stepper_Generic::_next_action(void)
{
    if (_steps_remaining > 0) {
        _delay_micros((uint32_t)_next_action_interval, (uint32_t)_last_action_end);

        hal.gpio->write(_profile.step_pin, HIGH);

        float ms = AP_HAL::micros();
        float pulse = _step_pulse; // save value because _calc_step_pulse() will overwrite it
        _calc_step_pulse();
        // We should pull HIGH for at least 1-2us (step_high_min)
        _delay_micros(1);
        hal.gpio->write(_profile.step_pin, LOW);
        // account for _calc_step_pulse() execution time; sets ceiling for max rpm on slower MCUs
        _last_action_end = AP_HAL::micros();
        ms = _last_action_end - ms;
        _next_action_interval = (pulse > ms) ? pulse - ms : 1.0;
    } else {
        // end of move
        _last_action_end = 0;
        _next_action_interval = 0;
    }

    return _next_action_interval;
}

void UR_Stepper_Generic::_calc_step_pulse(void)
{
    if (_steps_remaining <= 0) {  // this should not happen, but avoids strange calculations
        return;
    }

    _steps_remaining--;
    _step_count++;

    if (_profile.mode == LINEAR_SPEED) {
        switch (_get_current_state()) {
        case ACCELERATING:
            if (_step_count < _steps_to_cruise){
                _step_pulse = (_step_pulse - (2.0 * _step_pulse + _rest) / (4.0 * _step_count + 1.0));
                _rest = (_step_count < _steps_to_cruise) ? (int64_t)(2.0 * _step_pulse + _rest) % (int64_t)(4.0 * _step_count + 1.0) : 0.0;
            } else {
                // The series approximates target, set the final value to what it should be instead
                _step_pulse = _cruise_step_pulse;
            }
            break;

        case DECELERATING:
            _step_pulse = _step_pulse - (2.0 * _step_pulse + _rest) / (-4.0 * _steps_remaining + 1.0);
            _rest = (int64_t)(2.0 * _step_pulse + _rest) % (int64_t)(-4.0 * _steps_remaining + 1.0);
            break;

        default:
            break; // no speed changes
        }
    }
}

UR_STEPPER_NAMESPACE::State UR_Stepper_Generic::_get_current_state(void)
{
    enum State state;
    if (_steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (_steps_remaining <= _steps_to_brake){
            state = DECELERATING;
        } else if (_step_count <= _steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
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
        while (AP_HAL::micros() - start_us < delay_us);
#endif // CONFIG_SHAL_CORE
    }
}

#endif
