#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_URUS/CORE_URUS/CORE_URUS.h>

#include <UR_Stepper/UR_Stepper.h>

#include "urus_stepper_test.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static CLurus_stepper_test urus_stepper_test_app;

UR_Stepper *ur_stepper;

UR_STEPPER_NAMESPACE::stepper_profile_t stepper_profile;

void CLurus_stepper_test::setup(void)
{
    if (!hal.console->is_initialized()) {
        return;
    }

    hal.console->printf("\nStarting urus_stepper_test_app...\n");

    stepper_profile.mode = UR_STEPPER_NAMESPACE::SPEED_MODE::LINEAR_SPEED;
    stepper_profile.accel = 1000;
    stepper_profile.decel = 400;
    stepper_profile.dir_pin = 9;
    stepper_profile.step_pin = 10;
    stepper_profile.microsteps = 1;
    stepper_profile.rpm = 200;
    stepper_profile.steps = 200;

    ur_stepper = new UR_Stepper(stepper_profile);

    ur_stepper->configure();
    ur_stepper->move_degree(360);

    _cnt_sw = 0;
    _cnt_loop = 0;
}

void CLurus_stepper_test::loop(void)
{
    if (_cnt_loop > 5) {
        return;
    }

    _cnt_sw = _cnt_sw % 2;

    hal.scheduler->delay(10);

    if (ur_stepper->get_current_state() == UR_STEPPER_NAMESPACE::STOPPED && _cnt_sw == 0) {
        stepper_profile.mode = UR_STEPPER_NAMESPACE::SPEED_MODE::LINEAR_SPEED;
        stepper_profile.accel = 400;
        stepper_profile.decel = 100;
        ur_stepper->set_profile(stepper_profile);
        ur_stepper->move_degree(360);
        _cnt_sw++;
        _cnt_loop++;
    }

    hal.scheduler->delay(10);

    if (ur_stepper->get_current_state() == UR_STEPPER_NAMESPACE::STOPPED && _cnt_sw == 1) {
        stepper_profile.mode = UR_STEPPER_NAMESPACE::SPEED_MODE::LINEAR_SPEED;
        stepper_profile.accel = 100;
        stepper_profile.decel = 400;

        ur_stepper->set_profile(stepper_profile);
        ur_stepper->move_degree(360 * 8);
        _cnt_sw++;
        _cnt_loop++;
    }
}

void setup(void);
void loop(void);

void setup(void)
{
    urus_stepper_test_app.setup();
}

void loop(void)
{
    urus_stepper_test_app.loop();
}

AP_HAL_MAIN();
