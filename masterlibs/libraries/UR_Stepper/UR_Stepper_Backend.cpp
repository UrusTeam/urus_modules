#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper_Backend.h"

UR_Stepper_Backend::UR_Stepper_Backend(UR_Stepper &ur_stepper) :
    _ur_stepper(ur_stepper),
    _profile(_ur_stepper._profile)
{}

void UR_Stepper_Backend::setup_process(UR_Stepper::ProcessMode process_mode)
{}

void UR_Stepper_Backend::update()
{}

void UR_Stepper_Backend::move_degree(float deg)
{}

void UR_Stepper_Backend::move_steps(int32_t steps)
{}

UR_STEPPER_NAMESPACE::State UR_Stepper_Backend::_get_current_state(void)
{
    return UR_STEPPER_NAMESPACE::State::STOPPED;
}

#endif
