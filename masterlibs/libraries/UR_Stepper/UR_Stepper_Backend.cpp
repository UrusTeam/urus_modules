#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper_Backend.h"

UR_Stepper_Backend::UR_Stepper_Backend(UR_Stepper &ur_stepper) :
    _ur_stepper(ur_stepper)
{}

void UR_Stepper_Backend::setup_process(UR_Stepper::ProcessMode process_mode)
{}

void UR_Stepper_Backend::update()
{}

void UR_Stepper_Backend::move_degree(int64_t deg)
{}

void UR_Stepper_Backend::move_steps(int64_t steps)
{}

#endif
