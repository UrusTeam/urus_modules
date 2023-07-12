#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper.h"

class UR_Stepper_Backend {
public:

    UR_Stepper_Backend(UR_Stepper &ur_stepper);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~UR_Stepper_Backend(void) {}

    /** Process the backend.
      * @param  process_mode:
      *         [AutoProcess] - Update setup_process run in the scheduled
      *         callback. This is the default mode.
      *         [LoopProcess] - Update setup_process run not in scheduled
      *         callback. udpate() need to be called in somewhere
      *         to see the action, otherwise nothing happen.
      * @return None.
      */
    virtual void setup_process(UR_Stepper::ProcessMode process_mode);

    /** See update function on top class.
      */
    virtual void update();
    virtual void move_degree(float deg);
    virtual void move_steps(int32_t steps);
    virtual UR_STEPPER_NAMESPACE::State _get_current_state(void);

protected:
    // access to frontend
    UR_Stepper &_ur_stepper;
    UR_STEPPER_NAMESPACE::stepper_profile_t &_profile;
};

#endif
