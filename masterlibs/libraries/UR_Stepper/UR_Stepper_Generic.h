#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Stepper.h"
#include "UR_Stepper_Backend.h"

using namespace UR_STEPPER_NAMESPACE;

class UR_Stepper_Generic : public UR_Stepper_Backend
{
public:

    ~UR_Stepper_Generic();

    /** see setup_process function on backend class
      */
    void setup_process(UR_Stepper::ProcessMode process_mode) override;

    /** see update function on top class
      */
    void update() override;

    /** see configure function on top class
      */
    static UR_Stepper_Backend *configure(UR_Stepper &ur_stepper);

    void move_steps(int32_t steps, bool force) override;
    void move_degree(float deg, bool force) override;

private:
    UR_Stepper_Generic(UR_Stepper &ur_stepper);

    bool _configure();

    bool _auto_process = false;
    int32_t _steps;
    uint32_t _last_action_end;
    //uint32_t _next_action_interval;
#if LINEAR_SPEED_ENABLED == 1
    float _rest;
    float _step_count;        // current position
#endif // LINEAR_SPEED_ENABLED
    int32_t _steps_remaining;   // to complete the current move (absolute value)
#if LINEAR_SPEED_ENABLED == 1
    float _steps_to_cruise;   // steps to reach cruising (max) rpm
    float _steps_to_brake;    // steps needed to come to a full stop
#endif // LINEAR_SPEED_ENABLED
    float _step_pulse;        // step pulse duration (microseconds)
    float _cruise_step_pulse; // step pulse duration for constant speed section (max rpm)

    void _delay_micros(uint32_t delay_us, uint32_t start_us = 0);

    void _steps_loop(void);
    void _set_step_dir(STEP_DIR dir);
    void _update_step_pulse() override;
    void _start_move(int32_t steps);
    void _next_action(void);
    void _calc_step_pulse(void);
    State _get_current_state(void) override;

    /*
     * Calculate steps needed to rotate requested angle, given in degrees
     */
    float _calc_steps_for_rotation(float deg);

};

#endif
