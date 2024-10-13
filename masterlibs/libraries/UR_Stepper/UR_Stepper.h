#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#define UR_STEPPER_MAX_BACKENDS 1

#define MOTOR_STEPS         200.0f
#define MOTOR_RPM           200.0f
#define MICROSTEPS          1
#define MOTOR_ACCEL         1000.0f
#define MOTOR_DECEL         1000.0f

#define STEP_PIN        8
#define DIR_PIN         9

namespace UR_STEPPER_NAMESPACE
{
    enum STEP_DIR {
        DIR_CCW = HIGH,
        DIR_CW = LOW
    };

    enum SPEED_MODE {
        CONSTANT_SPEED=0,
        LINEAR_SPEED
    };

    enum State {
        STOPPED=0,
        ACCELERATING,
        CRUISING,
        DECELERATING
    };

    typedef struct __stepper_profile_s {
        SPEED_MODE mode = CONSTANT_SPEED;
        float accel = MOTOR_ACCEL;     // acceleration [steps/s^2]
        float decel = MOTOR_DECEL;     // deceleration [steps/s^2]
        float steps = MOTOR_STEPS;
        uint8_t microsteps = MICROSTEPS;
        float rpm = MOTOR_RPM;
        uint8_t dir_pin = DIR_PIN;
        uint8_t step_pin = STEP_PIN;
    } stepper_profile_t;
}

class UR_Stepper_Backend;

class UR_Stepper {

    friend class UR_Stepper_Backend;

public:

    enum ProcessMode {
        AutoProcess = 0,
        LoopProcess
    };

    UR_Stepper();
    UR_Stepper(UR_STEPPER_NAMESPACE::stepper_profile_t profile);

    /** Wrap only, make some people happy, this will be
      * removed in the future.
      * By default init() call to configure() in auto process
      * @param None.
      * @return None.
      */
    void init() {
        configure();
    }

    /** Configure the backend with their implementations.
      * By default configure() set "auto process" mode on
      * process() backend function.
      * @param  process_mode:
      *         [AutoProcess] - Update process run in the scheduled
      *         callback. This is the default mode.
      *         [LoopProcess] - Update process run not in scheduled
      *         callback. udpate() need to be called in somewhere
      *         to see the action, otherwise nothing happen.
      * @return None.
      */
    void configure(ProcessMode process_mode = ProcessMode::AutoProcess);

    /** Update the backend process.
      * @param  none.
      * @return None.
      */
    void update();
    void move_degree(float deg, bool force = false);
    void move_steps(int32_t steps, bool force = false);
    void set_profile(UR_STEPPER_NAMESPACE::stepper_profile_t profile, uint8_t backend = 0);
    UR_STEPPER_NAMESPACE::State get_current_state(uint8_t backend = 0);

private:

    UR_Stepper_Backend *_backends[UR_STEPPER_MAX_BACKENDS];
    uint8_t _backend_count;
    bool _backends_configuring:1;
    UR_STEPPER_NAMESPACE::stepper_profile_t _profile;

    // load backend drivers
    bool _add_backend(UR_Stepper_Backend *backend);
    void _configure_backends();
    void _configure_backends(ProcessMode process_mode);
};

#endif
