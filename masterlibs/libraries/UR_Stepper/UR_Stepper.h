#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#define UR_STEPPER_MAX_BACKENDS 1

class UR_Stepper_Backend;

class UR_Stepper {

    friend class UR_Stepper_Backend;

public:

    enum ProcessMode {
        AutoProcess = 0,
        LoopProcess
    };

    UR_Stepper();

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
    void move_degree(int64_t deg);
    void move_steps(int64_t steps);

private:

    UR_Stepper_Backend *_backends[UR_STEPPER_MAX_BACKENDS];
    uint8_t _backend_count;
    bool _backends_configuring:1;

    // load backend drivers
    bool _add_backend(UR_Stepper_Backend *backend);
    void _configure_backends();
    void _configure_backends(ProcessMode process_mode);
};

#endif
