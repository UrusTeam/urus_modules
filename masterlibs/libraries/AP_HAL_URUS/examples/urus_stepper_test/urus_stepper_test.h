#pragma once

/**
    This is the urus_stepper_test header main object class
    and it's the standard urus's app initialization structure
    model.
    Only setup() and loop() is required to run your app.
*/

#include <AP_HAL/AP_HAL.h>
#include <UR_Stepper/UR_Stepper.h>

class CLurus_stepper_test {
public:

    /** This setup is called only one time at startup
      * on the frontend.
      * you can configure or setup the IO environments.
      * After setup loop is called forever.
      */
    void setup(void);

    /** This loop is called everytime on the main loop
      * frontend.
      */
    void loop(void);

private:

    uint8_t _cnt_sw;
    uint16_t _cnt_loop;
};
