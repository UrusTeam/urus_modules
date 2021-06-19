#pragma once

/**
    This is the urus_rot_encoder_test header main object class
    and it's the standard urus's app initialization structure
    model.
    Only setup() and loop() is required to run your app.
*/

#include <AP_HAL/AP_HAL.h>

class CLurus_rot_encoder_test {
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
    void update(void);

private:

};
