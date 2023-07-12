/*
  JoyPad init example:
  Hiroshi Takey, December 2017.
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Joypad/AP_Joypad.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Joypad joypad;

uint8_t data_array_buttons[4] = {0};
bool button1 = false;

void setup()
{
    joypad.init();
    hal.gpio->write(17, 1);
}

void loop()
{
    data_array_buttons[0] = ~data_array_buttons[0] & (button1 << 1);
    button1 = !button1;
    hal.scheduler->suspend_timer_procs();
    joypad.set_button_array_data(data_array_buttons, 1);
    hal.scheduler->resume_timer_procs();
    hal.gpio->toggle(17);
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
