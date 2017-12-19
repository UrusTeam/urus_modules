#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup() {
     hal.scheduler->delay(5000);
     hal.console->println_P(PSTR("Empty setup"));
}
void loop() 
{
     hal.console->println_P(PSTR("loop"));
     hal.console->printf("hello %d\n", 1234);
     hal.scheduler->delay(1000);
}

AP_HAL_MAIN();

