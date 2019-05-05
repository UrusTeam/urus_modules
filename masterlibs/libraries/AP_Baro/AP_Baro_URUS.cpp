#include "AP_Baro_URUS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS
#include <AP_HAL_URUS/AP_HAL_URUS.h>

AP_Baro_URUS::AP_Baro_URUS(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

// Read the sensor
void AP_Baro_URUS::update(void)
{
    _copy_to_frontend(_instance, 96646, 23);
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
