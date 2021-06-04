#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

class HAL_URUS : public AP_HAL::HAL {
public:
    HAL_URUS();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
    void rcout_init();
    void rcin_init();
    void analogin_init();
    void i2c_init();
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
