/*
   URUS Synthetic Barometer driver.
   Copyright (c) 2017, 2019 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS
#include "AP_Baro_Backend.h"
#include "AP_Math/definitions.h"

class AP_Baro_URUS : public AP_Baro_Backend
{
public:
    AP_Baro_URUS(AP_Baro &baro);
    void update(void);

    void init(void);

    float calculate_qnh(float alt_qnh, float pressure_qnh, float temp, uint8_t alt_qnh_unit) override;
    float calculate_qfe(float alt_qfe, float pressure_qfe, float temp, uint8_t alt_qfe_unit) override;
    float get_altitude_difference(float base_pressure, float pressure, float temp) const override;

private:
    uint8_t _instance;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
