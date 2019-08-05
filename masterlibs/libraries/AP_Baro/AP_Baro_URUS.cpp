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

#include "AP_Baro_URUS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS
#include <AP_HAL_URUS/AP_HAL_URUS.h>
#include <AP_Math/definitions.h>

using namespace ISA_MATH_CONST;

AP_Baro_URUS::AP_Baro_URUS(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
#if HAL_BARO_DEFAULT == HAL_BARO_URUS
    _instance = _frontend.register_sensor();
#endif // HAL_BARO_DEFAULT
}

void AP_Baro_URUS::init(void)
{
}

// Get the calculation altitude differente based on th Layer 0 > Troposphere.
float AP_Baro_URUS::get_altitude_difference(float base_pressure, float pressure, float temp) const
{
    float ret;
    float temptmp    = T0 + temp - 15;
    float scaling = base_pressure / pressure;

    ret = CftTOm * (temptmp * (powf(scaling, (dTdh0SI / CgRGasSI)) - 1.0f) / dTdh0);

    return ret;
}

// -----------------------------------------------------------------------------
// Calculate QNH by hPa and feet by default.
// [1] Meteorology Specification A2669 Version 4.00 Page 100
float AP_Baro_URUS::calculate_qnh(float alt_qnh, float pressure_qnh, float temp, uint8_t alt_qnh_unit)
{
    uint8_t pQNHUnit = 0;
    uint8_t resultQNHUnit = 0;
    float resultQNH = 0;
    float hQFE = 0;
    float pQFE = 0;

    switch (alt_qnh_unit) {
    case 0:                         // [ft] feet
        hQFE = alt_qnh * CftTOm;
        break;
    case 1:                         // [m]  meters
        hQFE = alt_qnh;
        break;
    default:
        ;
    }

    switch (pQNHUnit) {
    case 0:                         // [hPa]    hectoPascal
        pQFE = pressure_qnh;
        break;
    case 1:                         // [inHg]   inches mercury
        pQFE = pressure_qnh / ChPaTOinHG;
        break;
    case 2:                         // [mmHg]   millimiters mercury
        pQFE = pressure_qnh / ChPaTOmmHG;
        break;
    default:
        ;
    }

    // Calculation(s)
    pressure_qnh = p0hPa * powf(powf((pQFE / p0hPa), (-1 * (CRGasSI * dTdh0SI)) / CgSI) - (hQFE * dTdh0SI) / (T0 + temp - 15), (-1 * CgSI) / (CRGasSI * dTdh0SI));

    // Unit Conversion(s)
    switch (resultQNHUnit) {
    case 0:                         // [hPa]    hectoPascal
        break;
    case 1:                         // [inHg]   inches mercury
        pressure_qnh = pressure_qnh * ChPaTOinHG;
        break;
    case 2:                         // [mmHg]   millimiters mercury
        pressure_qnh = pressure_qnh * ChPaTOmmHG;
        break;
    default:
        ;
    }

    resultQNH = pressure_qnh;
    return resultQNH;
}

// Calculate QFE by hPa and feet by default.
// [1] Meteorology Specification A2669 Version 4.00 Page 100
float AP_Baro_URUS::calculate_qfe(float alt_qfe, float pressure_qfe, float temp, uint8_t alt_qfe_unit)
{
    uint8_t pQFEUnit = 0;
    uint8_t resultQFEUnit = 0;
    float resultQFE = 0;
    float hQNH = 0;
    float pQNH = 0;

    switch (alt_qfe_unit) {
    case 0:                         // [ft] feet
        hQNH = alt_qfe * CftTOm;
        break;
    case 1:                         // [m]  meters
        hQNH = alt_qfe;
        break;
    default:
        ;
        // INTERNAL ERROR
    }

    switch (pQFEUnit) {
    case 0:                         // [hPa]    hectoPascal
        pQNH = pressure_qfe;
        break;
    case 1:                         // [inHg]   inches mercury
        pQNH = pressure_qfe / ChPaTOinHG;
        break;
    case 2:                         // [mmHg]   millimiters mercury
        pQNH = pressure_qfe / ChPaTOmmHG;
        break;
    default:
        ;
        // INTERNAL ERROR
    }

    // Calculation(s) ------------------------------------------------------------
    alt_qfe = p0hPa * powf(powf((pQNH / p0hPa), (-1 * (CRGasSI * dTdh0SI)) / CgSI) + ((hQNH * dTdh0SI) / (T0 + temp - 15)), (-1 * CgSI) / (CRGasSI * dTdh0SI));

    // Unit Conversion(s) --------------------------------------------------------
    switch (resultQFEUnit) {
    case 0:                         // [hPa]    hectoPascal
        break;
    case 1:                         // [inHg]   inches mercury
        alt_qfe = alt_qfe * ChPaTOinHG;
        break;
    case 2:                         // [mmHg]   millimiters mercury
        alt_qfe = alt_qfe * ChPaTOmmHG;
        break;
    default:
        ;
        // INTERNAL ERROR
    }

    resultQFE = alt_qfe;
    return resultQFE;
}

// Read the sensor
void AP_Baro_URUS::update(void)
{
#if HAL_BARO_DEFAULT == HAL_BARO_URUS
        float rnd_press;
        float rnd_temp;

        rnd_press = rand() % 5 + 1;
        rnd_temp = rand() % 5 + 1;
        rnd_temp = rnd_temp / 10;
        _copy_to_frontend(0, 97660.0 + rnd_press, 28.0 + rnd_temp);
#endif // HAL_BARO_DEFAULT
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
