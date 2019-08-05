#pragma once

#include <cmath>

#include <AP_HAL/AP_HAL.h>

#ifdef M_PI
# undef M_PI
#endif
#define M_PI      (3.141592653589793f)

#ifdef M_PI_2
# undef M_PI_2
#endif
#define M_PI_2    (M_PI / 2)

#define M_GOLDEN  1.6180339f

#define M_2PI         (M_PI * 2)

// MATH_CHECK_INDEXES modifies some objects (e.g. SoloGimbalEKF) to
// include more debug information.  It is also used by some functions
// to add extra code for debugging purposes. If you wish to activate
// this, do it here or as part of the top-level Makefile -
// e.g. Tools/Replay/Makefile
#ifndef MATH_CHECK_INDEXES
  #define MATH_CHECK_INDEXES 0
#endif

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M     0.01113195f
#define LATLON_TO_CM    1.113195f

// Semi-major axis of the Earth, in meters.
static const double WGS84_A = 6378137.0;

//Inverse flattening of the Earth
static const double WGS84_IF = 298.257223563;

// The flattening of the Earth
static const double WGS84_F = ((double)1.0 / WGS84_IF);

// Semi-minor axis of the Earth in meters
static const double WGS84_B = (WGS84_A * (1 - WGS84_F));

// Eccentricity of the Earth
static const double WGS84_E = (sqrt(2 * WGS84_F - WGS84_F * WGS84_F));

/*
  use AP_ prefix to prevent conflict with OS headers, such as NuttX
  clock.h
 */
#define AP_NSEC_PER_SEC   1000000000ULL
#define AP_NSEC_PER_USEC  1000ULL
#define AP_USEC_PER_SEC   1000000ULL
#define AP_USEC_PER_MSEC  1000ULL
#define AP_MSEC_PER_SEC   1000ULL
#define AP_SEC_PER_WEEK   (7ULL * 86400ULL)
#define AP_MSEC_PER_WEEK  (AP_SEC_PER_WEEK * AP_MSEC_PER_SEC)


#define INHG_TO_PA      3386.383613f
#define METERS_TO_FEET  3.280839895f

namespace ISA_MATH_CONST {

    const float p0      = 101325.0f;                    // [N/m^2] = [Pa]
    const float p1      = 22632.05545875171f;           // [N/m^2] = [Pa] Calculated @ 11000.0000000000001 [ft]
    const float p2      = 5474.884659730908f;           // [N/m^2] = [Pa] Calculated @ 20000.000000000001  [ft]
    const float p3      = 868.0176477556424f;           // [N/m^2] = [Pa] Calculated @ 32000.000000000001  [ft]

    const float p0hPa   = 1013.25f;                     // [hPa]
    const float p0inHG  = 29.9213f;                     // [inHG] Truncated 29.9213
    const float p0mmHG  = p0inHG * 25.4f;               // [mmHG] w/ 25.4 [mm] = 1.0 [in]

    const float T0      = 288.15f;                      // [K]
    const float T1      = 216.65f;                      // [K]
    const float T2      = 216.65f;                      // [K]

    const float h1      = 36089.238845144355f;          // [ft] = 11000 [m] w/ CftTOm
    const float h2      = 65616.79790026246f;           // [ft] = 20000 [m] w/ CftTOm
    const float h3      = 104986.87664041994f;          // [ft] = 32000 [m] w/ CftTOm

    const float dTdh0   = -0.0019812f;                  // [K/ft] w/ CftTOm
    const float dTdh0SI = -0.0065f;                     // [K/m]

    const float dTdh2   = 0.0003048f;                   // [K/ft] w/ CftTOm
    const float dTdh2SI = 0.001f;                       // [K/m]

    const float CPascalTOPSI   = 1.45037737730209e-04f;
    const float ChPaTOinHG     = p0inHG / p0hPa;
    const float ChPaTOmmHG     = p0mmHG / p0hPa;

    const float ClbPft3TOkgPm3 = 16.0184633739601f;     // [lb/ft^3] to [kg/m^3]

    const float CftTOm     = 0.3048f;
    const float CftTOnm    = 1.64578833693305e-04f;

    const float CnmTOm     = 1852.0f;

    const float CftPsTOkn  = CftTOnm * 3600.0f;
    const float CftPsTOmph = 3600.0f / 5280.0f;
    const float CftPsTOkph = CftTOm * 3600.0f / 1000.0f;

    const float CmPsTOkn   = 3600.0f / CnmTOm;

    const float CknTOftPs  = 1.0f / (CftTOnm * 3600.0f);

    const float CRGasSI    = 287.053f;                  // [m^2/(s^2*K)] = [J/(kg*K)]

    const float CgSI       = 9.80665f;                  // [m/s^2]

    const float CgRGas     = (CgSI * CftTOm) / CRGasSI;
    const float CgRGasSI   = CgSI / CRGasSI;

    const float CGamma     = 1.4f;    // [-]
    const float CGammaRGas = (CGamma * CRGasSI) / (CftTOm * CftTOm);    // [ft^2/(s^2*K)]

    const float CaSLSI         = sqrt(CGamma * CRGasSI * T0);
    const float CPressureSLSI  = 101325.0f;                             // [Pa] = [N/m^2]
    const float CaSLNU         = CaSLSI * CmPsTOkn;                     // [kts] Nautical Unit
    const float CRhoSLSI       = 1.225f;                                // [kg/m^3]

    const float CKelvinTOCelsius   = 273.15f;
    const float CKelvinTORankine   = 1.8f;

    const float CCelsiusTOFahrenheitLinear       =  32.0f;
    const float CCelsiusTOFahrenheitProportional =  1.8f;
};

