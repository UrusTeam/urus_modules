#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
 */

#include <AP_Math/AP_Math.h>
#include <inttypes.h>
#include <AP_Compass/AP_Compass.h>
#if !HAL_MINIMIZE_FEATURES_AVR
#include <AP_Airspeed/AP_Airspeed.h>
#endif
#if !HAL_MINIMIZE_FEATURES_AVR
#include <AP_Beacon/AP_Beacon.h>
#endif
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#if !HAL_MINIMIZE_FEATURES_AVR
#include <AP_Param/AP_Param.h>
class OpticalFlow;
#endif
#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

enum AHRS_VehicleClass {
    AHRS_VEHICLE_UNKNOWN,
    AHRS_VEHICLE_GROUND,
    AHRS_VEHICLE_COPTER,
    AHRS_VEHICLE_FIXED_WING,
    AHRS_VEHICLE_SUBMARINE,
};


// forward declare view class
#if !HAL_MINIMIZE_FEATURES
class AP_AHRS_View;
#endif

class AP_AHRS
{
public:
#if !HAL_MINIMIZE_FEATURES
    friend class AP_AHRS_View;
#endif

    // Constructor
    AP_AHRS(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps) :
        roll(0.0f),
        pitch(0.0f),
        yaw(0.0f),
        roll_sensor(0),
        pitch_sensor(0),
        yaw_sensor(0),
        _vehicle_class(AHRS_VEHICLE_UNKNOWN),
        _compass(nullptr),
#if !HAL_MINIMIZE_FEATURES_AVR
        _optflow(nullptr),
        _airspeed(nullptr),
        _beacon(nullptr),
#endif
        _compass_last_update(0),
        _ins(ins),
        _baro(baro),
        _gps(gps),
        _cos_roll(1.0f),
        _cos_pitch(1.0f),
        _cos_yaw(1.0f),
        _sin_roll(0.0f),
        _sin_pitch(0.0f),
        _sin_yaw(0.0f),
        _active_accel_instance(0)
    {
        // load default values from var_info table
#if !HAL_MINIMIZE_FEATURES_AVR
        AP_Param::setup_object_defaults(this, var_info);
#else
        gps_gain = 1.0f;
        _gps_use = 1;
        _kp_yaw = 0.2f;
        _kp = 0.2f;
        beta = 0.1f;
        _gps_minsats = 6;
        _ekf_type = 2;
#endif

        // base the ki values by the sensors maximum drift
        // rate.
        _gyro_drift_limit = ins.get_gyro_drift_rate();

        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;

        // initialise _home
        _home.options    = 0;
        _home.alt        = 0;
        _home.lng        = 0;
        _home.lat        = 0;
#if !HAL_MINIMIZE_FEATURES_AVR
        _last_trim = _trim.get();
#else
        _last_trim = _trim;
#endif

        _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, 0.0f);
        _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
    }

    // empty virtual destructor
    virtual ~AP_AHRS() {}

    // init sets up INS board orientation
    virtual void init() {
        set_orientation();
    };

    // Accessors
    void set_fly_forward(bool b) {
        _flags.fly_forward = b;
    }

    bool get_fly_forward(void) const {
        return _flags.fly_forward;
    }

    /*
      set the "likely flying" flag. This is not guaranteed to be
      accurate, but is the vehicle codes best guess as to the whether
      the vehicle is currently flying
     */
    void set_likely_flying(bool b) {
        if (b && !_flags.likely_flying) {
            _last_flying_ms = AP_HAL::millis();
        }
        _flags.likely_flying = b;
    }

    /*
      get the likely flying status. Returns true if the vehicle code
      thinks we are flying at the moment. Not guaranteed to be
      accurate
     */
    bool get_likely_flying(void) const {
        return _flags.likely_flying;
    }

    /*
      return time in milliseconds since likely_flying was set
      true. Returns zero if likely_flying is currently false
    */
    uint32_t get_time_flying_ms(void) const {
        if (!_flags.likely_flying) {
            return 0;
        }
        return AP_HAL::millis() - _last_flying_ms;
    }

    AHRS_VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    void set_vehicle_class(AHRS_VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    void set_wind_estimation(bool b) {
        _flags.wind_estimation = b;
    }

    void set_compass(Compass *compass) {
        _compass = compass;
        set_orientation();
    }

    const Compass* get_compass() const {
        return _compass;
    }
#if !HAL_MINIMIZE_FEATURES_AVR
    void set_optflow(const OpticalFlow *optflow) {
        _optflow = optflow;
    }

    const OpticalFlow* get_optflow() const {
        return _optflow;
    }
#endif
    // allow for runtime change of orientation
    // this makes initial config easier
    void set_orientation() {
#if !HAL_MINIMIZE_FEATURES_AVR
        _ins.set_board_orientation((enum Rotation)_board_orientation.get());
        if (_compass != nullptr) {
            _compass->set_board_orientation((enum Rotation)_board_orientation.get());
        }
#else
        _ins.set_board_orientation((enum Rotation)_board_orientation);
        if (_compass != nullptr) {
            _compass->set_board_orientation((enum Rotation)_board_orientation);
        }
#endif
    }
#if !HAL_MINIMIZE_FEATURES_AVR
    void set_airspeed(AP_Airspeed *airspeed) {
        _airspeed = airspeed;
    }
    void set_beacon(AP_Beacon *beacon) {
        _beacon = beacon;
    }
    const AP_Airspeed *get_airspeed(void) const {
        return _airspeed;
    }
    const AP_Beacon *get_beacon(void) const {
        return _beacon;
    }
#endif
    const AP_GPS &get_gps() const {
        return _gps;
    }

    const AP_InertialSensor &get_ins() const {
        return _ins;
    }

    const AP_Baro &get_baro() const {
        return _baro;
    }

    // get the index of the current primary accelerometer sensor
    virtual uint8_t get_primary_accel_index(void) const {
        return _ins.get_primary_accel();
    }

    // get the index of the current primary gyro sensor
    virtual uint8_t get_primary_gyro_index(void) const {
        return _ins.get_primary_gyro();
    }

    // accelerometer values in the earth frame in m/s/s
    virtual const Vector3f &get_accel_ef(uint8_t i) const {
        return _accel_ef[i];
    }
    virtual const Vector3f &get_accel_ef(void) const {
        return get_accel_ef(_ins.get_primary_accel());
    }

    // blended accelerometer values in the earth frame in m/s/s
    virtual const Vector3f &get_accel_ef_blended(void) const {
        return _accel_ef_blended;
    }

    // get yaw rate in earth frame in radians/sec
    float get_yaw_rate_earth(void) const {
        return get_gyro() * get_rotation_body_to_ned().c;
    }

    // Methods
    virtual void update(bool skip_ins_update=false) = 0;

    // report any reason for why the backend is refusing to initialise
    virtual const char *prearm_failure_reason(void) const {
        return nullptr;
    }

    // is the EKF backend doing its own sensor logging?
    virtual bool have_ekf_logging(void) const {
        return false;
    }

    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    // return a smoothed and corrected gyro vector
    virtual const Vector3f &get_gyro(void) const = 0;

    // return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
    Vector3f get_gyro_latest(void) const;

    // return the current estimate of the gyro drift
    virtual const Vector3f &get_gyro_drift(void) const = 0;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    virtual void reset_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset(bool recover_eulers=false) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset_attitude(const float &roll, const float &pitch, const float &yaw) = 0;

    // return the average size of the roll/pitch error estimate
    // since last call
    virtual float get_error_rp(void) const = 0;

    // return the average size of the yaw error estimate
    // since last call
    virtual float get_error_yaw(void) const = 0;

    // return a DCM rotation matrix representing our current
    // attitude
    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;
    const Matrix3f& get_rotation_autopilot_body_to_vehicle_body(void) const { return _rotation_autopilot_body_to_vehicle_body; }
    const Matrix3f& get_rotation_vehicle_body_to_autopilot_body(void) const { return _rotation_vehicle_body_to_autopilot_body; }

    // get our current position estimate. Return true if a position is available,
    // otherwise false. This call fills in lat, lng and alt
    virtual bool get_position(struct Location &loc) const = 0;

    // return a wind estimation vector, in m/s
    virtual Vector3f wind_estimate(void) = 0;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    virtual bool airspeed_estimate(float *airspeed_ret) const;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool airspeed_estimate_true(float *airspeed_ret) const {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        *airspeed_ret *= get_EAS2TAS();
        return true;
    }
    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
#if !HAL_MINIMIZE_FEATURES_AVR
        if (_airspeed) {
            return _airspeed->get_EAS2TAS();
        }
#endif
        return 1.0f;
    }
    // return true if airspeed comes from an airspeed sensor, as
    // opposed to an IMU estimate
    bool airspeed_sensor_enabled(void) const {
        return false;//_airspeed != nullptr && _airspeed->use() && _airspeed->healthy();
    }

    // return a ground vector estimate in meters/second, in North/East order
    virtual Vector2f groundspeed_vector(void);

    // return a ground velocity in meters/second, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_velocity_NED(Vector3f &vec) const {
        return false;
    }

    // returns the expected NED magnetic field
    virtual bool get_expected_mag_field_NED(Vector3f &ret) const {
        return false;
    }

    // returns the estimated magnetic field offsets in body frame
    virtual bool get_mag_field_correction(Vector3f &ret) const {
        return false;
    }

    // return a position relative to home in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_relative_position_NED_home(Vector3f &vec) const {
        return false;
    }

    // return a position relative to origin in meters, North/East/Down
    // order. This will only be accurate if have_inertial_nav() is
    // true
    virtual bool get_relative_position_NED_origin(Vector3f &vec) const {
        return false;
    }
    // return a position relative to home in meters, North/East
    // order. Return true if estimate is valid
    virtual bool get_relative_position_NE_home(Vector2f &vecNE) const {
        return false;
    }

    // return a position relative to origin in meters, North/East
    // order. Return true if estimate is valid
    virtual bool get_relative_position_NE_origin(Vector2f &vecNE) const {
        return false;
    }

    // return a Down position relative to home in meters
    // if EKF is unavailable will return the baro altitude
    virtual void get_relative_position_D_home(float &posD) const = 0;

    // return a Down position relative to origin in meters
    // Return true if estimate is valid
    virtual bool get_relative_position_D_origin(float &posD) const {
        return false;
    }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) {
        return groundspeed_vector().length();
    }

    // return true if we will use compass for yaw
    virtual bool use_compass(void) {
        return _compass && _compass->use_for_yaw();
    }

    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
    }

    // set the correct centrifugal flag
    // allows arducopter to disable corrections when disarmed
    void set_correct_centrifugal(bool setting) {
        _flags.correct_centrifugal = setting;
    }

    // get the correct centrifugal flag
    bool get_correct_centrifugal(void) const {
        return _flags.correct_centrifugal;
    }

    // get trim
    const Vector3f &get_trim() const {
#if !HAL_MINIMIZE_FEATURES_AVR
        return _trim.get();
#else
        return _trim;
#endif
    }

    // set trim
    virtual void            set_trim(Vector3f new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    virtual void            add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // helper trig value accessors
    float cos_roll() const  {
        return _cos_roll;
    }
    float cos_pitch() const {
        return _cos_pitch;
    }
    float cos_yaw() const   {
        return _cos_yaw;
    }
    float sin_roll() const  {
        return _sin_roll;
    }
    float sin_pitch() const {
        return _sin_pitch;
    }
    float sin_yaw() const   {
        return _sin_yaw;
    }

    // for holding parameters
#if !HAL_MINIMIZE_FEATURES_AVR
    static const struct AP_Param::GroupInfo var_info[] PROGMEM;
#endif

    // return secondary attitude solution if available, as eulers in radians
    virtual bool get_secondary_attitude(Vector3f &eulers) {
        return false;
    }

    // return secondary attitude solution if available, as quaternion
    virtual bool get_secondary_quaternion(Quaternion &quat) {
        return false;
    }

    // return secondary position solution if available
    virtual bool get_secondary_position(struct Location &loc) {
        return false;
    }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const struct Location &get_home(void) const {
        return _home;
    }

    // set the home location in 10e7 degrees. This should be called
    // when the vehicle is at this position. It is assumed that the
    // current barometer and GPS altitudes correspond to this altitude
    virtual void set_home(const Location &loc) = 0;

    // set the EKF's origin location in 10e7 degrees.  This should only
    // be called when the EKF has no absolute position reference (i.e. GPS)
    // from which to decide the origin on its own
    virtual bool set_origin(const Location &loc) { return false; }

    // returns the inertial navigation origin in lat/lon/alt
    virtual bool get_origin(Location &ret) const { return false; }

    // return true if the AHRS object supports inertial navigation,
    // with very accurate position and velocity
    virtual bool have_inertial_nav(void) const {
        return false;
    }

    // return the active accelerometer instance
    uint8_t get_active_accel_instance(void) const {
        return _active_accel_instance;
    }

    // is the AHRS subsystem healthy?
    virtual bool healthy(void) const = 0;

    // true if the AHRS has completed initialisation
    virtual bool initialised(void) const {
        return true;
    };

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    virtual uint32_t getLastYawResetAngle(float &yawAng) const {
        return 0;
    };

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) const {
        return 0;
    };

    // return the amount of NE velocity change in metres/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const {
        return 0;
    };

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosDownReset(float &posDelta) const {
        return 0;
    };

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    virtual bool resetHeightDatum(void) {
        return false;
    }

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const {
        return false;
    }

    // time that the AHRS has been up
    virtual uint32_t uptime_ms(void) const = 0;

    // get the selected ekf type, for allocation decisions
    int8_t get_ekf_type(void) const {
        return _ekf_type;
    }

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    virtual void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const { ret.zero(); _ins.get_delta_velocity(ret); dt = _ins.get_delta_velocity_dt(); }

    // create a view
#if !HAL_MINIMIZE_FEATURES
    AP_AHRS_View *create_view(enum Rotation rotation);
#endif

    // return calculated AOA
    float getAOA(void);

    // return calculated SSA
    float getSSA(void);

    virtual void update_AOA_SSA(void);

    // get_hgt_ctrl_limit - get maximum height to be observed by the
    // control loops in meters and a validity flag.  It will return
    // false when no limiting is required
    virtual bool get_hgt_ctrl_limit(float &limit) const { return false; };

protected:
    AHRS_VehicleClass _vehicle_class;

    // settable parameters
    // these are public for ArduCopter
    float _kp_yaw;
    float _kp;
    float gps_gain;

    float beta;
    int8_t _gps_use;
    int8_t _wind_max;
    int8_t _board_orientation;
    int8_t _gps_minsats;
    int8_t _gps_delay;
    int8_t _ekf_type;

    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
        uint8_t likely_flying           : 1;    // 1 if vehicle is probably flying
    } _flags;

    // time when likely_flying last went true
    uint32_t _last_flying_ms;

    // calculate sin/cos of roll/pitch/yaw from rotation
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // pointer to compass object, if available
    Compass         * _compass;

#if !HAL_MINIMIZE_FEATURES_AVR
    // pointer to OpticalFlow object, if available
    const OpticalFlow *_optflow;

    // pointer to airspeed object, if available
    AP_Airspeed     * _airspeed;

    // pointer to beacon object, if available
    AP_Beacon     * _beacon;
#endif
    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.
    AP_InertialSensor   &_ins;
    AP_Baro             &_baro;
    const AP_GPS        &_gps;

    // a vector to capture the difference between the controller and body frames
#if !HAL_MINIMIZE_FEATURES
    AP_Vector3f         _trim;
#else
    Vector3f         _trim;
#endif

    // cached trim rotations
    Vector3f _last_trim;
    Matrix3f _rotation_autopilot_body_to_vehicle_body;
    Matrix3f _rotation_vehicle_body_to_autopilot_body;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    Vector3f        _accel_ef_blended;

    // Declare filter states for HPF and LPF used by complementary
    // filter in AP_AHRS::groundspeed_vector
    Vector2f _lp; // ground vector low-pass filter
    Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input

    // reference position for NED positions
    struct Location _home;

    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;

    // optional view class
#if !HAL_MINIMIZE_FEATURES
    AP_AHRS_View *_view;
#endif

    // AOA and SSA
    float _AOA, _SSA;
    uint32_t _last_AOA_update_ms;
};

#include "AP_AHRS_DCM.h"
#include "AP_AHRS_NavEKF.h"

#if AP_AHRS_NAVEKF_AVAILABLE
#define AP_AHRS_TYPE AP_AHRS_NavEKF
#else
#define AP_AHRS_TYPE AP_AHRS
#endif
