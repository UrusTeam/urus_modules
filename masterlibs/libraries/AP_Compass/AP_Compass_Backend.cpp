#include <AP_HAL/AP_HAL.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

extern const AP_HAL::HAL& hal;

AP_Compass_Backend::AP_Compass_Backend(Compass &compass) :
    _compass(compass)
{
    _sem = hal.util->new_semaphore();
}

void AP_Compass_Backend::rotate_field(Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];
    mag.rotate(MAG_BOARD_ORIENTATION);
    mag.rotate(state.rotation);

    if (!state.external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        mag.rotate(_compass._board_orientation);
    } else {
        // add user selectable orientation
#if !HAL_MINIMIZE_FEATURES_AVR
        mag.rotate((enum Rotation)state.orientation.get());
#else
        mag.rotate((enum Rotation)state.orientation);
#endif
    }
}

void AP_Compass_Backend::publish_raw_field(const Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    state.last_update_ms = AP_HAL::millis();
#if !HAL_MINIMIZE_FEATURES
    _compass._calibrator[instance].new_sample(mag);
#endif
}

void AP_Compass_Backend::correct_field(Vector3f &mag, uint8_t i)
{
    Compass::mag_state &state = _compass._state[i];

#if !HAL_MINIMIZE_FEATURES_AVR
    if (state.diagonals.get().is_zero()) {
        state.diagonals.set(Vector3f(1.0f,1.0f,1.0f));
    }

    const Vector3f &offsets = state.offset.get();
    const Vector3f &diagonals = state.diagonals.get();
    const Vector3f &offdiagonals = state.offdiagonals.get();
    const Vector3f &mot = state.motor_compensation.get();
#else
    if (state.diagonals.is_zero()) {
        state.diagonals = Vector3f(1.0f,1.0f,1.0f);
    }

    const Vector3f &offsets = state.offset;
    const Vector3f &diagonals = state.diagonals;
    const Vector3f &offdiagonals = state.offdiagonals;
    const Vector3f &mot = state.motor_compensation;
#endif

    /*
     * note that _motor_offset[] is kept even if compensation is not
     * being applied so it can be logged correctly
     */
    mag += offsets;
    if(_compass._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && !is_zero(_compass._thr_or_curr)) {
        state.motor_offset = mot * _compass._thr_or_curr;
        mag += state.motor_offset;
    } else {
        state.motor_offset.zero();
    }

    Matrix3f mat(
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x,    diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z,    diagonals.z
    );

    mag = mat * mag;
}

/*
  copy latest data to the frontend from a backend
 */
void AP_Compass_Backend::publish_filtered_field(const Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    state.field = mag;

    state.last_update_ms = AP_HAL::millis();
    state.last_update_usec = AP_HAL::micros();
}

void AP_Compass_Backend::set_last_update_usec(uint32_t last_update, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];
    state.last_update_usec = last_update;
}

/*
  register a new backend with frontend, returning instance which
  should be used in publish_field()
 */
uint8_t AP_Compass_Backend::register_compass(void) const
{
    return _compass.register_compass();
}


/*
  set dev_id for an instance
*/
void AP_Compass_Backend::set_dev_id(uint8_t instance, uint32_t dev_id)
{
#if !HAL_MINIMIZE_FEATURES_AVR
    _compass._state[instance].dev_id.set_and_notify(dev_id);
#else
    _compass._state[instance].dev_id = dev_id;
#endif
}

/*
  set external for an instance
*/
void AP_Compass_Backend::set_external(uint8_t instance, bool external)
{
    if (_compass._state[instance].external != 2) {
#if !HAL_MINIMIZE_FEATURES_AVR
        _compass._state[instance].external.set_and_notify(external);
#else
        _compass._state[instance].external = external;
#endif
    }
}

bool AP_Compass_Backend::is_external(uint8_t instance)
{
    return _compass._state[instance].external;
}

// set rotation of an instance
void AP_Compass_Backend::set_rotation(uint8_t instance, enum Rotation rotation)
{
    _compass._state[instance].rotation = rotation;
}
