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

#include "AP_AccelCal.h"
#include <stdarg.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>

#define AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS 1000

const extern AP_HAL::HAL& hal;
static bool _start_collect_sample;
uint8_t AP_AccelCal::_num_clients = 0;

AP_AccelCal_Client* AP_AccelCal::_clients[AP_ACCELCAL_MAX_NUM_CLIENTS] {};

void AP_AccelCal::update()
{
    if (!get_calibrator(0)) {
        return;
    }
    if (_started) {
        update_status();
        AccelCalibrator *cal;
        uint8_t num_active_calibrators = 0;
        for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
            num_active_calibrators++;
        }
        if (num_active_calibrators != _num_active_calibrators) {
            fail();
            return;
        }
        if(_start_collect_sample) {
            collect_sample();
        }
        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {

                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                uint8_t step;
                if ((cal = get_calibrator(0)) == nullptr) {
                    fail();
                    return;
                }
                step = cal->get_num_samples_collected()+1;

                for(uint8_t i=1 ; (cal = get_calibrator(i))  ; i++) {
                    if (step != cal->get_num_samples_collected()+1) {
                        fail();
                        return;
                    }
                }

                // if we're on a new step, print a message describing the step
                if (step != _step) {
                    _step = step;

                    const char *msg;
                    switch (step) {
                        case 1:
                            msg = "level";
                            break;
                        case 2:
                            msg = "on its LEFT side";
                            break;
                        case 3:
                            msg = "on its RIGHT side";
                            break;
                        case 4:
                            msg = "nose DOWN";
                            break;
                        case 5:
                            msg = "nose UP";
                            break;
                        case 6:
                            msg = "on its BACK";
                            break;
                        default:
                            fail();
                            return;
                    }
                    hal.console->printf("Place vehicle %s and press any key.\n", msg);
                 }

                gcs_vehicle_position((float)step);
                break;
            }
            case ACCEL_CAL_COLLECTING_SAMPLE:
                // check for timeout

                for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
                    cal->check_for_timeout();
                }

                update_status();

                if (_status == ACCEL_CAL_FAILED) {
                    fail();
                }
                return;
            case ACCEL_CAL_SUCCESS:
                // save
                if (_saving) {
                    bool done = true;
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if (client_active(i) && _clients[i]->_acal_get_saving()) {
                            done = false;
                            break;
                        }
                    }
                    if (done) {
                        success();
                    }
                    return;
                } else {
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i) && _clients[i]->_acal_get_fail()) {
                            fail();
                            return;
                        }
                    }
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i)) {
                            _clients[i]->_acal_save_calibrations();
                        }
                    }
                    _saving = true;
                }
                return;
            default:
            case ACCEL_CAL_FAILED:
                fail();
                return;
        }
    } else if (_last_result != ACCEL_CAL_NOT_STARTED) {
        // only continuously report if we have ever completed a calibration
        switch (_last_result) {
            case ACCEL_CAL_SUCCESS:
                gcs_vehicle_position(ACCELCAL_VEHICLE_POS_SUCCESS);
                hal.console->printf("POS SUCCESS\n");
                break;
            case ACCEL_CAL_FAILED:
                gcs_vehicle_position(ACCELCAL_VEHICLE_POS_FAILED);
                hal.console->printf("POS FAILED\n");
                break;
            default:
                // should never hit this state
                break;
        }
    }
}

void AP_AccelCal::start(AP_HAL::UARTDriver *gcs)
{
    if (gcs == nullptr || _started) {
        return;
    }

    _start_collect_sample = false;
    _num_active_calibrators = 0;

    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        cal->clear();
        cal->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5f);
        _num_active_calibrators++;
    }

    _started = true;
    _saving = false;
    _gcs = gcs;
    _use_gcs_snoop = true;
    _last_position_request_ms = 0;
    _step = 0;

    _last_result = ACCEL_CAL_NOT_STARTED;

    update_status();
}

void AP_AccelCal::success()
{
    hal.console->printf("Cal successful\n");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_success();
    }

    _last_result = ACCEL_CAL_SUCCESS;

    clear();
}

void AP_AccelCal::cancel()
{
    hal.console->printf("Cal cancelled\n");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_cancellation();
    }

    _last_result = ACCEL_CAL_NOT_STARTED;

    clear();
}

void AP_AccelCal::fail()
{
    hal.console->printf("Cal FAILED\n");

    for(uint8_t i=0 ; i < _num_clients ; i++) {
        _clients[i]->_acal_event_failure();
    }

    _last_result = ACCEL_CAL_FAILED;

    clear();
}

void AP_AccelCal::clear()
{
    if (!_started) {
        return;
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->clear();
    }

    _step = 0;
    _started = false;
    _saving = false;

    update_status();
}

void AP_AccelCal::collect_sample()
{
    if (_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        return;
    }

    for(uint8_t i=0; i<_num_clients; i++) {
        if (client_active(i) && !_clients[i]->_acal_get_ready_to_sample()) {
            hal.console->printf("Not ready to sample\n");
            return;
        }
    }

    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->collect_sample();
    }
    // setup snooping of packets so we can see the COMMAND_ACK
    _start_collect_sample = false;
    update_status();
}

void AP_AccelCal::register_client(AP_AccelCal_Client* client) {
    if (client == nullptr || _num_clients >= AP_ACCELCAL_MAX_NUM_CLIENTS) {
        return;
    }


    for(uint8_t i=0; i<_num_clients; i++) {
        if(_clients[i] == client) {
            return;
        }
    }
    _clients[_num_clients] = client;
    _num_clients++;
}

AccelCalibrator* AP_AccelCal::get_calibrator(uint8_t index) {
    AccelCalibrator* ret;
    for(uint8_t i=0; i<_num_clients; i++) {
        for(uint8_t j=0 ; (ret = _clients[i]->_acal_get_calibrator(j)) ; j++) {
            if (index == 0) {
                return ret;
            }
            index--;
        }
    }
    return nullptr;
}

void AP_AccelCal::update_status() {
    AccelCalibrator *cal;

    if (!get_calibrator(0)) {
        // no calibrators
        _status = ACCEL_CAL_NOT_STARTED;

        return;
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_FAILED) {
            _status = ACCEL_CAL_FAILED;         //fail if even one of the calibration has
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            _status = ACCEL_CAL_COLLECTING_SAMPLE;          // move to Collecting sample state if all the callibrators have
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            _status = ACCEL_CAL_WAITING_FOR_ORIENTATION;    // move to waiting for user ack for orientation confirmation
            return;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_NOT_STARTED) {
            _status = ACCEL_CAL_NOT_STARTED;    // we haven't started if all the calibrators haven't
            return;
        }
    }

    _status = ACCEL_CAL_SUCCESS;    // we have succeeded calibration if all the calibrators have
    return;
}

bool AP_AccelCal::client_active(uint8_t client_num)
{
    return (bool)_clients[client_num]->_acal_get_calibrator(0);
}

bool AP_AccelCal::gcs_vehicle_position(float position)
{
    _use_gcs_snoop = false;

    if (_status == ACCEL_CAL_WAITING_FOR_ORIENTATION && is_equal((float) _step, position)) {
        _start_collect_sample = true;
        return true;
    }

    return false;
}
