/*
   High precision rotary encoder interface.
   Copyright (c) 2021 Hiroshi Takey <htakey@gmail.com>

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

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "UR_Rotary_Encoder.h"

#define ROT_PIN_A   2
#define ROT_PIN_B   4

#define TICK_MASK       0x14
#define MAX_STEPS_CNT   0xFF

#define MIN_RPM                         30LLU
#define STEPS_PER_RPM                   20LLU
#define ONE_MINUTE_USEC                 60LLU * 1000LLU * 1000LLU
#define USPSTP_TOFROM_RPM(x, stp)       (ONE_MINUTE_USEC / stp) / x
#define CALC_MINIMUM_RPM(rpm, stprpm)   USPSTP_TOFROM_RPM(rpm, stprpm)

extern const AP_HAL::HAL& hal;

UR_Rotary_Encoder::rot_enc_state_t UR_Rotary_Encoder::_rotenc_state;
UR_Rotary_Encoder::flipflop_t UR_Rotary_Encoder::_flipflop = {.clock = 0};
uint32_t UR_Rotary_Encoder::_uspstp = 0;
UR_Rotary_Encoder::rot_enc_conf_t UR_Rotary_Encoder::_rotenc_conf;

UR_Rotary_Encoder::UR_Rotary_Encoder()
{
    _rotenc_conf.pin_a = ROT_PIN_A;
    _rotenc_conf.pin_b = ROT_PIN_B;
    _rotenc_conf.tick_mask = TICK_MASK;
    _rotenc_conf.max_steps_cnt = MAX_STEPS_CNT;
    _rotenc_conf.step_per_rpm = STEPS_PER_RPM;
    _rotenc_conf.min_rpm = MIN_RPM;

    _rotenc_state.step_cnt = 0;
}

void UR_Rotary_Encoder::inverted_dir(bool inverted)
{
    if (inverted) {
        _rotenc_state.invert_dir = 0x02;
    } else {
        _rotenc_state.invert_dir = 0x00;
    }
}

UR_Rotary_Encoder::DIR_ROT UR_Rotary_Encoder::get_enc_dir()
{
    return _rotenc_state.dir_enc;
}

bool UR_Rotary_Encoder::get_step_status()
{
    bool status = _rotenc_state.step_status;
    if (status) {
        _rotenc_state.step_status = false;
    }

    return status;
}

uint16_t UR_Rotary_Encoder::get_step_val()
{
    return _rotenc_state.step_cnt;
}

void UR_Rotary_Encoder::set_step_val(uint16_t val)
{
    _rotenc_state.step_cnt = val;
}

bool UR_Rotary_Encoder::_configure()
{
    hal.gpio->pinMode(_rotenc_conf.pin_b, HAL_GPIO_INPUT);
    hal.gpio->pinMode(_rotenc_conf.pin_a, HAL_GPIO_INPUT);
    hal.gpio->write(_rotenc_conf.pin_b, HIGH);
    hal.gpio->write(_rotenc_conf.pin_a, HIGH);

    return true;
}

void UR_Rotary_Encoder::interrupt_flipflop()
{
    bool _st_pina = hal.gpio->read(_rotenc_conf.pin_a);
    bool _st_pinb = hal.gpio->read(_rotenc_conf.pin_b);

    if (_rotenc_state.st_pina_last == _st_pina) {
        return;
    }

    _rotenc_state.st_pina_last = _st_pina;

    uint8_t _pin_state[2] = {0, 0};

    _pin_state[_flipflop.clock++] = (uint8_t)_st_pina << 0 |
                                    (uint8_t)_st_pinb << 1;

    _rotenc_state.tick_state += _pin_state[0] + _flipflop.clock;

    // parse each tick transition (see header file comments brief).
    if ((_rotenc_conf.tick_mask >> _rotenc_state.tick_state) & 0x01) {
        _rotenc_state.tick_state = _rotenc_state.tick_state << _rotenc_state.invert_dir;
        switch (_rotenc_state.tick_state) {
            case 0x10:
            case 0x02: {
                if (_rotenc_state.step_cnt < _rotenc_conf.max_steps_cnt) {
                    _rotenc_state.dir_enc = DIR_ROT::ROT_CW;
                    _rotenc_state.step_cnt++;
                }
                break;
            }
            case 0x08:
            case 0x04: {
                if (_rotenc_state.step_cnt > 0) {
                    _rotenc_state.dir_enc = DIR_ROT::ROT_CCW;
                    _rotenc_state.step_cnt--;
                }
                break;
            }
            default:
                ;
        }

        _uspstp = AP_HAL::micros() - _rotenc_state.delta_uspstp;
        _rotenc_state.delta_uspstp = AP_HAL::micros();
        _rotenc_state.step_status = true;
    }

    if (_rotenc_conf.tick_mask == 0x00) {
        if (_rotenc_state.step_cnt >= _rotenc_conf.max_steps_cnt) {
            _rotenc_state.step_cnt = 0;
        }
        _rotenc_state.step_cnt++;
        _uspstp = AP_HAL::micros() - _rotenc_state.delta_uspstp;
        _rotenc_state.delta_uspstp = AP_HAL::micros();
        _rotenc_state.step_status = true;
    }

    if (_flipflop.clock) {
        _rotenc_state.tick_state = 0;
    }
}

void UR_Rotary_Encoder::configure(ProcessMode process_mode)
{
    _configure();
}

void UR_Rotary_Encoder::update()
{
    interrupt_flipflop();
}

uint32_t UR_Rotary_Encoder::get_rpm()
{
    if (_uspstp == 0) {
        _uspstp = 1;
    }

    if (_uspstp < (uint32_t)CALC_MINIMUM_RPM(_rotenc_conf.min_rpm, _rotenc_conf.step_per_rpm)) {
        return (uint32_t)USPSTP_TOFROM_RPM(_uspstp, _rotenc_conf.step_per_rpm);
    }

    return 0;
}

void UR_Rotary_Encoder::set_encoder_state_conf(rot_enc_conf_t encst)
{
    _rotenc_conf.pin_a = encst.pin_a > 0 ? encst.pin_a : ROT_PIN_A;
    _rotenc_conf.pin_b = encst.pin_b > 0 ? encst.pin_b : ROT_PIN_B;
    _rotenc_conf.max_steps_cnt = encst.max_steps_cnt > 0 ? encst.max_steps_cnt : MAX_STEPS_CNT;
    _rotenc_conf.tick_mask = encst.tick_mask == 0xFF ? TICK_MASK : encst.tick_mask;
    _rotenc_conf.step_per_rpm = encst.step_per_rpm > 0 ? encst.step_per_rpm : STEPS_PER_RPM;
    _rotenc_conf.min_rpm = encst.min_rpm > 0 ? encst.min_rpm : MIN_RPM;
}

#endif
