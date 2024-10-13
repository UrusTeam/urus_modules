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
#include <UR_Protocol/utility/UR_RingBuffer.h>

#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
#include <avr/interrupt.h>
#endif

#include <AP_Math/AP_Math.h>

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

//volatile UR_Rotary_Encoder::rot_enc_state_t UR_Rotary_Encoder::_rotenc_state;
//UR_Rotary_Encoder::flipflop_t UR_Rotary_Encoder::_flipflop = {.clock = 0};
//volatile uint32_t UR_Rotary_Encoder::_uspstp = 0;
//volatile UR_Rotary_Encoder::rot_enc_conf_t UR_Rotary_Encoder::_rotenc_conf;
//UR_Rotary_Encoder *UR_Rotary_Encoder::_this_rotary;
const int8_t UR_Rotary_Encoder::_rot_table[] = {0, 1, -1, 1, 1, 0, 1, -1,};

//static volatile rbuf_t buffer;
/*
volatile int8_t _oldState = 0;
volatile long _position = 0;        // Internal position (4 times _positionExt)
volatile long _positionExt = 0;     // External position
unsigned long _positionExtTimePrev = 0; // The time the previous position change was detected.
unsigned long _positionExtTime = 0;     // The time the last position change was detected.

const int8_t KNOBDIR[] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};
#define LATCH0 0 // input state at position 0
#define LATCH3 3 // input state at position 3
*/

//static volatile uint8_t dat = 0;
//static volatile uint8_t _oldState = 0;
//static volatile int8_t position = 0;
//static volatile int8_t _positionExt = 0;

UR_Rotary_Encoder::UR_Rotary_Encoder()
{
    _rotenc_conf.pin_a = ROT_PIN_A;
    _rotenc_conf.pin_b = ROT_PIN_B;
    _rotenc_conf.tick_mask = TICK_MASK;
    _rotenc_conf.max_steps_cnt = MAX_STEPS_CNT;
    _rotenc_conf.step_per_rpm = STEPS_PER_RPM;
    _rotenc_conf.min_rpm = MIN_RPM;

    _rotenc_state.step_cnt = 0;
    //_this_rotary = this;
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
    int16_t step_tmp = 0;
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
    //uint8_t oldSREG = SREG;
	//cli();
	step_tmp = _rotenc_state.step_cnt;
	//SREG = oldSREG;

    DIR_ROT encstate = DIR_ROT::ROT_IDLE;
    if (_last_step_cnt > step_tmp) {
        _last_step_cnt = step_tmp;
        encstate = DIR_ROT::ROT_CCW;
    } else if (_last_step_cnt < step_tmp) {
        _last_step_cnt = step_tmp;
        encstate = DIR_ROT::ROT_CW;
    }

	return encstate;
#else
    return _rotenc_state.dir_enc;
#endif
}

bool UR_Rotary_Encoder::get_step_status()
{
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
    //uint8_t oldSREG = SREG;
	//cli();
#endif
    bool status = _rotenc_state.step_status;
    //bool status = !ringbuf_empty((rbuf_t*)&buffer);
    if (status) {
        _rotenc_state.step_status = false;
    }
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
    //SREG = oldSREG;
#endif
    return status;
}

int16_t UR_Rotary_Encoder::get_step_val()
{
#if (CONFIG_SHAL_CORE == SHAL_CORE_APM)
    //rbuf_t *buft = (rbuf_t*)&buffer;
    //buf_item_t elem;
    int16_t positiontmp;
    //uint8_t oldSREG = SREG;
	//cli();
	//elem = ringbuf_get((rbuf_t*)&buffer);
	//step = _rotenc_state.step_cnt;
	positiontmp = _rotenc_state.step_cnt;
	//SREG = oldSREG;
	//uint16_t step = elem.time;
	if (_rotenc_conf.tick_mask == 0x00) {
        return positiontmp >> 1;
	}
	return positiontmp >> 2;
#else
    return _rotenc_state.step_cnt;
#endif
}

void UR_Rotary_Encoder::set_step_val(int16_t val)
{
    _rotenc_state.step_cnt = val;
}

uint8_t UR_Rotary_Encoder::_read_pins()
{
    uint8_t _st_pina = hal.gpio->read(_rotenc_conf.pin_a);
    uint8_t _st_pinb = hal.gpio->read(_rotenc_conf.pin_b);

    if (_rotenc_state.invert_dir == 0x02) {
        uint8_t aux_pin = _st_pina;
        _st_pina = _st_pinb;
        _st_pinb = aux_pin;
    }

    return _st_pina | (_st_pinb << 1);
}

bool UR_Rotary_Encoder::_configure()
{
    hal.gpio->pinMode(_rotenc_conf.pin_b, HAL_GPIO_INPUT);
    hal.gpio->pinMode(_rotenc_conf.pin_a, HAL_GPIO_INPUT);
    hal.gpio->write(_rotenc_conf.pin_b, HIGH);
    hal.gpio->write(_rotenc_conf.pin_a, HIGH);

    _rotenc_state.st_pin_last = _read_pins();

/*
    bool _st_pina = hal.gpio->read(_rotenc_conf.pin_b);
    bool _st_pinb = hal.gpio->read(_rotenc_conf.pin_a);

    _rotenc_state.st_pina_last = _st_pina;

    uint8_t _pin_state[2] = {0, 0};

    _pin_state[_flipflop.clock++] = (uint8_t)_st_pina << 0 |
                                    (uint8_t)_st_pinb << 1;

    _rotenc_state.tick_state += _pin_state[0] + _flipflop.clock;
*/
    return true;
}

void UR_Rotary_Encoder::interrupt_flipflop()
{
    //uint8_t _st_pina = hal.gpio->read(_rotenc_conf.pin_a);
    //uint8_t _st_pinb = hal.gpio->read(_rotenc_conf.pin_b);

    uint8_t pin_dat = _read_pins();

    if (_rotenc_state.st_pin_last == pin_dat) {
        return;
    }

    //hal.console->printf("%d-%d\n", pin_dat, _rotenc_state.st_pin_last);
    //if (_rotenc_state.st_pin_last != pin_dat) {
        _rotenc_state.step_cnt = _rotenc_state.step_cnt + _rot_table[_rotenc_state.st_pin_last | (pin_dat << 1)];
        _rotenc_state.st_pin_last = pin_dat;

        if (pin_dat == 0x03) {
            _uspstp = _rotenc_state.delta_uspstp;
            _rotenc_state.delta_uspstp = AP_HAL::micros();
            _rotenc_state.step_status = true;
            //hal.console->printf_PS(PSTR("%d*%d\n"), pin_dat, position);
            //hal.console->printf_PS(PSTR("%d\n"), position);
            //position += KNOBDIR[_oldState];
            //_positionExt = position >> 2;
            //return;
        }
        if ((pin_dat == 0x00) && (_rotenc_conf.tick_mask == 0x00)) {
            _rotenc_state.step_status = true;
        }
        //_rotenc_state.st_pin_last = pin_dat;
    //}

/*
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
*/
/*
        buf_item_t b_item;
        b_item.time = _rotenc_state.step_cnt;
        ringbuf_put((rbuf_t*)&buffer, b_item);
*/
/*
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
*/
}

void UR_Rotary_Encoder::configure(ProcessMode process_mode)
{
    _configure();
    //ringbuf_init((rbuf_t*)&buffer);
}

void UR_Rotary_Encoder::update()
{
    interrupt_flipflop();
}

uint32_t UR_Rotary_Encoder::get_rpm()
{
    uint32_t deltaus = 0;
    uint32_t uspstp = 0;

    //uint8_t oldSREG = SREG;
	//cli();
	deltaus = _rotenc_state.delta_uspstp;
	uspstp = _uspstp;
	//SREG = oldSREG;

    if (uspstp == 0) {
        uspstp = 1;
    }

    uint32_t difft = deltaus - uspstp;
    if (difft < (uint32_t)CALC_MINIMUM_RPM(_rotenc_conf.min_rpm, _rotenc_conf.step_per_rpm)) {
        return (uint32_t)USPSTP_TOFROM_RPM(difft, _rotenc_conf.step_per_rpm);
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
