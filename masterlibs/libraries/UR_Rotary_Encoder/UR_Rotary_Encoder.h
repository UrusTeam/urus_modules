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

#pragma once

#include <AP_HAL_URUS/AP_HAL_URUS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

class UR_Rotary_Encoder {
public:

    enum ProcessMode {
        AutoProcess = 0,
        LoopProcess
    };

    enum DIR_ROT {
        ROT_CCW = 0,
        ROT_CW
    };

    /*
     *  Steps and Ticks.-
     *      Steps are the completed transition between ticks.
     *      Ticks are each one transition when sensing the pin state,
     *      on commons rotary encoders this have two ticks when
     *      complete one step, and usually have 40 ticks per RPM, then
     *      40ticks/2ticks = 20Steps per RPM.
     *      Setting tick_mask to 0x00 will parse only one ticks on each
     *      transition making the transition and steps symetric 1:1.
     */
    typedef struct __rot_enc_conf_s {
        uint8_t pin_a = 0;              // main pin, this pin control flipflop clock interruption
        uint8_t pin_b = 0;              // 2nd and alternative
        uint16_t max_steps_cnt = 0;     // max steps counter
        uint16_t step_per_rpm = 0;      // steps per rpm, usually 20 steps on two ticks
                                        // transition and 40 steps on symetric.
        uint8_t tick_mask = 0xFF;       // tick mask, set 0xFF for default conf, set 0x00 for simple
        uint8_t min_rpm = 1;            // set the minimun RPM to return value on get
    } rot_enc_conf_t;

    typedef struct __rot_enc_state_s {
        uint16_t step_cnt = 0;          // step counter state
        DIR_ROT dir_enc = ROT_CW;       // rotation direction state
        bool step_status = false;       // status step state
        uint8_t invert_dir = 0;         // set 0x02 will invert the rotation direction,
                                        // setting 0x00 make normal rotation
        bool st_pina_last = false;      // store last pin state
        uint8_t tick_state = 0;         // the pins ticks interrupt states
        uint32_t delta_uspstp = 0;      // delta microseconds per steps
    } rot_enc_state_t;

    typedef struct __flipflop_s {
       uint8_t clock : 1;               // sense each transition as flipflop clock.
    } flipflop_t;

    UR_Rotary_Encoder();

    /** Wrap only, make some people happy, this will be
      * removed in the future.
      * By default init() call to configure() in auto process
      * @param None.
      * @return None.
      */
    void init() {
        configure();
    }

    /** Configure the backend with their implementations.
      * By default configure() set "auto process" mode on
      * process() backend function.
      * @param  process_mode:
      *         [AutoProcess] - Update process run in the scheduled
      *         callback. This is the default mode.
      *         [LoopProcess] - Update process run not in scheduled
      *         callback. udpate() need to be called in somewhere
      *         to see the action, otherwise nothing happen.
      * @return None.
      */
    void configure(ProcessMode process_mode = ProcessMode::AutoProcess);

    /** Update the backend process.
      * @param  none.
      * @return None.
      */
    void update();

    // this could be ever called on the ISR context or on the loop.
    static void interrupt_flipflop();
    uint16_t get_step_val();
    void set_step_val(uint16_t val);
    bool get_step_status();
    DIR_ROT get_enc_dir();
    void inverted_dir(bool inverted);
    uint32_t get_rpm();
    void set_encoder_state_conf(rot_enc_conf_t encst);

private:

    // rotary encoder state
    static volatile rot_enc_state_t _rotenc_state;
    static volatile rot_enc_conf_t _rotenc_conf;
    static flipflop_t _flipflop;

    // stores last timedelta microseconds per step result.
    static uint32_t _uspstp;

    bool _configure();
};

#endif
