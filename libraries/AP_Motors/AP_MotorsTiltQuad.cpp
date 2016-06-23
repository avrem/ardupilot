// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_MotorsTiltQuad.cpp - motor matrix for Aeroxo tilt quad
 *
 */

#include "AP_MotorsTiltQuad.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// setup_motors - configures the motors for a tiltquad
void AP_MotorsTiltQuad::setup_motors()
{
    _conv = 1000;

    // call parent
    AP_MotorsMatrix::setup_motors();

    // X frame set-up
    add_motor(AP_MOTORS_MOT_1,   45, 0, 1);
    add_motor(AP_MOTORS_MOT_2, -135, 0, 3);
    add_motor(AP_MOTORS_MOT_3,  -45, 0, 4);
    add_motor(AP_MOTORS_MOT_4,  135, 0, 2);

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();
}

// set update rate to motors - a value in hertz
void AP_MotorsTiltQuad::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 4 motors and 4 servos
    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _speed_hz);

    uint32_t mask2 =
        1U << 8 |
        1U << 9 |
        1U << 10 |
        1U << 11;
    hal.rcout->set_freq(mask2, 50);    
}
