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

// setup_motors - configures the motors for a tiltquad
void AP_MotorsTiltQuad::setup_motors()
{
    _conv = 1000;

    // call parent
    AP_MotorsMatrix::setup_motors();

    // X frame set-up
    add_motor_tq(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
    add_motor_tq(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
    add_motor_tq(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4);
    add_motor_tq(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);

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
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6 |
        1U << AP_MOTORS_MOT_7 |
        1U << AP_MOTORS_MOT_8;
    rc_set_freq(mask2, 50);
}

void AP_MotorsTiltQuad::add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    // call raw motor set-up method
     add_motor_raw(
        motor_num,
        cosf(radians(angle_degrees + 90))*sqrt(2),               // roll factor
        cosf(radians(angle_degrees))*sqrt(2),                    // pitch factor
        yaw_factor,                                      // yaw factor
        testing_order);
}
