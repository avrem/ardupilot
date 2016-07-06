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

    // X frame set-up, yaw factors for plane mode
    add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
    add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
    add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
    add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);

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
        1U << (servo_offset + 0) |
        1U << (servo_offset + 1) |
        1U << (servo_offset + 2) |
        1U << (servo_offset + 3);
    hal.rcout->set_freq(mask2, 50);    
}

// sets motor tilt based on desired r/p/y and current conversion
void AP_MotorsTiltQuad::output_tilt()
{
    int32_t roll_tilt = constrain_int32(_roll_tilt * 2000, -250, 250);
    int32_t pitch_tilt = _pitch_tilt * 2000;
    int32_t yaw_tilt = constrain_int32(_yaw_tilt * 1000, -166, 166);

    int32_t s1 = 1000 + _conv;
    int32_t s2 = 2000 - _conv;
    int32_t s3 = 2000 - _conv;
    int32_t s4 = 1000 + _conv;

    s1 = constrain_int32(s1, 1000, 2000) + pitch_tilt;
    s2 = constrain_int32(s2, 1000, 2000) + pitch_tilt;
    s3 = constrain_int32(s3, 1000, 2000) - pitch_tilt;
    s4 = constrain_int32(s4, 1000, 2000) - pitch_tilt;

    s1 = constrain_int32(s1, 1000, 2000) - roll_tilt + yaw_tilt;
    s2 = constrain_int32(s2, 1000, 2000) - roll_tilt + yaw_tilt;
    s3 = constrain_int32(s3, 1000, 2000) - roll_tilt + yaw_tilt;
    s4 = constrain_int32(s4, 1000, 2000) - roll_tilt + yaw_tilt;

    hal.rcout->write(servo_offset + 0, s1);
    hal.rcout->write(servo_offset + 1, s2);
    hal.rcout->write(servo_offset + 2, s3);
    hal.rcout->write(servo_offset + 3, s4);
}
