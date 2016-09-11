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

void AP_MotorsTiltQuad::add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order, float rt_factor, float pt_factor, float yt_factor)
{
    add_motor(motor_num, angle_degrees, yaw_factor, testing_order);
    _roll_tilt_factor[motor_num] = rt_factor;
    _pitch_tilt_factor[motor_num] = pt_factor;
    _yaw_tilt_factor[motor_num] = yt_factor;
}

// setup_motors - configures the motors for a tiltquad
void AP_MotorsTiltQuad::setup_motors()
{
    _conv = 1000;

    // call parent
    AP_MotorsMatrix::setup_motors();

    // X frame set-up, yaw factors for plane mode
    add_motor_tq(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1,  1, 1);
    add_motor_tq(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3, -1,  1, 1);
    add_motor_tq(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, -1, 1);
    add_motor_tq(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, -1, 1);

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
    uint16_t s[4];

    // base conversion angles
    s[0] = 1000 + _conv;
    s[1] = 2000 - _conv;
    s[2] = 2000 - _conv;
    s[3] = 1000 + _conv;

    for (int i = 0; i < 4; i++) {
        float throttle = ((float)calc_thrust_to_pwm(_thrust_rpyt_out[i]) - get_pwm_output_min()) / (get_pwm_output_max() - get_pwm_output_min());
        float mot_thrust = throttle * (1 - _thrust_curve_expo) + _thrust_curve_expo * throttle * throttle;
        mot_thrust *= _lift_max;
        if (!is_zero(mot_thrust)) {
            float thrust = _roll_tilt * _roll_tilt_factor[i] + _pitch_tilt * _pitch_tilt_factor[i] + 
                _yaw_tilt * _yaw_tilt_factor[i];
            // as we use thrust vectoring, scale servo angles by motor thrust
            float angle = asinf(constrain_float(thrust / constrain_float(mot_thrust, 0.1f, 1.0f), -1.0f, 1.0f));

            s[i] += constrain_int16(angle / M_PI_2 * 1000, -300, 300);
        }

        hal.rcout->write(servo_offset + i, s[i]);
    }
}
