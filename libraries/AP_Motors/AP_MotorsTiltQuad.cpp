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


const AP_Param::GroupInfo AP_MotorsTiltQuad::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)
    // parameters 50 ~ 59 for tilt-quad

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed in hz
    // @Values: 50, 125, 250
    AP_GROUPINFO("SV_SPEED", 50, AP_MotorsTiltQuad, _servo_speed, 50),

    AP_GROUPINFO("SV_OFFSET", 51, AP_MotorsTiltQuad, _servo_offset, 8),

    AP_GROUPINFO("SV1_TRIM", 52, AP_MotorsTiltQuad, _servo_trim[0], 0),
    AP_GROUPINFO("SV2_TRIM", 53, AP_MotorsTiltQuad, _servo_trim[1], 0),
    AP_GROUPINFO("SV3_TRIM", 54, AP_MotorsTiltQuad, _servo_trim[2], 0),
    AP_GROUPINFO("SV4_TRIM", 55, AP_MotorsTiltQuad, _servo_trim[3], 0),

    AP_GROUPINFO("SV_SCALE", 56, AP_MotorsTiltQuad, _servo_scale, 1),

    AP_GROUPINFO("SV_LIMIT", 57, AP_MotorsTiltQuad, _servo_limit, 100),

    AP_GROUPEND
};

void AP_MotorsTiltQuad::add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order, float servo_factor)
{
    add_motor(motor_num, angle_degrees, yaw_factor, testing_order);
    _servo_factor[motor_num] = servo_factor;
}

// setup_motors - configures the motors for a tiltquad
void AP_MotorsTiltQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // X frame set-up, yaw factors for plane mode
    add_motor_tq(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1,  1);
    add_motor_tq(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3, -1);
    add_motor_tq(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1);
    add_motor_tq(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2,  1);

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
        1U << (_servo_offset + 0) |
        1U << (_servo_offset + 1) |
        1U << (_servo_offset + 2) |
        1U << (_servo_offset + 3);
    hal.rcout->set_freq(mask2, _servo_speed);    
}

// sets motor tilt based on desired r/p/y and current conversion
void AP_MotorsTiltQuad::output_tilt()
{
    uint16_t s[4];

    limit_tilt = false;

    for (int i = 0; i < 4; i++) {
        s[i] = 1000 + _conv + _servo_trim[i];

        float throttle = ((float)calc_thrust_to_pwm(_thrust_rpyt_out[i]) - get_pwm_output_min()) / (get_pwm_output_max() - get_pwm_output_min());
        float mot_thrust = throttle * (1 - _thrust_curve_expo) + _thrust_curve_expo * throttle * throttle;
        mot_thrust *= _lift_max;
        if (_flags.armed && !is_zero(mot_thrust)) {
            float thrust = _roll_tilt * _roll_factor[i] + _pitch_tilt * _pitch_factor[i] - 
                _yaw_tilt * _yaw_factor[i];
            // as we use thrust vectoring, scale servo angles by motor thrust
            float angle = asinf(constrain_float(thrust / constrain_float(mot_thrust, 0.1f, 1.0f), -1.0f, 1.0f));
            int16_t pwm = constrain_int16(angle / M_PI_2 * 1000, -_servo_limit, _servo_limit);
            if (pwm == -_servo_limit || pwm == _servo_limit)
                limit_tilt = true;
            s[i] += pwm;
        }
        else
            limit_tilt = true;

        s[i] = 1500 + (s[i] - 1500) * _servo_scale * _servo_factor[i];

        hal.rcout->write(_servo_offset + i, s[i]);
    }

    hal.rcout->write(7, _conv); // telemetry feedback
}

// return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
float AP_MotorsTiltQuad::get_current_limit_max_throttle()
{
    return AP_MotorsMulticopter::get_current_limit_max_throttle() * _thr_max;
}

void AP_MotorsTiltQuad::output_to_motors()
{
    float spin_max_temp = _spin_max;

    // apply temporary spin limit and clear
    if (_spin_limit < 1.0f) {
        _spin_max = _spin_min + (_spin_max - _spin_min) * _spin_limit;
        _spin_limit = 1.0f;
    }

    AP_MotorsMatrix::output_to_motors();

    _spin_max = spin_max_temp;
}
