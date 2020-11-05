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
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_MotorsTiltQuad::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 3), // 3 to keep parameters from before deep nesting index fix

    AP_GROUPINFO("COAXIAL", 40, AP_MotorsTiltQuad, _coaxial, 0),

    AP_GROUPINFO("CX1_TRIM", 41, AP_MotorsTiltQuad, _servos[0].coax_trim, 0),
    AP_GROUPINFO("CX2_TRIM", 42, AP_MotorsTiltQuad, _servos[1].coax_trim, 0),
    AP_GROUPINFO("CX3_TRIM", 43, AP_MotorsTiltQuad, _servos[2].coax_trim, 0),
    AP_GROUPINFO("CX4_TRIM", 44, AP_MotorsTiltQuad, _servos[3].coax_trim, 0),

    AP_GROUPINFO("CX_FOLDING", 45, AP_MotorsTiltQuad, _coax_folding, 0),

    // 50 was SV_SPEED
    // 51 was SV_OFFSET

    AP_GROUPINFO("SV1_TRIM", 52, AP_MotorsTiltQuad, _servos[0].trim, 0),
    AP_GROUPINFO("SV2_TRIM", 53, AP_MotorsTiltQuad, _servos[1].trim, 0),
    AP_GROUPINFO("SV3_TRIM", 54, AP_MotorsTiltQuad, _servos[2].trim, 0),
    AP_GROUPINFO("SV4_TRIM", 55, AP_MotorsTiltQuad, _servos[3].trim, 0),

    AP_GROUPINFO("SV_SCALE", 56, AP_MotorsTiltQuad, _servo_scale, 1),

    AP_GROUPINFO("SV_LIMIT", 57, AP_MotorsTiltQuad, _servo_limit, 100),

    AP_GROUPINFO("PITCH_SPEED", 58, AP_MotorsTiltQuad, _pitch_speed, 0),
    AP_GROUPINFO("ELEVON_POWER", 59, AP_MotorsTiltQuad, _elevon_power, 0),

    AP_GROUPEND
};

void AP_MotorsTiltQuad::add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order, float servo_factor)
{
    add_motor(motor_num, angle_degrees, yaw_factor, testing_order);
    _servos[motor_num].factor = servo_factor;
}

// init
void AP_MotorsTiltQuad::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // X frame set-up, yaw factors for plane mode
    add_motor_tq(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1,  1);
    add_motor_tq(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3, -1);
    add_motor_tq(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1);
    add_motor_tq(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2,  1);

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    // enable fast channels or instant pwm    
    set_update_rate(_speed_hz);

    // setup servo mappings
    for (int i = 0; i < 4; i++) {
        _servos[i].chan = SRV_Channels::get_channel_for((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_motor5 + i), CH_9 + i);
        if (!_servos[i].chan) {
            gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTiltQuad: unable to setup output channels");
            // don't set initialised_ok
            return;
        }
        add_motor_num(AP_MOTORS_MOT_5 + i);
    }

    if (_coaxial) {
        // setup coaxial servo mappings
        for (int i = 0; i < 4; i++) {
            if (!SRV_Channels::get_channel_for((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_motor9 + i), CH_5 + i)) {
                gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTiltQuad: unable to setup coaxial output channels");
                // don't set initialised_ok
                return;
            }
            add_motor_num(AP_MOTORS_MOT_9 + i);
        }
    }

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TILTQUAD);
}

void AP_MotorsTiltQuad::set_airspeed(float aspeed)
{
    if (is_positive(_pitch_speed) && aspeed > 10) {
        _thrust_speed_scale = constrain_float((_pitch_speed - aspeed) / _pitch_speed, 0.f, 1.f);
        if (is_zero(_elevon_power)) {
            _thrust_speed_scale = get_constrained_speed_scale();
        }
    } else {
        _thrust_speed_scale = 1.f;
    }

    _elevon_scale = _elevon_power * aspeed * aspeed;
}

// sets motor tilt based on desired r/p/y and current conversion
void AP_MotorsTiltQuad::output_tilt()
{
    limit_tilt = false;

    uint32_t now = AP_HAL::millis();
    float folding_thrust_ratio = 1.0f - constrain_float((float)(now - _last_unfolded_ms) / 2000, 0.0f, 1.0f);

    for (int i = 0; i < 4; i++) {
        bool swing_stab = _in_recovery && i == _recovery_motor;

        uint16_t s = 1000 + (swing_stab ? 0 : _conv) + _servos[i].trim;

        float mot_thrust = _thrust_rpyt_out[i] * _lift_max;
        if (_flags.armed && !is_zero(mot_thrust)) {
            float thrust_vert = _roll_tilt * _roll_factor[i] + _pitch_tilt * _pitch_factor[i];
            float thrust_horiz = _yaw_tilt * _yaw_factor[i];
            if (swing_stab) {
                thrust_vert = _roll_tilt_full * _roll_factor[i] + _pitch_tilt_full * _pitch_factor[i];
                thrust_horiz = 0;
            }
            mot_thrust *= (1.0f + folding_thrust_ratio) * 0.5f;
            mot_thrust = constrain_float(mot_thrust, 0.1f, 1.0f);
            float angle = constrain_float(thrust_vert / (mot_thrust * _thrust_speed_scale + _elevon_scale) - thrust_horiz / mot_thrust, -M_PI_2, M_PI_2);
            int16_t pwm = constrain_int16(angle / M_PI_2 * 1000, -_servo_limit, _servo_limit);
            if (pwm == -_servo_limit || pwm == _servo_limit)
                limit_tilt = true;
            s += pwm;
        }
        else
            limit_tilt = true;

        s = 1500 + (s - 1500) * _servo_scale * _servos[i].factor * (_servos[i].chan->get_reversed() ? -1 : 1);

        rc_write(AP_MOTORS_MOT_5 + i, s);

        if (_coaxial)
            rc_write(AP_MOTORS_MOT_9 + i, s + _servos[i].coax_trim);
    }
}

// output - sends commands to the motors
void AP_MotorsTiltQuad::output()
{
    AP_MotorsMatrix::output();
    output_tilt();

    bool fold_props = _coax_folding && _conv < _coax_folding * 10;
    SRV_Channels::set_output_scaled(SRV_Channel::k_unused1, fold_props ? 100 : 0);
    if (!fold_props)
        _last_unfolded_ms = AP_HAL::millis();
}

void AP_MotorsTiltQuad::output_armed_stabilizing()
{
    AP_MotorsMatrix::output_armed_stabilizing();
    if (_in_recovery) {
        _thrust_rpyt_out[_failed_motor] = 0;
        _thrust_rpyt_out[_recovery_motor] = get_throttle_hover();
    }
}

void AP_MotorsTiltQuad::engage_recovery()
{
    if (_in_recovery)
        return;

    const uint8_t paired_motors[] = {1, 0, 3, 2};

    _in_recovery = true;
    _failed_motor = get_lost_motor();
    _recovery_motor = paired_motors[_failed_motor];

    gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)_failed_motor + 1);
}
