/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Tiltquad variables and functions

#if FRAME_CONFIG == TILT_QUAD_FRAME

void Copter::tiltquad_throttle_input_slew()
{
    static float f_control_in = -400;
    int16_t pwm = channel_throttle->read();
    if (pwm < 1400 || pwm > 1600)
        f_control_in += (pwm - 1500) * 0.001f;
    if (f_control_in < -400)
        f_control_in = -400;
    channel_throttle->set_pwm(1500 + f_control_in);
}

void Copter::update_tiltquad_conversion()
{    
    float rc_conv = g.rc_8.norm_input_dz();
    if (rc_conv > 0.1)
        p_conversion += (1500 - p_conversion) * 0.01f;
    else if (rc_conv < -0.1)
        p_conversion += (1100 - p_conversion) * 0.01f;
    
    // calculate conversion state
    if (p_conversion < 1100)
        _conv = 0;
    else if (p_conversion > 1500)
        _conv = 1000;
    else {
        _conv = 1000 - (1500 - p_conversion) / 4 * 10;
        _conv = constrain_int16(_conv, 0, 1000);
    }

    attitude_control.set_conversion(_conv);
    motors.set_conversion(_conv);
}

// update_tiltquad_tilt - sets motor tilt based on conversion
void Copter::update_tiltquad_tilt()
{
    int32_t roll_angle2 = attitude_control.aeroxo_rate_bf_to_motor_roll(0) *
        (1000 - _conv) * 2;

    int32_t yaw_angle2 = attitude_control.aeroxo_rate_bf_to_motor_yaw(0) *
        _conv * 20;

    int32_t pitch_angle2 = attitude_control.aeroxo_rate_bf_to_motor_pitch(0) *
        (1000 - _conv) * 2;

    roll_angle2 = constrain_int32(roll_angle2, -250, 250);
    yaw_angle2 = constrain_int32(yaw_angle2, -166, 166);

    int32_t s1 = 1000 + _conv;
    int32_t s2 = 2000 - _conv;
    int32_t s3 = 2000 - _conv;
    int32_t s4 = 1000 + _conv;

    s1 = constrain_int32(s1, 1000, 2000) + pitch_angle2;
    s2 = constrain_int32(s2, 1000, 2000) + pitch_angle2;
    s3 = constrain_int32(s3, 1000, 2000) - pitch_angle2;
    s4 = constrain_int32(s4, 1000, 2000) - pitch_angle2;

    s1 = constrain_int32(s1, 1000, 2000) - roll_angle2 + yaw_angle2;
    s2 = constrain_int32(s2, 1000, 2000) - roll_angle2 + yaw_angle2;
    s3 = constrain_int32(s3, 1000, 2000) - roll_angle2 + yaw_angle2;
    s4 = constrain_int32(s4, 1000, 2000) - roll_angle2 + yaw_angle2;

    const int servo_offset = 8; // on navio2 servos start from output #9
    hal.rcout->write(servo_offset + 0, s1);
    hal.rcout->write(servo_offset + 1, s2);
    hal.rcout->write(servo_offset + 2, s3);
    hal.rcout->write(servo_offset + 3, s4);
}

#endif // FRAME_CONFIG == TILT_QUAD_FRAME
