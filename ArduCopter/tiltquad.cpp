/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Tiltquad variables and functions

#if FRAME_CONFIG == TILT_QUAD_FRAME

void Copter::update_tiltquad_conversion()
{
    const long CONV_THROTTLE = 1500;

    // If signal is more than 1700 (for noise protecting) return.
    if ( hal.rcout->read(7) > CONV_THROTTLE+200 )
    {

        p_conversion+=( 1500 - p_conversion)*0.1f;
    }
    else
    {
        // if 1500-1700 - stop.
        // if <1500 - proceed.
        if ( hal.rcout->read(7) < CONV_THROTTLE )
        {
            //10.10.2014
            if (p_conversion >  hal.rcout->read(7)  )
            {
                p_conversion+=( hal.rcout->read(7) - p_conversion)*0.1f;
            }
        }
    }  

    // calculate conversion state
    if (p_conversion < 1100)
        _conv = 0;
    else if (p_conversion > 1500)
        _conv = 1000;
    else {
        _conv = 1000 - (1500 - p_conversion) / 4 * 10;  //Min 1100 Max 1900 -> (max-x)/800 Default 1900    
        _conv = constrain_int16(_conv, 0, 1000);
    }

    attitude_control.set_conversion(_conv);
    motors.set_conversion(_conv);
}

// update_tiltquad_tilt - sets motor tilt based on conversion
void Copter::update_tiltquad_tilt()
{
    int32_t roll_angle2 = (wrap_180_cd(attitude_control.aeroxo_rate_bf_to_motor_roll(0))) *
        (1000 - _conv) / 4500 * 2;

    int32_t yaw_angle2 = (wrap_180_cd(attitude_control.aeroxo_rate_bf_to_motor_yaw(0))) *
        (_conv) / 4500 * 20;

    int32_t pitch_angle2 = (wrap_180_cd(attitude_control.aeroxo_rate_bf_to_motor_pitch(0))) *
        (1000 - _conv) / 4500 * 2;

    roll_angle2 = constrain_int32(roll_angle2, -250, 250);
    yaw_angle2 = constrain_int32(yaw_angle2, -166, 166);

    int32_t s3 = (1500 - p_conversion) * 10 / 4 + 1000;
    int32_t s2 = (1500 - p_conversion) * 10 / 4 + 1000;
    int32_t s1 = 2000 - (1500 - p_conversion)* 10 /4;
    int32_t s4 = 2000 - (1500 - p_conversion)* 10 /4;

    s1 = constrain_int32(s1, 1000, 2000) + pitch_angle2;
    s2 = constrain_int32(s2, 1000, 2000) + pitch_angle2;
    s3 = constrain_int32(s3, 1000, 2000) - pitch_angle2;
    s4 = constrain_int32(s4, 1000, 2000) - pitch_angle2;

    s1 = constrain_int32(s1, 1000, 2000) - roll_angle2 + yaw_angle2;
    s2 = constrain_int32(s2, 1000, 2000) - roll_angle2 + yaw_angle2;
    s3 = constrain_int32(s3, 1000, 2000) - roll_angle2 + yaw_angle2;
    s4 = constrain_int32(s4, 1000, 2000) - roll_angle2 + yaw_angle2;

    hal.rcout->write(4, s1);
    hal.rcout->write(5, s2);
    hal.rcout->write(6, s3);
    hal.rcout->write(7, s4);
}

#endif // FRAME_CONFIG == TILT_QUAD_FRAME