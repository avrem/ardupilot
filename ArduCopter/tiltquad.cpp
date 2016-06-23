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

#endif // FRAME_CONFIG == TILT_QUAD_FRAME
