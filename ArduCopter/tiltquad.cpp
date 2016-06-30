/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Tiltquad variables and functions

#if FRAME_CONFIG == TILT_QUAD_FRAME

void Copter::update_tiltquad_manual_throttle()
{
    if (mode_has_manual_throttle(control_mode)) {
        int16_t t_off = channel_throttle->get_control_in() - channel_throttle->get_control_mid();
        if (abs(t_off) > g.throttle_deadzone)
            _tilt_manual_throttle = constrain_float(_tilt_manual_throttle + 0.2f * 0.01f * t_off / 500, 0, 1);
    }
    else if (motors.armed()) // reset manual throttle to hover if we're flying in any autopilot mode
        _tilt_manual_throttle = 0.5f;
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
