/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Tiltquad variables and functions

#if FRAME_CONFIG == TILT_QUAD_FRAME

void Copter::update_tiltquad_manual_throttle()
{
    if (mode_has_manual_throttle(control_mode)) {
        int16_t t_off = channel_throttle->get_control_in() - channel_throttle->get_control_mid();
        if (abs(t_off) > g.throttle_deadzone)
            _tilt_manual_throttle = constrain_float(_tilt_manual_throttle + 0.02f * 0.01f * t_off / 500, 0, 1);
    }
    else if (motors.armed()) // reset manual throttle to hover if we're flying in any autopilot mode
        _tilt_manual_throttle = g.throttle_mid * 0.001f;
}

void Copter::update_tiltquad_tilt()
{    
    if (failsafe.rc_override_active) {
        float rc_conv = g.rc_6.norm_input_dz();
        if (rc_conv > 0.1)
            _tilt -= 0.01f;
        else if (rc_conv < -0.1)
            _tilt += 0.01f;
        _tilt = constrain_float(_tilt, 0.0f, 1.0f);
    }
    else
        _tilt = constrain_float(1.0f - g.rc_6.percent_input() * 0.01f, 0.0f, 1.0f);

    attitude_control.set_tilt(_tilt);
    motors.set_tilt(_tilt);
}

#endif // FRAME_CONFIG == TILT_QUAD_FRAME
