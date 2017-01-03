/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Tiltquad variables and functions

#if FRAME_CONFIG == TILT_QUAD_FRAME

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
