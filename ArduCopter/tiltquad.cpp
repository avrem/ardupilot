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

#endif // FRAME_CONFIG == TILT_QUAD_FRAME