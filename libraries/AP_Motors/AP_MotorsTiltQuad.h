// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsTiltQuad.h
/// @brief	Motor control class for TiltQuadcopters

#ifndef __AP_MOTORS_TILT_QUAD_H__
#define __AP_MOTORS_TILT_QUAD_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsTiltQuad
class AP_MotorsTiltQuad : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsTiltQuad(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {};

    // setup_motors - configures the motors for a tiltquad
    virtual void        setup_motors();
    virtual void        set_update_rate( uint16_t speed_hz );

    virtual void        set_yaw(float yaw_in);
    void                set_conversion(int16_t conv) {_conv = conv;}

    // sets motor tilt based on desired r/p/y and current conversion
    void                output_tilt();

    const int servo_offset = 8; // on navio2 servos start from output #9

protected:
    int16_t            _conv; // conversion state
    float              _real_yaw_in;
};
#endif  // AP_MOTORS_TILT_QUAD
