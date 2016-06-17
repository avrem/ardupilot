// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsTiltQuad.h
/// @brief	Motor control class for TiltQuadcopters

#ifndef __AP_MOTORS_TILT_QUAD_H__
#define __AP_MOTORS_TILT_QUAD_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
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

    void                add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    void                set_conversion(int16_t conv) {_conv = conv;}

protected:

    int16_t            _conv; // conversion state
};
#endif  // AP_MOTORS_TILT_QUAD
