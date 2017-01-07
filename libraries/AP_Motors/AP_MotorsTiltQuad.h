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
        AP_MotorsMatrix(loop_rate, speed_hz),
        _conv(1000),
        _thr_max(1.0f),
        _spin_limit(1.0f)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // setup_motors - configures the motors for a tiltquad
    virtual void        setup_motors();
    void                add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order, float servo_factor);
    virtual void        set_update_rate( uint16_t speed_hz );

    void                set_tilt(float tilt) {_conv = constrain_int16(1000 * (1.0f - tilt), 0, 1000);}

    // sets motor tilt based on desired r/p/y and current conversion
    void                output_tilt();

    void                set_roll_tilt(float roll_tilt) { _roll_tilt = roll_tilt; }
    void                set_pitch_tilt(float pitch_tilt) { _pitch_tilt = pitch_tilt; }
    void                set_yaw_tilt(float yaw_tilt) { _yaw_tilt = yaw_tilt; }

    // set absolute throttle cap 
    void                set_thr_max(float thr_max) { _thr_max = thr_max; }
    // set temporary spin limit
    void set_spin_limit(float spin_limit) { _spin_limit = constrain_float(spin_limit, 0.0f, 1.0f); }

    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    virtual float       get_current_limit_max_throttle();

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters
    AP_Int16           _servo_speed;
    AP_Int8            _servo_offset;
    AP_Int16           _servo_trim[4];
    AP_Float           _servo_scale;
    AP_Int16           _servo_limit;

    int16_t            _conv; // conversion state

    float              _roll_tilt;
    float              _pitch_tilt;
    float              _yaw_tilt;

    float              _servo_factor[4];

    float              _thr_max; // the maximum allowed throttle
    float              _spin_limit; // temporary spin limit, cleared on each loop
};
#endif  // AP_MOTORS_TILT_QUAD
