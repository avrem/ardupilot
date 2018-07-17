/// @file	AP_MotorsTiltQuad.h
/// @brief	Motor control class for TiltQuad
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsTiltQuad
class AP_MotorsTiltQuad : public AP_MotorsMatrix {
public:
    AP_MotorsTiltQuad(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    { AP_Param::setup_object_defaults(this, var_info); }

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}

    void set_tilt(float tilt) { _conv = constrain_int16(1000 * (1.0f - tilt), 0, 1000); }

    void set_roll_tilt(float roll_tilt) { _roll_tilt = roll_tilt; }
    void set_pitch_tilt(float pitch_tilt) { _pitch_tilt = pitch_tilt; }
    void set_yaw_tilt(float yaw_tilt) { _yaw_tilt = yaw_tilt; }

    void set_airspeed(float aspeed);

    void output() override;

    bool limit_tilt = true;

    float get_constrained_speed_scale() { return MAX(_thrust_speed_scale, 0.5f); }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    struct {
        SRV_Channel *chan;
        AP_Int16 trim;
        AP_Int16 coax_trim;
        float factor;
    } _servos[4];

    AP_Int8  _coaxial;
    AP_Int8  _coax_folding;

    AP_Float _servo_scale;
    AP_Int16 _servo_limit;

    AP_Float _pitch_speed;
    AP_Float _elevon_power;

    int16_t  _conv = 1000;

    float    _roll_tilt;
    float    _pitch_tilt;
    float    _yaw_tilt;

    float    _thrust_speed_scale = 1.0f;
    float    _elevon_scale;

    uint32_t _last_unfolded_ms;

    void     add_motor_tq(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order, float servo_factor);
    void     output_tilt();
};
