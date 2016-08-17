// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_TiltQuad.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_TiltQuad_H
#define AC_AttitudeControl_TiltQuad_H

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_TiltQuad : public AC_AttitudeControl_Multi {
public:
	AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsTiltQuad& motors,
                        float dt);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_TiltQuad() {}

    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    // should be called at 100hz or more
    virtual void rate_controller_run();

    void set_conversion(int16_t conv) {_conv = constrain_float(conv * 0.001f, 0.f, 1.f);}

    float aeroxo_rate_bf_to_motor_roll(float rate_target_rads);
    float aeroxo_rate_bf_to_motor_pitch(float rate_target_rads);
    float aeroxo_rate_bf_to_motor_yaw(float rate_target_rads);

    static float process_rate_pid(AC_PID &pid, float rate_error_rads, float rate_target_rads, bool saturated);

    // relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
    virtual void relax_bf_rate_controller();

    float control_mix(float k_copter, float k_plane);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    AC_PID _pid_stabilize_roll_tilt;
    AC_PID _pid_stabilize_pitch_tilt;
    AC_PID _pid_rate_yaw_tilt;

    AP_MotorsTiltQuad& _motors_tq;

    float _conv; // conversion state, 0..1

};

#endif // AC_AttitudeControl_TiltQuad_H
