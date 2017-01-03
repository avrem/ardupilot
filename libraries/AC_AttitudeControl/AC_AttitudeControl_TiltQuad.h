#pragma once

/// @file    AC_AttitudeControl_TiltQuad.h
/// @brief   TiltQuad attitude control library

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_TiltQuad : public AC_AttitudeControl_Multi {
public:
    AC_AttitudeControl_TiltQuad(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt);
    virtual ~AC_AttitudeControl_TiltQuad() {}

    // tilt pid accessors
    AC_PID& get_rate_roll_tilt_pid() { return _pid_rate_roll_tilt; }
    AC_PID& get_rate_pitch_tilt_pid() { return _pid_rate_pitch_tilt; }
    AC_PID& get_rate_yaw_tilt_pid() { return _pid_rate_yaw_tilt; }

    void set_tilt(float tilt) { _tilt = constrain_float(tilt, 0.0f, 1.0f); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // Ensure attitude controller have zero errors to relax rate controller output
    void relax_attitude_controllers() override;

    // reset rate controller I terms
    void reset_rate_controller_I_terms() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    float control_mix(float k_copter, float k_plane);

    AP_MotorsTiltQuad& _motors_tq;

    AC_PID _pid_rate_roll_tilt;
    AC_PID _pid_rate_pitch_tilt;
    AC_PID _pid_rate_yaw_tilt;

    float _tilt;
};
