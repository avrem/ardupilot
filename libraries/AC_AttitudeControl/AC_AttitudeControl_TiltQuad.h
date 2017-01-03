#pragma once

/// @file    AC_AttitudeControl_TiltQuad.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_TiltQuad : public AC_AttitudeControl_Multi {
public:
	AC_AttitudeControl_TiltQuad(AP_AHRS_View &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsTiltQuad& motors,
                        float dt);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_TiltQuad() {}

    // tilt pid accessors
    AC_PID& get_rate_roll_tilt_pid() { return _pid_rate_roll_tilt; }
    AC_PID& get_rate_pitch_tilt_pid() { return _pid_rate_pitch_tilt; }
    AC_PID& get_rate_yaw_tilt_pid() { return _pid_rate_yaw_tilt; }

    void set_tilt(float tilt) {_tilt = constrain_float(tilt, 0.0f, 1.0f);}

    virtual float rate_target_to_motor_roll(float rate_actual_rads, float rate_target_rads) override;
    virtual float rate_target_to_motor_pitch(float rate_actual_rads, float rate_target_rads) override;
    virtual float rate_target_to_motor_yaw(float rate_actual_rads, float rate_target_rads) override;

    static float process_rate_pid(AC_PID &pid, float rate_error_rads, float rate_target_rads, bool saturated);

    // Ensure attitude controller have zero errors to relax rate controller output
    virtual void relax_attitude_controllers() override;

    float control_mix(float k_copter, float k_plane);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    AC_PID _pid_rate_roll_tilt;
    AC_PID _pid_rate_pitch_tilt;
    AC_PID _pid_rate_yaw_tilt;

    AP_MotorsTiltQuad& _motors_tq;

    float _tilt;

};
