// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_TiltQuad.h
/// @brief   ArduCopter attitude control library

#ifndef AC_AttitudeControl_TiltQuad_H
#define AC_AttitudeControl_TiltQuad_H

#include "AC_AttitudeControl_Multi.h"
#include <AC_PID/AC_PID2.h>
#include <AC_PID/APM_PI2.h>

class AC_AttitudeControl_TiltQuad : public AC_AttitudeControl_Multi {
public:
	AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsMulticopter& motors,
                        float dt) :
        AC_AttitudeControl_Multi(ahrs, aparm, motors, dt)
        {
            _pi_stabilize_roll=APM_PI2(16.0f,0,350000); //
            _pi_stabilize_pitch=APM_PI2(16.0f,0,350000); //
            _pi_stabilize_yaw=APM_PI2(0.5f,0,350000); //

            _pi_stabilize_roll_tilt=APM_PI2(6.0f,0,350000);
            _pi_stabilize_pitch_tilt=APM_PI2(12.0f,0,350000);
            _pi_stabilize_yaw_tilt=APM_PI2(0.5f,0,350000);

            _pid2_rate_roll=AC_PID2(6.0f,32.0f,0); //
            _pid2_rate_pitch=AC_PID2(6.0f,32.0f,0); //
            _pid2_rate_yaw=AC_PID2(0,1.0f,0); //

            _pid2_rate_roll_tilt=AC_PID2(2.0f,12.0f,0);
            _pid2_rate_pitch_tilt=AC_PID2(4.0f,24.0f,0);
            _pid2_rate_yaw_tilt=AC_PID2(0,1.0f,0,0);

            loadAeroxoTiltrotorParameters();
        }

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_TiltQuad() {}

    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    // should be called at 100hz or more
    virtual void rate_controller_run();

    void loadAeroxoTiltrotorParameters(); 

    void set_conversion(int16_t conv) {_conv = conv;}

    float aeroxo_rate_bf_to_motor_roll(float rate_target_cds);
    float aeroxo_rate_bf_to_motor_pitch(float rate_target_cds);
    float aeroxo_rate_bf_to_motor_yaw(float rate_target_cds);

    // relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
    virtual void relax_bf_rate_controller();

protected:

    APM_PI2 _pi_stabilize_roll;
    APM_PI2 _pi_stabilize_pitch;
    APM_PI2 _pi_stabilize_yaw;

    APM_PI2 _pi_stabilize_roll_tilt;
    APM_PI2 _pi_stabilize_pitch_tilt;
    APM_PI2 _pi_stabilize_yaw_tilt;

    AC_PID2 _pid2_rate_roll;
    AC_PID2 _pid2_rate_pitch;
    AC_PID2  _pid2_rate_yaw;

    AC_PID2  _pid2_rate_roll_tilt;
    AC_PID2  _pid2_rate_pitch_tilt;
    AC_PID2  _pid2_rate_yaw_tilt;

    int16_t _conv; // conversion state
};

#endif // AC_AttitudeControl_TiltQuad_H
