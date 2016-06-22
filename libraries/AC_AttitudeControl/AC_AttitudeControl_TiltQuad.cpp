// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_TiltQuad.h"
#include <AP_Math/AP_Math.h>
#include "ElytraConfigurator.h"

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_TiltQuad::rate_controller_run()
{
    _motors.set_roll(aeroxo_rate_bf_to_motor_roll(0));
    _motors.set_pitch(aeroxo_rate_bf_to_motor_pitch(0));
    _motors.set_yaw(aeroxo_rate_bf_to_motor_yaw(0)*20); // NOTE: motors yaw used only in plane mode
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_roll(float rate_target_cds)
{
    float current_rate = _ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100;

    float rate_error = rate_target_cds - current_rate;  
    float angle_error = _att_error_rot_vec_rad.x * AC_ATTITUDE_CONTROL_DEGX100;

    float p = _pi_stabilize_roll.kP() * angle_error * _conv / 1000.0f + _pi_stabilize_roll_tilt.kP() * angle_error * (1000 - _conv) / 1000.0f;
    float i = _pi_stabilize_roll.get_i(_pid2_rate_roll.kI() * angle_error * _conv / 1000.0f + _pid2_rate_roll_tilt.kI() * angle_error * (1000 - _conv) / 1000.0f, _dt);
    float d = _pid2_rate_roll.kP() * rate_error * _conv / 1000.0f + _pid2_rate_roll_tilt.kP() * rate_error * (1000 - _conv) / 1000.0f;

    return constrain_float((p + i + d) / 57.0f / 4500.f, -1.0f, 1.0f);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_cds)
{
    float current_rate = _ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100;

    float rate_error = rate_target_cds - current_rate;
    float angle_error = _att_error_rot_vec_rad.y * AC_ATTITUDE_CONTROL_DEGX100;

    float p = _pi_stabilize_pitch.kP() * angle_error * _conv / 1000.0f + _pi_stabilize_pitch_tilt.kP() * angle_error * (1000 - _conv) / 1000.0f;
    float i = _pi_stabilize_pitch.get_i(_pid2_rate_pitch.kI() * angle_error * _conv / 1000.0f + _pid2_rate_pitch_tilt.kI() * angle_error * (1000 - _conv) / 1000.0f, _dt);
    float d = _pid2_rate_pitch.kP() * rate_error * _conv / 1000.0f + _pid2_rate_pitch_tilt.kP() * rate_error * (1000 - _conv) / 1000.0f;

    return constrain_float((p + i + d) / 57.0f / 4500.f, -1.0f, 1.0f);
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_cds)
{
    float current_rate = _ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100;
    float rate_error = _att_target_euler_rate_rads.z * AC_ATTITUDE_CONTROL_DEGX100 - current_rate;//rate_target_cds - current_rate; NOOOOOOO!
   
    float p = _pi_stabilize_yaw.kP() * rate_error *_conv / 1000.0f + _pi_stabilize_yaw_tilt.kP() * rate_error * (1000 - _conv) / 1000.0f;
    float i = _pi_stabilize_yaw.get_i(_pid2_rate_yaw.kI() * rate_error * _conv / 1000.0f + _pid2_rate_yaw_tilt.kI() * rate_error * (1000 - _conv) / 1000.0f, _dt);
	
    return constrain_float((p + i) / 57.0f / 4500.f, -1.0f, 1.0f);
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl_TiltQuad::relax_bf_rate_controller()
{
    AC_AttitudeControl::relax_bf_rate_controller();

    _pi_stabilize_roll.reset_I();
    _pi_stabilize_pitch.reset_I();
    _pi_stabilize_yaw.reset_I();

    _pi_stabilize_roll_tilt.reset_I();
    _pi_stabilize_pitch_tilt.reset_I();
    _pi_stabilize_yaw_tilt.reset_I();

    _pid2_rate_roll.reset_I();
    _pid2_rate_pitch.reset_I();
    _pid2_rate_yaw.reset_I();

    _pid2_rate_roll_tilt.reset_I();
    _pid2_rate_pitch_tilt.reset_I();
    _pid2_rate_yaw_tilt.reset_I();
}

void AC_AttitudeControl_TiltQuad::loadAeroxoTiltrotorParameters()
{
    ElytraConfigurator elCfg;
    printf("Scanning XML Elytra config...\n");
    elCfg.scanSetupFile();

    if (elCfg.getOkLoad()) {
        _pi_stabilize_roll=APM_PI2(elCfg.getPRoll(),0,elCfg.getIMaxRoll()); //
        _pi_stabilize_pitch=APM_PI2(elCfg.getPPitch(),0,elCfg.getIMaxPitch()); //
        _pi_stabilize_yaw=APM_PI2(elCfg.getPYaw(),0,elCfg.getIMaxYaw()); //

        _pi_stabilize_roll_tilt=APM_PI2(elCfg.getPRollTilt(),0,elCfg.getIMaxRoll());
        _pi_stabilize_pitch_tilt=APM_PI2(elCfg.getPPitchTilt(),elCfg.getIMaxPitch());
        _pi_stabilize_yaw_tilt=APM_PI2(elCfg.getPYawTilt(),0,elCfg.getIMaxYaw());

        _pid2_rate_roll=AC_PID2(elCfg.getDRoll(),elCfg.getIRoll(),0); //
        _pid2_rate_pitch=AC_PID2(elCfg.getDPitch(),elCfg.getIPitch(),0); //
        _pid2_rate_yaw=AC_PID2(0,elCfg.getIYaw(),0); //

        _pid2_rate_roll_tilt=AC_PID2(elCfg.getDRollTilt(),elCfg.getIRollTilt(),0); //
        _pid2_rate_pitch_tilt=AC_PID2(elCfg.getDPitchTilt(),elCfg.getIPitchTilt(),0); //
        _pid2_rate_yaw_tilt=AC_PID2(0,elCfg.getIYawTilt(),0); //

        printf("Elytra: loaded parameters from XML!\n");
    }
    else 
        printf("Elytra: loaded default parameters!\n");
}
