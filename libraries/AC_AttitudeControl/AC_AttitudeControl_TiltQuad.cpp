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
    _motors.set_yaw(aeroxo_rate_bf_to_motor_yaw(_ang_vel_target_rads.z));
}

float AC_AttitudeControl_TiltQuad::control_mix(float k_copter, float k_plane)
{
    return k_copter * _conv + k_plane * (1 - _conv);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_roll(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().x;
    float rate_error_rads = rate_target_rads - current_rate_rads;  
    float angle_error_rads = _att_error_rot_vec_rad.x;

    float pi_tilt = _pi_stabilize_roll_tilt.get_pi(angle_error_rads, _dt);
    float d_tilt = _p_rate_roll_tilt.get_p(rate_error_rads);
    _motors_tq.set_roll_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    float pi = _pi_stabilize_roll.get_pi(angle_error_rads, _dt);
    float d = _p_rate_roll.get_p(rate_error_rads);
    return control_mix(constrain_float(pi + d, -1.0f, 1.0f), 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;
    float angle_error_rads = _att_error_rot_vec_rad.y;

    float pi_tilt = _pi_stabilize_pitch_tilt.get_pi(angle_error_rads, _dt);
    float d_tilt = _p_rate_pitch_tilt.get_p(rate_error_rads);
    _motors_tq.set_pitch_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    float pi = _pi_stabilize_pitch.get_pi(angle_error_rads, _dt);
    float d = _p_rate_pitch.get_p(rate_error_rads);
    return control_mix(constrain_float(pi + d, -1.0f, 1.0f), 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    float pi = constrain_float(_pi_stabilize_yaw.get_pi(rate_error_rads, _dt), -1.0f, 1.0f);
    _motors_tq.set_yaw_tilt(control_mix(pi, 0));

    float pi_tilt = constrain_float(_pi_stabilize_yaw_tilt.get_pi(rate_error_rads, _dt), -1.0f, 1.0f);
    return control_mix(0, pi_tilt);
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
}

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),
    _motors_tq(motors),
    _p_rate_roll(0.133f),
    _p_rate_pitch(0.133f),
    _p_rate_roll_tilt(0.178f),
    _p_rate_pitch_tilt(0.266f)
{
        loadAeroxoTiltrotorParameters(); 
}

void AC_AttitudeControl_TiltQuad::loadAeroxoTiltrotorParameters()
{
    ElytraConfigurator elCfg;
    printf("Scanning XML Elytra config...\n");
    elCfg.scanSetupFile();

    if (elCfg.getOkLoad()) {
        _pi_stabilize_roll  = APM_PI2( elCfg.getPRoll(), elCfg.getIRoll(), elCfg.getIMaxRoll());
        _pi_stabilize_pitch = APM_PI2(elCfg.getPPitch(),elCfg.getIPitch(),elCfg.getIMaxPitch());
        _pi_stabilize_yaw   = APM_PI2(  elCfg.getPYaw(),  elCfg.getIYaw(),  elCfg.getIMaxYaw());

        _pi_stabilize_roll_tilt  = APM_PI2( elCfg.getPRollTilt(), elCfg.getIRollTilt(),  elCfg.getIMaxRoll());
        _pi_stabilize_pitch_tilt = APM_PI2(elCfg.getPPitchTilt(),elCfg.getIPitchTilt(), elCfg.getIMaxPitch());
        _pi_stabilize_yaw_tilt   = APM_PI2(  elCfg.getPYawTilt(),  elCfg.getIYawTilt(),   elCfg.getIMaxYaw());

        _p_rate_roll  = AC_P(elCfg.getDRoll());
        _p_rate_pitch = AC_P(elCfg.getDPitch());

        _p_rate_roll_tilt  = AC_P(elCfg.getDRollTilt());
        _p_rate_pitch_tilt = AC_P(elCfg.getDPitchTilt());

        printf("Elytra: loaded parameters from XML!\n");
    }
    else {
        _pi_stabilize_roll       = APM_PI2(0.378f, 0.733f, 0.133f);
        _pi_stabilize_pitch      = APM_PI2(0.378f, 0.733f, 0.133f);
        _pi_stabilize_yaw        = APM_PI2(0.111f, 0.222f, 0.133f);

        _pi_stabilize_roll_tilt  = APM_PI2(0.534f, 1.022f, 1.000f);
        _pi_stabilize_pitch_tilt = APM_PI2(0.756f, 1.466f, 1.000f);
        _pi_stabilize_yaw_tilt   = APM_PI2(0.055f, 0.111f, 0.133f);

        printf("Elytra: loaded default parameters!\n");
    }
}
