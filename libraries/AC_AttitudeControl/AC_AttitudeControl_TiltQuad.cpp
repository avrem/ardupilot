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

    float p = control_mix(_pi_stabilize_roll.kP(), _pi_stabilize_roll_tilt.kP()) * angle_error_rads;
    float i = _pi_stabilize_roll.get_i(control_mix(_pid2_rate_roll.kI(), _pid2_rate_roll_tilt.kI()) * angle_error_rads, _dt);
    float d = control_mix(_pid2_rate_roll.kP(), _pid2_rate_roll_tilt.kP()) * rate_error_rads;

    return constrain_float(p + i + d, -1.0f, 1.0f);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;
    float angle_error_rads = _att_error_rot_vec_rad.y;

    float p = control_mix(_pi_stabilize_pitch.kP(), _pi_stabilize_pitch_tilt.kP()) * angle_error_rads;
    float i = _pi_stabilize_pitch.get_i(control_mix(_pid2_rate_pitch.kI(), _pid2_rate_pitch_tilt.kI()) * angle_error_rads, _dt);
    float d = control_mix(_pid2_rate_pitch.kP(), _pid2_rate_pitch_tilt.kP()) * rate_error_rads;

    return constrain_float(p + i + d, -1.0f, 1.0f);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;
  
    float p = control_mix(_pi_stabilize_yaw.kP(), _pi_stabilize_yaw_tilt.kP()) * rate_error_rads;
    float i = _pi_stabilize_yaw.get_i(control_mix(_pid2_rate_yaw.kI(), _pid2_rate_yaw_tilt.kI()) * rate_error_rads, _dt);
	
    return constrain_float(p + i, -1.0f, 1.0f);
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

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),

    _pid2_rate_roll (0.133f, 0.733f, 0, 0, 0, dt),
    _pid2_rate_pitch(0.133f, 0.733f, 0, 0, 0, dt),
    _pid2_rate_yaw  (     0, 0.222f, 0, 0, 0, dt),

    _pid2_rate_roll_tilt (0.089f, 0.511f, 0, 0, 0, dt),
    _pid2_rate_pitch_tilt(0.089f, 0.511f, 0, 0, 0, dt),
    _pid2_rate_yaw_tilt  (     0, 0.444f, 0, 0, 0, dt) 
{
        loadAeroxoTiltrotorParameters(); 
}

void AC_AttitudeControl_TiltQuad::loadAeroxoTiltrotorParameters()
{
    ElytraConfigurator elCfg;
    printf("Scanning XML Elytra config...\n");
    elCfg.scanSetupFile();

    if (elCfg.getOkLoad()) {
        _pi_stabilize_roll=APM_PI2(elCfg.getPRoll(),0,elCfg.getIMaxRoll());
        _pi_stabilize_pitch=APM_PI2(elCfg.getPPitch(),0,elCfg.getIMaxPitch());
        _pi_stabilize_yaw=APM_PI2(elCfg.getPYaw(),0,elCfg.getIMaxYaw());

        _pi_stabilize_roll_tilt=APM_PI2(elCfg.getPRollTilt(),0,elCfg.getIMaxRoll());
        _pi_stabilize_pitch_tilt=APM_PI2(elCfg.getPPitchTilt(),elCfg.getIMaxPitch());
        _pi_stabilize_yaw_tilt=APM_PI2(elCfg.getPYawTilt(),0,elCfg.getIMaxYaw());

        _pid2_rate_roll  = AC_PID( elCfg.getDRoll(),  elCfg.getIRoll(), 0, 0, 0, _dt);
        _pid2_rate_pitch = AC_PID(elCfg.getDPitch(), elCfg.getIPitch(), 0, 0, 0, _dt);
        _pid2_rate_yaw   = AC_PID(                0,   elCfg.getIYaw(), 0, 0, 0, _dt);

        _pid2_rate_roll_tilt  = AC_PID( elCfg.getDRollTilt(),  elCfg.getIRollTilt(), 0, 0, 0, _dt);
        _pid2_rate_pitch_tilt = AC_PID(elCfg.getDPitchTilt(), elCfg.getIPitchTilt(), 0, 0, 0, _dt);
        _pid2_rate_yaw_tilt   = AC_PID(                    0,   elCfg.getIYawTilt(), 0, 0, 0, _dt);

        printf("Elytra: loaded parameters from XML!\n");
    }
    else {
        _pi_stabilize_roll       = APM_PI2(0.378f, 0, 0.133f);
        _pi_stabilize_pitch      = APM_PI2(0.378f, 0, 0.133f);
        _pi_stabilize_yaw        = APM_PI2(0.111f, 0, 0.133f);

        _pi_stabilize_roll_tilt  = APM_PI2(0.267f, 0, 0.133f);
        _pi_stabilize_pitch_tilt = APM_PI2(0.267f, 0, 0.133f);
        _pi_stabilize_yaw_tilt   = APM_PI2(0.222f, 0, 0.133f);

        printf("Elytra: loaded default parameters!\n");
    }
}
