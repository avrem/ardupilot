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

    _pid_stabilize_roll_tilt.set_input_filter_d(angle_error_rads);
    float pi_tilt = _pid_stabilize_roll_tilt.get_pi();
    float d_tilt = _pid_stabilize_roll_tilt.kD() * rate_error_rads;
    _motors_tq.set_roll_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    _pid_stabilize_roll.set_input_filter_d(angle_error_rads);
    float pi = _pid_stabilize_roll.get_pi();
    float d = _pid_stabilize_roll.kD() * rate_error_rads;
    return control_mix(constrain_float(pi + d, -1.0f, 1.0f), 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;
    float angle_error_rads = _att_error_rot_vec_rad.y;

    _pid_stabilize_pitch_tilt.set_input_filter_d(angle_error_rads);
    float pi_tilt = _pid_stabilize_pitch_tilt.get_pi();
    float d_tilt = _pid_stabilize_pitch_tilt.kD() * rate_error_rads;
    _motors_tq.set_pitch_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    _pid_stabilize_pitch.set_input_filter_d(angle_error_rads);
    float pi = _pid_stabilize_pitch.get_pi();
    float d = _pid_stabilize_pitch.kD() * rate_error_rads;
    return control_mix(constrain_float(pi + d, -1.0f, 1.0f), 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    _pi_stabilize_yaw.set_input_filter_d(rate_error_rads);
    float pi = constrain_float(_pi_stabilize_yaw.get_pi(), -1.0f, 1.0f);
    _motors_tq.set_yaw_tilt(control_mix(pi, 0));

    _pi_stabilize_yaw_tilt.set_input_filter_d(rate_error_rads);
    float pi_tilt = constrain_float(_pi_stabilize_yaw_tilt.get_pi(), -1.0f, 1.0f);
    return control_mix(0, pi_tilt);
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl_TiltQuad::relax_bf_rate_controller()
{
    AC_AttitudeControl::relax_bf_rate_controller();

    _pid_stabilize_roll.reset_I();
    _pid_stabilize_pitch.reset_I();
    _pi_stabilize_yaw.reset_I();

    _pid_stabilize_roll_tilt.reset_I();
    _pid_stabilize_pitch_tilt.reset_I();
    _pi_stabilize_yaw_tilt.reset_I();
}

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),
    _motors_tq(motors),
    _pid_stabilize_roll(0.378f, 0.733f, 0.133f, 0.133f, 0, _dt),
    _pid_stabilize_pitch(0.378f, 0.733f, 0.133f, 0.133f, 0, _dt),
    _pi_stabilize_yaw(0.111f, 0.222f, 0, 0.133f, 0, _dt),
    _pid_stabilize_roll_tilt(0.534f, 1.022f, 0.178f, 1.000f, 0, _dt),
    _pid_stabilize_pitch_tilt(0.756f, 1.466f, 0.266f, 1.000f, 0, _dt),
    _pi_stabilize_yaw_tilt(0.055f, 0.111f, 0, 0.133f, 0, _dt)
{
        loadAeroxoTiltrotorParameters(); 
}

void AC_AttitudeControl_TiltQuad::loadAeroxoTiltrotorParameters()
{
    ElytraConfigurator elCfg;
    printf("Scanning XML Elytra config...\n");
    elCfg.scanSetupFile();

    if (elCfg.getOkLoad()) {
        _pid_stabilize_roll  = AC_PID( elCfg.getPRoll(),  elCfg.getIRoll(),  elCfg.getDRoll(),  elCfg.getIMaxRoll(), 0, _dt);
        _pid_stabilize_pitch = AC_PID(elCfg.getPPitch(), elCfg.getIPitch(), elCfg.getDPitch(), elCfg.getIMaxPitch(), 0, _dt);
        _pi_stabilize_yaw    = AC_PID(  elCfg.getPYaw(),   elCfg.getIYaw(),                 0,   elCfg.getIMaxYaw(), 0, _dt);

        _pid_stabilize_roll_tilt  = AC_PID( elCfg.getPRollTilt(),  elCfg.getIRollTilt(),  elCfg.getDRollTilt(),  elCfg.getIMaxRoll(), 0, _dt);
        _pid_stabilize_pitch_tilt = AC_PID(elCfg.getPPitchTilt(), elCfg.getIPitchTilt(), elCfg.getDPitchTilt(), elCfg.getIMaxPitch(), 0, _dt);
        _pi_stabilize_yaw_tilt    = AC_PID(  elCfg.getPYawTilt(),   elCfg.getIYawTilt(),                     0,   elCfg.getIMaxYaw(), 0, _dt);

        printf("Elytra: loaded parameters from XML!\n");
    }
    else {
        printf("Elytra: loaded default parameters!\n");
    }
}
