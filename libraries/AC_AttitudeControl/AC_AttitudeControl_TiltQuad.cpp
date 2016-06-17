// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_TiltQuad.h"
//#include <AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "ElytraConfigurator.h"

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_TiltQuad::rate_controller_run()
{
    _motors.set_roll(aeroxo_rate_bf_to_motor_roll(0));
    _motors.set_pitch(aeroxo_rate_bf_to_motor_pitch(0));
    _motors.set_yaw(aeroxo_rate_bf_to_motor_yaw(0)*20);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
	float angle_error;       // simply target_rate - current_rate
	float conv = (float)_conv;
    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
   
	//conv=1000.0f;

	//Vector3f targets = attitude_control.angle_ef_targets();

	angle_error = _att_error_rot_vec_rad.x;

	p = (_pi_stabilize_roll.kP() * angle_error*conv/1000.0f)+(_pi_stabilize_roll_tilt.kP() * angle_error*(1000-conv)/1000.0f);

	d = (_pid2_rate_roll.kP() * rate_error*conv/1000.0f)+(_pid2_rate_roll_tilt.kP() * rate_error*(1000-conv)/1000.0f);

	i = _pi_stabilize_roll.get_i((_pid2_rate_roll.kI() * angle_error*conv/1000.0f)+(_pid2_rate_roll_tilt.kI() * angle_error*(1000-conv)/1000.0f),_dt);
    // constrain output and return
    return constrain_float((p+i+d)/57.0f, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX); //PID here
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
	float angle_error;       // simply target_rate - current_rate
	float conv = (float)_conv;
    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
   
	//conv=1000.0f;

	angle_error = _att_error_rot_vec_rad.y;

	p = (_pi_stabilize_pitch.kP() * angle_error*conv/1000.0f)+(_pi_stabilize_pitch_tilt.kP() * angle_error*(1000-conv)/1000.0f);

	d = (_pid2_rate_pitch.kP() * rate_error*conv/1000.0f)+(_pid2_rate_pitch_tilt.kP() * rate_error*(1000-conv)/1000.0f);

	i = _pi_stabilize_pitch.get_i((_pid2_rate_pitch.kI() * angle_error*conv/1000.0f)+(_pid2_rate_pitch_tilt.kI() * angle_error*(1000-conv)/1000.0f),_dt);
	//i = _pi_stabilize_pitch.get_i(57000,0.04f);
    // constrain output and return
    return constrain_float((p+i+d)/57.0f, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX); //PID here
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i;//,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
	//float angle_error;       // simply target_rate - current_rate
	float conv = (float)_conv;
    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = _att_target_euler_rate_rads.z - current_rate;//rate_target_cds - current_rate; NOOOOOOO!
   
	//conv=1000.0f;

	//angle_error = _att_error_rot_vec_rad.z; //For future use

	p = (_pi_stabilize_yaw.kP() * rate_error*conv/1000.0f)+(_pi_stabilize_yaw_tilt.kP() * rate_error*(1000-conv)/1000.0f);

	i = _pi_stabilize_yaw.get_i((_pid2_rate_yaw.kI() * rate_error*conv/1000.0f)+(_pid2_rate_yaw_tilt.kI() * rate_error*(1000-conv)/1000.0f),_dt);
	
	//p=i=0;
    // constrain output and return
    return constrain_float((p+i)/57.0f, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX); //PID here
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
