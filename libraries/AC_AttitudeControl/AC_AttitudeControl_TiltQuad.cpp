// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_TiltQuad.h"
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_TiltQuad::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_stabilize_roll_tilt, "STB_RL2_", 4, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_stabilize_pitch_tilt, "STB_PI2_", 5, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw_tilt, "RAT_YA2_", 6, AC_AttitudeControl_TiltQuad, AC_PID),

    AP_GROUPEND
};

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_TiltQuad::rate_controller_run()
{
    _motors.set_roll(aeroxo_rate_bf_to_motor_roll(_ang_vel_target_rads.x));
    _motors.set_pitch(aeroxo_rate_bf_to_motor_pitch(_ang_vel_target_rads.y));
    _motors.set_yaw(aeroxo_rate_bf_to_motor_yaw(_ang_vel_target_rads.z));
}

float AC_AttitudeControl_TiltQuad::control_mix(float k_copter, float k_plane)
{
    return k_copter * _conv + k_plane * (1 - _conv);
}

float AC_AttitudeControl_TiltQuad::process_rate_pid(AC_PID &pid, float rate_error_rads, float rate_target_rads, bool saturated)
{
    // Pass error to PID controller
    pid.set_input_filter_d(rate_error_rads);
    pid.set_desired_rate(rate_target_rads);

    float integrator = pid.get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!saturated || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = pid.get_i();
    }

    // Compute output in range -1 ~ +1
    float output = pid.get_p() + integrator + pid.get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_roll(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().x;
    float rate_error_rads = rate_target_rads - current_rate_rads;  
    float angle_error_rads = _att_error_rot_vec_rad.x;

    _pid_stabilize_roll_tilt.set_input_filter_d(angle_error_rads);
    float pi_tilt = _pid_stabilize_roll_tilt.get_pi();
    float d_tilt = _pid_stabilize_roll_tilt.kD() * - current_rate_rads;
    _motors_tq.set_roll_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    float output = process_rate_pid(get_rate_roll_pid(), rate_error_rads, rate_target_rads, _motors.limit.roll_pitch);
    return control_mix(output, 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_pitch(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;
    float angle_error_rads = _att_error_rot_vec_rad.y;

    _pid_stabilize_pitch_tilt.set_input_filter_d(angle_error_rads);
    float pi_tilt = _pid_stabilize_pitch_tilt.get_pi();
    float d_tilt = _pid_stabilize_pitch_tilt.kD() * - current_rate_rads;
    _motors_tq.set_pitch_tilt(control_mix(0, constrain_float(pi_tilt + d_tilt, -1.0f, 1.0f)));

    float output = process_rate_pid(get_rate_pitch_pid(), rate_error_rads, rate_target_rads, _motors.limit.roll_pitch);
    return control_mix(output, 0);
}

float AC_AttitudeControl_TiltQuad::aeroxo_rate_bf_to_motor_yaw(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    _pid_rate_yaw.set_input_filter_d(rate_error_rads);
    float pid = constrain_float(_pid_rate_yaw.get_pid(), -1.0f, 1.0f);
    _motors_tq.set_yaw_tilt(control_mix(pid, 0));

    _pid_rate_yaw_tilt.set_input_filter_d(rate_error_rads);
    float pid_tilt = constrain_float(_pid_rate_yaw_tilt.get_pid(), -1.0f, 1.0f);
    return control_mix(0, pid_tilt);
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl_TiltQuad::relax_bf_rate_controller()
{
    AC_AttitudeControl::relax_bf_rate_controller();

    _pid_stabilize_roll_tilt.reset_I();
    _pid_stabilize_pitch_tilt.reset_I();
    _pid_rate_yaw_tilt.reset_I();
}

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),
    _motors_tq(motors),
    _pid_stabilize_roll_tilt(0.5f, 0.25f, 0.2f, 1.000f, 0, _dt),
    _pid_stabilize_pitch_tilt(1.2f, 0.3f, 0.5f, 1.000f, 0, _dt),
    _pid_rate_yaw_tilt(0.075f, 0.0125f, 0.025f, 0.266f, 5, _dt)
{ 
    _pid_rate_yaw = AC_PID(0.15f, 0.025f, 0, 0.266f, 0, _dt);
}
