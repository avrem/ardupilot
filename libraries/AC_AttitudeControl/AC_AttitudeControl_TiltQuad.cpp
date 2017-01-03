#include "AC_AttitudeControl_TiltQuad.h"
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_TiltQuad::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_roll_tilt, "RAT_RL2_", 4, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch_tilt, "RAT_PI2_", 5, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw_tilt, "RAT_YA2_", 6, AC_AttitudeControl_TiltQuad, AC_PID),

    AP_GROUPINFO("THR_MIX_MIN", 7, AC_AttitudeControl_TiltQuad, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),
    AP_GROUPINFO("THR_MIX_MAX", 8, AC_AttitudeControl_TiltQuad, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // 9 was PITCH_SPEED

    AP_GROUPEND
};

float AC_AttitudeControl_TiltQuad::control_mix(float k_copter, float k_plane)
{
    return k_copter * (1.0f - _tilt) + k_plane * _tilt;
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
    float output = pid.get_p() + integrator + pid.get_d() + pid.get_ff(rate_target_rads);

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl_TiltQuad::rate_target_to_motor_roll(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    float output_tilt = process_rate_pid(_pid_rate_roll_tilt, rate_error_rads, rate_target_rads, _motors_tq.limit_tilt);
    _motors_tq.set_roll_tilt(control_mix(0, output_tilt));

    float output = process_rate_pid(get_rate_roll_pid(), rate_error_rads, rate_target_rads, _motors.limit.roll_pitch);
    return control_mix(output, 0);
}

float AC_AttitudeControl_TiltQuad::rate_target_to_motor_pitch(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    float output_tilt = process_rate_pid(_pid_rate_pitch_tilt, rate_error_rads, rate_target_rads, _motors_tq.limit_tilt);
    _motors_tq.set_pitch_tilt(control_mix(0, output_tilt));

    float output = process_rate_pid(get_rate_pitch_pid(), rate_error_rads, rate_target_rads, _motors.limit.roll_pitch);
    return control_mix(output, 0);
}

float AC_AttitudeControl_TiltQuad::rate_target_to_motor_yaw(float rate_actual_rads, float rate_target_rads)
{
    float rate_error_rads = rate_target_rads - rate_actual_rads;

    float output = process_rate_pid(_pid_rate_yaw, rate_error_rads, rate_target_rads, _motors_tq.limit_tilt);
    _motors_tq.set_yaw_tilt(control_mix(output, 0));

    float output_tilt = process_rate_pid(_pid_rate_yaw_tilt, rate_error_rads, rate_target_rads, _motors.limit.yaw);
    return control_mix(0, output_tilt);
}

// Ensure attitude controller have zero errors to relax rate controller output
void AC_AttitudeControl_TiltQuad::relax_attitude_controllers()
{
    AC_AttitudeControl::relax_attitude_controllers();

    _pid_rate_roll_tilt.reset_I();
    _pid_rate_pitch_tilt.reset_I();
    _pid_rate_yaw_tilt.reset_I();
}

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),
    _motors_tq(motors),
    _tilt(0.0f),
    _pid_rate_roll_tilt(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_YAW_FILT_HZ, _dt),
    _pid_rate_pitch_tilt(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_YAW_FILT_HZ, _dt),
    _pid_rate_yaw_tilt(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, _dt)
{ 
}
