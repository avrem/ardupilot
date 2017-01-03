#include "AC_AttitudeControl_TiltQuad.h"
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_TiltQuad::var_info[] = {
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_roll_tilt, "RAT_RL2_", 4, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch_tilt, "RAT_PI2_", 5, AC_AttitudeControl_TiltQuad, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw_tilt, "RAT_YA2_", 6, AC_AttitudeControl_TiltQuad, AC_PID),

    AP_GROUPINFO("THR_MIX_MIN",  7, AC_AttitudeControl_TiltQuad, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),
    AP_GROUPINFO("THR_MIX_MAX",  8, AC_AttitudeControl_TiltQuad, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // 9 was PITCH_SPEED

    AP_GROUPINFO("THR_MIX_MAN", 10, AC_AttitudeControl_TiltQuad, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    AP_GROUPEND
};

AC_AttitudeControl_TiltQuad::AC_AttitudeControl_TiltQuad(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsTiltQuad& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt),
    _motors_tq(motors),
    _tilt(0.0f),
    _pid_rate_roll_tilt  (AC_ATC_MULTI_RATE_RP_P,  AC_ATC_MULTI_RATE_RP_I,  AC_ATC_MULTI_RATE_RP_D,  0.0f, AC_ATC_MULTI_RATE_RP_IMAX,  10.0f, 0.0f, 10.0f, dt),
    _pid_rate_pitch_tilt (AC_ATC_MULTI_RATE_RP_P,  AC_ATC_MULTI_RATE_RP_I,  AC_ATC_MULTI_RATE_RP_D,  0.0f, AC_ATC_MULTI_RATE_RP_IMAX,  10.0f, 0.0f, 10.0f, dt),
    _pid_rate_yaw_tilt   (AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, 10.0f, 0.0f, 10.0f, dt)
{ 
    _pid_rate_roll       (AC_ATC_MULTI_RATE_RP_P,  AC_ATC_MULTI_RATE_RP_I,  AC_ATC_MULTI_RATE_RP_D,  0.0f, AC_ATC_MULTI_RATE_RP_IMAX,  10.0f, 0.0f, 10.0f, dt);
    _pid_rate_pitch      (AC_ATC_MULTI_RATE_RP_P,  AC_ATC_MULTI_RATE_RP_I,  AC_ATC_MULTI_RATE_RP_D,  0.0f, AC_ATC_MULTI_RATE_RP_IMAX,  10.0f, 0.0f, 10.0f, dt);
    _pid_rate_yaw        (AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, 10.0f, 0.0f, 10.0f, dt);
    AP_Param::setup_object_defaults(this, var_info);
}

float AC_AttitudeControl_TiltQuad::control_mix(float k_copter, float k_plane)
{
    return k_copter * (1.0f - _tilt) + k_plane * _tilt;
}

void AC_AttitudeControl_TiltQuad::relax_attitude_controllers()
{
    AC_AttitudeControl::relax_attitude_controllers();

    _pid_rate_roll_tilt.reset_filter();
    _pid_rate_pitch_tilt.reset_filter();
    _pid_rate_yaw_tilt.reset_filter();
}

void AC_AttitudeControl_TiltQuad::reset_rate_controller_I_terms()
{
    AC_AttitudeControl::reset_rate_controller_I_terms();

    _pid_rate_roll_tilt.reset_I();
    _pid_rate_pitch_tilt.reset_I();
    _pid_rate_yaw_tilt.reset_I();
}

void AC_AttitudeControl_TiltQuad::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    float roll_tilt_in = _pid_rate_roll_tilt.update_all(_rate_target_ang_vel.x, gyro_latest.x, _motors_tq.limit_tilt);
    _motors_tq.set_roll_tilt(control_mix(0, roll_tilt_in));
    float roll_in = _pid_rate_roll.update_all(_rate_target_ang_vel.x, gyro_latest.x, _motors_tq.limit.roll);
    _motors_tq.set_roll(control_mix(roll_in, 0));

    float pitch_tilt_in = _pid_rate_pitch_tilt.update_all(_rate_target_ang_vel.y, gyro_latest.y, _motors_tq.limit_tilt);
    _motors_tq.set_pitch_tilt(control_mix(0, pitch_tilt_in));
    float pitch_in = _pid_rate_pitch.update_all(_rate_target_ang_vel.y, gyro_latest.y, _motors_tq.limit.pitch);
    _motors_tq.set_pitch(control_mix(pitch_in, 0));

    float yaw_tilt_in = _pid_rate_yaw.update_all(_rate_target_ang_vel.z, gyro_latest.z, _motors_tq.limit_tilt);
    _motors_tq.set_yaw_tilt(control_mix(yaw_tilt_in, 0));
    float yaw_in = _pid_rate_yaw_tilt.update_all(_rate_target_ang_vel.z, gyro_latest.z, _motors.limit.yaw);
    _motors_tq.set_yaw(control_mix(0, yaw_in) / _motors_tq.get_constrained_speed_scale());

    control_monitor_update();
}
