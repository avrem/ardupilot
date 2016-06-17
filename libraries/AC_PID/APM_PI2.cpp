// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	ACM_PI.cpp
/// @brief	Generic PI algorithm

#include <AP_Math/AP_Math.h>

#include "APM_PI2.h"

const AP_Param::GroupInfo APM_PI2::var_info[] = {
    AP_GROUPINFO("P",    0, APM_PI2, _kp, 0),
    AP_GROUPINFO("I",    1, APM_PI2, _ki, 0),
    AP_GROUPINFO("IMAX", 2, APM_PI2, _imax, 0),
    AP_GROUPEND
};

float APM_PI2::get_p(float error)
{
    return (float)error * _kp;
}

/*int32_t APM_PI2::get_i(int32_t error, float dt)
{
	_imax=350000;
    if(dt != 0) {
        _integrator += ((float)error) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
    return _integrator;
}*/

float APM_PI2::get_i(float error, float dt)
{
	//_imax=350000;
    if(dt != 0) {
        _integrator += (error) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
    return _integrator;
}

int32_t APM_PI2::get_pi(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt);
}

void
APM_PI2::reset_I()
{
    _integrator = 0;
}

void
APM_PI2::load_gains()
{
    _kp.load();
    _ki.load();
    _imax.load();
}

void
APM_PI2::save_gains()
{
    _kp.save();
    _ki.save();
    _imax.save();
}
