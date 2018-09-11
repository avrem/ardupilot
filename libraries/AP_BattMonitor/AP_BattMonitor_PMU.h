#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_ICEngine/AP_ICEngine.h>

#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_PMU : public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_PMU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    virtual ~AP_BattMonitor_PMU(void) {}

    // initialise
    void init() override;

    // read the latest battery voltage
    void read() override;

    bool has_cell_voltages() const override { return true; }

    bool has_current() const override { return true; }

    void status_msg(mavlink_channel_t chan) const override;

    void engine_control(float start_control, float cold_start, float height_delay) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_Int16    _throttle_min;
    AP_Int16    _throttle_max;
    AP_Float    _throttle_gain;
    AP_Int16    _temp_choke;
    AP_Int16    _temp_min;
    AP_Int16    _temp_max;
    AP_Int16    _cooler_max;
    AP_Int16    _charge_target;
    AP_Int16    _charge_max;
    AP_Float    _charge_amp_per_volt;
    AP_Float    _charge_amp_offset;
    AP_Int16    _rpm_max;

private:
    AP_HAL::UARTDriver *_port;
    uint8_t _rx_buf[32];
    int _rx_pos = 0;

    float _charge_current;
    uint16_t _rpm;
    AP_ICEngine::ICE_State _ice_state;

    bool _ice_should_run, _rc_should_run, _armed;

    void send_packet(uint8_t *p, uint8_t *last);
    void process_telemetry();
};
