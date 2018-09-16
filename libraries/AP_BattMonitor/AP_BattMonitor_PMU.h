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

private:
    AP_HAL::UARTDriver *_port;
    uint8_t _rx_buf[32];
    int _rx_pos = 0;

    float _charge_current;
    uint16_t _rpm;
    AP_ICEngine::ICE_State _ice_state;

    void send_packet(uint8_t *p, uint8_t *last);
    void process_telemetry();
};
