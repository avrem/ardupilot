#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
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

    bool has_current() const override { return true; }

private:
    AP_HAL::UARTDriver *_port;
    uint8_t _rx_buf[32];
    int _rx_pos = 0;

    float _charge_current;

    void process_telemetry();
};
