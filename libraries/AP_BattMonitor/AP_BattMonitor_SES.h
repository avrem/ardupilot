// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_SES : public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_SES(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state, const AP_SerialManager& serial_manager):
        AP_BattMonitor_Backend(mon, instance, mon_state)
    {
        _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SES, 0);
        if (_port != nullptr)
            _port->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_SES, 0));
    };

    virtual ~AP_BattMonitor_SES(void) {};

    // initialise
    void init();

    // read the latest battery voltage
    void read();

private:
    AP_HAL::UARTDriver *_port;
    uint8_t _rx_buf[16];
    int _rx_pos = 0;

    void process_telemetry();
};
