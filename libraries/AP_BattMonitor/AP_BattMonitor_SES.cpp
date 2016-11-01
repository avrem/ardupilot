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

#include <AP_HAL/AP_HAL.h>

#include "AP_BattMonitor_SES.h"

extern const AP_HAL::HAL& hal;

#define AP_BATTMONITOR_SES_TIMEOUT_MICROS 5000000    // sensor becomes unhealthy if no successful readings for 5 seconds
#define AP_SES_TELEMETRY_LENGTH 11

void AP_BattMonitor_SES::init(void)
{
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_SES::read()
{
    // timeout after 5 seconds
    if ((AP_HAL::micros() - _state.last_time_micros) > AP_BATTMONITOR_SES_TIMEOUT_MICROS) {
        _state.healthy = false;
    }

    if (_port == nullptr)
        return;

    if (_port->txspace() >= 3) { // request data from SES board
        uint8_t telem_req[] = {0x41, 0x80, 0xC1};
        _port->write(telem_req, sizeof(telem_req));
    }

    int numc = MIN(_port->available(), 64); // try to avoid bogging down in SES data
    for (int i = 0; i < numc; i++) { // process incoming data
        uint8_t b = _port->read();
        if ((b & 0xC0) == 0x40) { // frame start
            _rx_pos = 0;
            _rx_buf[_rx_pos++] = b;
        }
        else if (_rx_pos > 0 && _rx_buf[0] == 0x41) { // payload
            _rx_buf[_rx_pos++] = b & 0x7F; // strip mark bit
            if (_rx_pos == AP_SES_TELEMETRY_LENGTH) {
                process_telemetry();
                _rx_pos = 0;
            }
        }
    }

}

uint16_t read_14(const uint8_t *p)
{
    return p[0] + (p[1] << 7);
}

void AP_BattMonitor_SES::process_telemetry()
{
    if (_rx_buf[0] != 0x41 || _rx_pos != AP_SES_TELEMETRY_LENGTH)
        return;

    uint8_t cs = 0;
    for (int i = 0; i < AP_SES_TELEMETRY_LENGTH; i++)
        cs ^= _rx_buf[i];
    if (cs != 0)
        return;

    _state.voltage = read_14(_rx_buf + 1) * 0.01f;
    _state.current_amps = read_14(_rx_buf + 3) * 0.1f;
    _state.last_time_micros = AP_HAL::micros();
    _state.healthy = true;
}
