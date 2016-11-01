#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_BattMonitor_PMU.h"

extern const AP_HAL::HAL& hal;

#define AP_BATTMONITOR_PMU_TIMEOUT_MICROS 5000000    // sensor becomes unhealthy if no successful readings for 5 seconds
#define AP_PMU_TELEMETRY_LENGTH 21

int read_2(const uint8_t *p)
{
    return p[0] + (p[1] << 7);
}

int read_2s(const uint8_t *p)
{
    return ((read_2(p) << 18) >> 18);
}

AP_BattMonitor_PMU::AP_BattMonitor_PMU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PMU, 0);

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_PMU::init(void)
{
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_PMU::read()
{
    // timeout after 5 seconds
    if ((AP_HAL::micros() - _state.last_time_micros) > AP_BATTMONITOR_PMU_TIMEOUT_MICROS) {
        _state.healthy = false;
    }

    if (_port == nullptr)
        return;

    if (_port->txspace() >= 3) { // request extended telemetry
        uint8_t telem_req[] = {0x44, 0x80, 0xC4};
        _port->write(telem_req, sizeof(telem_req));
    }

    int numc = MIN((int)_port->available(), 64); // try to avoid bogging down in PMU data
    for (int i = 0; i < numc; i++) { // process incoming data
        uint8_t b = _port->read();
        if ((b & 0xC0) == 0x40) { // frame start
            _rx_pos = 0;
            _rx_buf[_rx_pos++] = b;
        }
        else if (_rx_pos > 0 && _rx_buf[0] == 0x44) { // payload
            _rx_buf[_rx_pos++] = b & 0x7F; // strip mark bit
            if (_rx_pos == sizeof(_rx_buf))
                _rx_pos = 0;
            if (_rx_pos == AP_PMU_TELEMETRY_LENGTH) {
                process_telemetry();
                _rx_pos = 0;
            }
        }
    }
}

void AP_BattMonitor_PMU::process_telemetry()
{
    if (_rx_buf[0] != 0x44 || _rx_pos != AP_PMU_TELEMETRY_LENGTH)
        return;

    uint8_t cs = 0;
    for (int i = 0; i < AP_PMU_TELEMETRY_LENGTH; i++)
        cs ^= _rx_buf[i];
    if (cs != 0)
        return;

     uint32_t tnow = AP_HAL::micros();
     float dt = tnow - _state.last_time_micros;

    _state.voltage = read_2(_rx_buf + 1) * 0.01f;
    _state.current_amps = read_2(_rx_buf + 3) * 0.1f;
    _charge_current = read_2s(_rx_buf + 7) * 0.1f;

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
        // .0002778 is 1/3600 (conversion to hours)
        float current_delta = _state.current_amps - _charge_current;
        _state.consumed_mah = MAX(0, _state.consumed_mah + current_delta * dt * 0.0000002778f);
    }

    _state.last_time_micros = tnow;
    _state.healthy = true;
}
