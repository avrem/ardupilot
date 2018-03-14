#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_BattMonitor_PMU.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_PMU::var_info[] = {
    AP_GROUPINFO("_THR_MIN", 0, AP_BattMonitor_PMU, _throttle_min, 0),
    AP_GROUPINFO("_THR_MAX", 1, AP_BattMonitor_PMU, _throttle_max, 0),

    AP_GROUPINFO("_THR_GAIN", 2, AP_BattMonitor_PMU, _throttle_gain, 0.01f),

    AP_GROUPINFO("_TEMP_CHK", 3, AP_BattMonitor_PMU, _temp_choke, 10),
    AP_GROUPINFO("_TEMP_MIN", 4, AP_BattMonitor_PMU, _temp_min, 60),
    AP_GROUPINFO("_TEMP_MAX", 5, AP_BattMonitor_PMU, _temp_max, 130),

    AP_GROUPINFO("_CLR_MAX", 6, AP_BattMonitor_PMU, _cooler_max, 0),

    AP_GROUPINFO("_CHG_TGT", 7, AP_BattMonitor_PMU, _charge_target, 5),
    AP_GROUPINFO("_CHG_MAX", 8, AP_BattMonitor_PMU, _charge_max, 0),

    // 9 was _SHNT_RAT
    AP_GROUPINFO("_AMPPERVOLT", 10, AP_BattMonitor_PMU, _charge_amp_per_volt, 157),
    AP_GROUPINFO("_AMP_OFFSET", 11, AP_BattMonitor_PMU, _charge_amp_offset, 1.65f),

    AP_GROUPEND
};

#define AP_BATTMONITOR_PMU_TIMEOUT_MICROS 5000000    // sensor becomes unhealthy if no successful readings for 5 seconds
#define AP_PMU_TELEMETRY_LENGTH 23

int read_2(const uint8_t *p)
{
    return p[0] + (p[1] << 7);
}

int read_2s(const uint8_t *p)
{
    return ((read_2(p) << 18) >> 18);
}

void write_2(uint8_t* &p, uint16_t v)
{
    *++p = v & 0x7F;
    *++p = (v >> 7) & 0x7F;
}

void write_5(uint8_t* &p, float f)
{
    union { float f; unsigned v; } u;
    u.f = f;
    for (int i = 0; i < 5; i++)
        *++p = (u.v >> (7 * i)) & 0x7F;
}

void AP_BattMonitor_PMU::send_packet(uint8_t *p, uint8_t *last)
{
    int n = last - p + 1;
    if (n <= 0)
        return;

    uint8_t cs = 0; // set checksum
    for (int i = 0; i < n; i++)
        cs ^= p[i];
    p[n] = cs;

    for (int i = 1; i < n + 1; i++) // set mark bits
        p[i] |= 0x80;

    unsigned sz = n + 1; // forward to serial
    if (_port->txspace() >= sz)
        _port->write(p, sz);
}

AP_BattMonitor_PMU::AP_BattMonitor_PMU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PMU, 0);

    // starts with not healthy
    _state.healthy = false;

    AP_Param::setup_object_defaults(this, var_info);
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

    uint16_t cvalue = RC_Channels::get_radio_in(7-1);
    bool rc_should_run = cvalue >= 1700;
    if (_rc_should_run != rc_should_run) {
        _rc_should_run = rc_should_run;
        _ice_should_run = _rc_should_run;
    }

    if (_port->txspace() >= 3) { // ICE start/stop
        uint8_t ice_start[] = {0x42, 0x80, 0xC2};
        uint8_t ice_stop[] = {0x43, 0x80, 0xC3};

        _port->write(_ice_should_run ? ice_start : ice_stop, 3);
    }

    RC_Channel *ch8 = RC_Channels::rc_channel(8-1);
    if (ch8) { // ICE injection control
        uint8_t ice_injection[4] = {0x47};
        uint8_t *p = ice_injection;
        write_2(p, ch8->percent_input() * 10);
        send_packet(ice_injection, p);
    }

    uint8_t ice_params[128] = {0x48};
    uint8_t *p = ice_params;
    write_2(p, _throttle_min);
    write_2(p, _throttle_max);
    write_2(p, _temp_choke);
    write_2(p, _temp_min);
    write_2(p, _temp_max);
    write_2(p, _cooler_max);
    write_2(p, _charge_target);
    write_2(p, _charge_max);
    write_5(p, _throttle_gain);
    write_5(p, _params._volt_multiplier);
    write_5(p, _params._curr_amp_per_volt);
    write_5(p, _params._curr_amp_offset);
    write_5(p, _charge_amp_per_volt);
    write_5(p, _charge_amp_offset);
    send_packet(ice_params, p);

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
    _rpm = read_2(_rx_buf + 5) * 10;
    _charge_current = read_2s(_rx_buf + 7) * 0.1f;

    _state.temperature = read_2s(_rx_buf + 9) * 0.1f;
    _state.temperature_time = AP_HAL::millis();

    _state.cell_voltages.cells[0] = read_2(_rx_buf + 11); // throttle
    _state.cell_voltages.cells[1] = read_2(_rx_buf + 14); // starter
    _state.cell_voltages.cells[2] = read_2(_rx_buf + 16); // cooler
    _state.cell_voltages.cells[3] = read_2(_rx_buf + 18); // fuel
    _state.cell_voltages.cells[4] = read_2(_rx_buf + 20); // gen temp

    auto state = (AP_ICEngine::ICE_State)_rx_buf[13];
    if (_ice_state != state) {
        if (state == AP_ICEngine::ICE_STARTING)
            gcs().send_text(MAV_SEVERITY_INFO, "Starting engine");
        else if (state == AP_ICEngine::ICE_OFF)
            gcs().send_text(MAV_SEVERITY_INFO, "Stopped engine");

        _ice_state = state;
   }

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
        // .0002778 is 1/3600 (conversion to hours)
        float current_delta = _state.current_amps - _charge_current;
        _state.consumed_mah = MAX(0, _state.consumed_mah + current_delta * dt * 0.0000002778f);
    }

    _state.last_time_micros = tnow;
    _state.healthy = true;

    if (_state.temperature > _temp_max + 10 && _ice_should_run) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Engine overheating");
        _ice_should_run = false;
    }

    struct log_RPM pkt1 = {
        LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
        time_us     : AP_HAL::micros64(),
        rpm1        : (float)_rpm,
        rpm2        : 0
    };
    DataFlash_Class::instance()->WriteBlock(&pkt1, sizeof(pkt1));

    struct log_Current pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT2_MSG),
        time_us             : AP_HAL::micros64(),
        voltage             : 0,
        voltage_resting     : 0,
        current_amps        : _charge_current,
        current_total       : 0,
        consumed_wh         : 0,
        temperature         : 0,
        resistance          : 0
    };
    DataFlash_Class::instance()->WriteBlock(&pkt2, sizeof(pkt2));
}

void AP_BattMonitor_PMU::status_msg(mavlink_channel_t chan) const
{
    if (HAVE_PAYLOAD_SPACE(chan, RPM))
        mavlink_msg_rpm_send(chan, _rpm, 0);
    if (HAVE_PAYLOAD_SPACE(chan, BATTERY2))
        mavlink_msg_battery2_send(chan, _state.voltage, _charge_current * 100);
}

// handle DO_ENGINE_CONTROL messages via MAVLink or mission
void AP_BattMonitor_PMU::engine_control(float start_control, float cold_start, float height_delay)
{
    _ice_should_run = !_ice_should_run;
    // for now just restart whole stuff
}
