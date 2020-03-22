#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         500000 // sensor becomes unhealthy if no successful readings for 0.5 seconds

class BattInfoCb;
class BattInfo2Cb;

class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_UAVCAN_Type {
        UAVCAN_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params);

    void init() override {}

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    bool has_current() const override {
        return true;
    }

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_BattMonitor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);
    static void handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb);
    static void handle_battery_info2_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfo2Cb &cb);

private:
    void handle_battery_info(const BattInfoCb &cb);
    void handle_battery_info2(const BattInfo2Cb &cb);

    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_UAVCAN_Type _type;

    HAL_Semaphore _sem_battmon;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    bool _has_cell_voltages;
};
