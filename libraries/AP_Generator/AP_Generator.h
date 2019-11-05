/*
control of hybrid power plant
*/

#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/esc/Status.hpp>
#include <aeroxo/equipment/genset/ecu/Status.hpp>

// generator components indices
#define AEROXO_UAVCAN_COOLER_ID   20
#define AEROXO_UAVCAN_STARTER_ID  21
#define AEROXO_UAVCAN_THROTTLE_ID 22
#define AEROXO_UAVCAN_IGNITION_ID 23

#define AP_GENERATOR_STALE_AFTER_MS 500

#define AP_GENERATOR_MIN_RPM 500

#define AP_GENERATOR_PARAM_PWM_MAX 2000
#define AP_GENERATOR_PARAM_PWM_MIN 1000

#define AP_GENERATOR_MIN_CURRENT 1

class AP_Generator {
public:
    AP_Generator();

    static const struct AP_Param::GroupInfo var_info[];

    void update(void);   

    static AP_Generator *instance() { return _singleton; }

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    enum ICE_State {
        ICE_OFF=0,
        ICE_START_DELAY=2,
        ICE_STARTING=3,
        ICE_RUNNING=4
    };

    enum {
        IGNITION_AUTO,
        IGNITION_ON,
        IGNITION_OFF
    };

    // get current engine control state
    ICE_State get_state(void) const { return state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay);

    void send_telemetry(mavlink_channel_t chan) const;

protected:
    // enable library
    AP_Int8 enable;

    // channel for pilot to command engine start, 0 for none
    AP_Int8 start_chan;

    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;

    // pwm values 
    AP_Int16 pwm_starter_low;
    AP_Int16 pwm_throttle_min;
    AP_Int16 pwm_throttle_max;
    AP_Int16 pwm_cooler_max;

    AP_Float throttle_gain;

    AP_Int16 choke_temp;
    AP_Int16 ice_temp_min;
    AP_Int16 ice_temp_max;

    AP_Int16 charge_target;
    AP_Int16 gen_max;

    AP_Int32 rpm_max;
    AP_Float rpm_gain;

    AP_Float stall_timeout;

    AP_Int8 manual_ignition;

private:
    static AP_Generator *_singleton;

    enum ICE_State state = ICE_OFF;

    // time when we started the starter
    uint32_t starter_start_time_ms;

    // time when we last ran the starter
    uint32_t starter_last_run_ms;

    uint32_t _last_unstalled_ms;

    uint32_t _last_update_ms;

    float _gen_current;
    int32_t _rpm;

    float _ice_temp = NAN, _gen_temp = NAN, _vsi_temp = NAN;

    bool _should_run, _rc_should_run, _armed;
    float _limit;

    struct {
        uint32_t starter_status;
        uint32_t ecu_status;
    } _last_update;

    float _cooler, _starter, _throttle, _rectifier;

    float get_gen_target();
    float get_temp_limit(float temp, float temp_min, float temp_max);

    void update_desired_state();

    static float param_pwm_to_value(uint16_t pwm);

    void update_cooler(float dt);
    void update_rectifier(float dt);
    void update_starter(float dt);
    void update_throttle(float dt);

    void escStatusCallback(const uavcan::equipment::esc::Status &msg);
    void ecuStatusCallback(const aeroxo::equipment::genset::ecu::Status &msg);
};
