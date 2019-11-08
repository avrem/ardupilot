#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Generator.h"

extern const AP_HAL::HAL& hal;

AP_Generator *AP_Generator::_singleton;

const AP_Param::GroupInfo AP_Generator::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable generator control
    // @Description: This enables generator control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Generator, enable,               0, AP_PARAM_FLAG_ENABLE),

    // @Param: START_CHAN
    // @DisplayName: Input channel for generator start
    // @Description: This is an RC input channel for requesting engine start. Engine will try to start when channel is at or above 1700. Engine will stop when channel is at or below 1300. Between 1301 and 1699 the engine will not change state unless a MAVLink command or mission item commands a state change, or the vehicle is disamed.
    // @Values: 0:None,1:Chan1,2:Chan2,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("START_CHAN",   1, AP_Generator, start_chan,           0),

    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @Range: 0.1 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("STARTER_TIME", 2, AP_Generator, starter_time,         4),

    // @Param: START_DELAY
    // @DisplayName: Time to wait between starts
    // @Description: Delay between start attempts
    // @Range: 1 10
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("START_DELAY",  3, AP_Generator, starter_delay,        2),

    // @Param: PWM_STRT_LOW
    // @DisplayName: PWM value at start
    // @Description: This is the value that send to the throttle at start
    // @Values: 1100
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("PWM_STRT_LOW", 4, AP_Generator, pwm_starter_low,   1100),

    // @Param: PWM_THR_MIN
    // @DisplayName: PWM value for minimum throttle angle
    // @Description: This is the value that send to the throttle for reaching minimum angle
    // @Values: 1000 2000
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("PWM_THR_MIN",  5, AP_Generator, pwm_throttle_min,     0),

    // @Param: PWM_THR_MAX
    // @DisplayName: PWM value for maximum throttle angle
    // @Description: This is the value that send to the throttle for reaching maximum angle
    // @Values: 1000 2000
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("PWM_THR_MAX",  6, AP_Generator, pwm_throttle_max,     0),

    // @Param: THR_GAIN
    // @DisplayName: I
    // @Description: This is the gain from integral of angle to throttle
    // @Range: 0 1.0
    // @User: Standard
    AP_GROUPINFO("THR_GAIN",     7, AP_Generator, throttle_gain,    0.01f),

    // @Param: TEMP_CHK
    // @DisplayName: Choke temperature
    // @Description: If ICEngine temperature lower than that value throttle will close
    // @Range: 0 40
    // @Units: Celsius degrees
    // @User: Standard
    AP_GROUPINFO("TEMP_CHK",     8, AP_Generator, choke_temp,          10),

    // @Param: TEMP_MIN
    // @DisplayName: ICEngine minimum temprature
    // @Description: This value set minimum temperature of ICEngine
    // @Range: 0 150
    // @Units: Celsius degrees
    // @User: Standard
    AP_GROUPINFO("TEMP_MIN",     9, AP_Generator, ice_temp_min,        60),

    // @Param: TEMP_MAX
    // @DisplayName: ICEngine maximum temperature
    // @Description: This value set maximum temperature of ICEngine
    // @Range: 0 150
    // @Units: Celsius degrees
    // @User: Standard
    AP_GROUPINFO("TEMP_MAX",    10, AP_Generator, ice_temp_max,       130),

    // @Param: PWM_CLR_MAX
    // @DisplayName: PWM value for maximum cooler rotation
    // @Description: This is the value that send to the cooler for reaching maximum rotation speed
    // @Values: 
    // @User: Standard
    AP_GROUPINFO("PWM_CLR_MAX", 11, AP_Generator, pwm_cooler_max,       0),

    // @Param: CHG_THT
    // @DisplayName: Target value for current
    // @Description: This value sets desired current
    // @Range: 0 10
    // @Units: A
    // @User: Standard
    AP_GROUPINFO("CHG_TGT",     12, AP_Generator, charge_target,        5), // should get from batt_capacity

    // @Param: CURR_MAX
    // @DisplayName: Maximum current
    // @Description: This value sets maximum current from generator
    // @Range: 0 100
    // @Units: A 
    // @User: Standard
    AP_GROUPINFO("CURR_MAX",    13, AP_Generator, gen_max,              0),

    // @Param: RPM_MAX
    // @DisplayName: Maximal RPM of generator
    // @Description: This value sets maximal RPM for generator
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("RPM_MAX",     14, AP_Generator, rpm_max,              0),

    // @Param: RPM_GAIN
    // @DisplayName: I
    // @Description: This is the gain from integral of angle to throttle
    // @Range: 0 0.1
    // @User: Standard
    AP_GROUPINFO("RPM_GAIN",    15, AP_Generator, rpm_gain,        0.005f),

    // @Param: STALL_TIME
    // @DisplayName: Generator timeout
    // @Description: The time after which generator turns off if it doesn't generate any current
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("STALL_TIME",  16, AP_Generator, stall_timeout,        0),

    // @Param: IGN_MANUAL
    // @DisplayName: Manual ignition
    // @Description: 
    // @Values: 1: IGNITION ON, 2: IGNITION OFF
    // @User: Standard
    AP_GROUPINFO("IGN_MANUAL",  17, AP_Generator, manual_ignition,      0),

    AP_GROUPEND
};

AP_Generator::AP_Generator()
{
    if (_singleton == nullptr)
        _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Generator::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr)
        return;

    auto* node = ap_uavcan->get_node();

    auto esc_listener = new uavcan::Subscriber<uavcan::equipment::esc::Status>(*node);
    esc_listener->start(
        [](const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg)
    {if (instance() != nullptr) instance()->escStatusCallback(msg);}
    );

    auto ecu_listener = new uavcan::Subscriber<aeroxo::equipment::genset::ecu::Status>(*node);
    ecu_listener->start(
        [](const uavcan::ReceivedDataStructure<aeroxo::equipment::genset::ecu::Status>& msg)
    {if (instance() != nullptr) instance()->ecuStatusCallback(msg);}
    );

    auto fuel_listener = new uavcan::Subscriber<aeroxo::equipment::genset::ecu::TankStatus>(*node);
    fuel_listener->start(
        [](const uavcan::ReceivedDataStructure<aeroxo::equipment::genset::ecu::TankStatus>& msg)
    {if (instance() != nullptr) instance()->tankStatusCallback(msg);}
    );
}

void AP_Generator::escStatusCallback(const uavcan::equipment::esc::Status& msg)
{
    if (!enable || msg.esc_index != AEROXO_UAVCAN_STARTER_ID)
        return;

    _last_update.starter_status = AP_HAL::millis();
    _rpm = msg.rpm;
    _vsi_temp = msg.temperature - C_TO_KELVIN;
    _gen_current = -msg.current;
}

void AP_Generator::ecuStatusCallback(const aeroxo::equipment::genset::ecu::Status &msg)
{
    if (!enable)
        return;

    _last_update.ecu_status = AP_HAL::millis();
    _ice_temp = msg.engine_temperature - C_TO_KELVIN;
    _gen_temp = msg.generator_temperature - C_TO_KELVIN;
}

void AP_Generator::tankStatusCallback(const aeroxo::equipment::genset::ecu::TankStatus &msg)
{
    if (!enable)
        return;

    _last_update.tank_status = AP_HAL::millis();
    _fuel_lvl_raw = msg.fuel_level;
}

void AP_Generator::update()
{
    if (!enable)
        return;

    uint32_t now = AP_HAL::millis();
    float dt = (now - _last_update_ms) * 0.001f;
    if (dt >= 0.2f)
        dt = 0.0f;

    update_desired_state();

    update_starter(dt);
    update_throttle(dt);
    update_cooler(dt);

    update_fuel();

    _last_update_ms = now;	

    DataFlash_Class::instance()->Log_Write(
        "GEN",
        "TimeUS,Curr,CTot,RPM,EngTemp,GenTemp,VsiTemp,Fuel,Clr,Str,Thr",
        "sAAqOOO%%%%",
        "F0000000000",
        "QffIfffbbbb",
        AP_HAL::micros64(),
        (double)_gen_current,
        (double)(_gen_current + (AP::battery().healthy() ? AP::battery().current_amps() : 0)),
        _rpm,
        (double)_ice_temp,
        (double)_gen_temp,
        (double)_vsi_temp,
        _fuel_lvl_pct,
        (int8_t)(_cooler * 100),
        (int8_t)(_starter * 100),
        (int8_t)(_throttle * 100)
        );
}

float AP_Generator::get_temp_limit(float temp, float temp_min, float temp_max)
{
    if (isnan(temp))
        return 0.0f;
    return constrain_float((temp_max - temp) / (temp_max - temp_min), 0.0f, 1.0f);
}

void AP_Generator::update_desired_state()
{
    if (start_chan != 0) {
        bool rc_should_run = RC_Channels::get_radio_in(start_chan - 1) > 1700;
        if (_rc_should_run != rc_should_run) {
            _rc_should_run = rc_should_run;
            _should_run = _rc_should_run;
        }
    }

    bool armed = hal.util->get_soft_armed();
    if (_armed && !armed) // stop ICE when disarming
        _should_run = false;
    _armed = armed;

    uint32_t now = AP_HAL::millis();

    if (!AP::battery().healthy()) {
        if (_should_run)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Generator: battery monitor unhealthy");
        _should_run = false;
    }

    if (now > _last_update.starter_status + AP_GENERATOR_STALE_AFTER_MS) {
        if (_should_run)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Generator: starter unhealthy");
        _should_run = false;
        _rpm = 0;
        _gen_current = 0;
        _vsi_temp = NAN;
    }

    if (now > _last_update.ecu_status + AP_GENERATOR_STALE_AFTER_MS) {
        _ice_temp = NAN;
        _gen_temp = NAN;
    }

    if (isnan(_ice_temp) || isnan(_gen_temp)) {
        if (_should_run)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Generator: temperature monitor unhealthy");
        _should_run = false;
    }

    float ice_limit = get_temp_limit(_ice_temp, ice_temp_max + 5, ice_temp_max + 20);
    float gen_limit = get_temp_limit(_gen_temp, 100, 115);

    _limit = MIN(ice_limit, gen_limit);

    if (is_zero(_limit)) {
        if (_should_run)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Generator: engine overheating");
        _should_run = false;
    }
    
    if (!_should_run || _gen_current > AP_GENERATOR_MIN_CURRENT)
        _last_unstalled_ms = now;

    if (is_positive(stall_timeout) && now - _last_unstalled_ms > stall_timeout * 1000) {
        if (_should_run)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Generator: engine stalled");
        _should_run = false;
    }
}

float AP_Generator::param_pwm_to_value(uint16_t pwm)
{
    return constrain_float((float)(pwm - AP_GENERATOR_PARAM_PWM_MIN) /
        (AP_GENERATOR_PARAM_PWM_MAX - AP_GENERATOR_PARAM_PWM_MIN), 0.0f, 1.0f);
}

void AP_Generator::update_cooler(float dt)
{
    float cooler = isnan(_ice_temp) ? 1.0f : 
        constrain_float((_ice_temp - ice_temp_min) / (ice_temp_max - ice_temp_min), 0.f, 1.f);
    const float max_step = 1.0f * dt;
    _cooler = constrain_float(cooler, _cooler - max_step, _cooler + max_step);

    AP_UAVCAN::act_write(AEROXO_UAVCAN_COOLER_ID, _cooler * param_pwm_to_value(pwm_cooler_max));
}

void AP_Generator::update_rectifier(float dt)
{
    const float rectifier_max = 0.5f;

    if (rpm_max != 0) {
        float error = _rpm - rpm_max;
        const float max_step = rectifier_max;
        float step = constrain_float(error * rpm_gain, -max_step, max_step);
        _rectifier = constrain_float(_rectifier + step * dt, 0.0f, rectifier_max);
    }
    else
        _rectifier = 0.0f;

    if (state != ICE_RUNNING)
        _rectifier = rectifier_max;
}

void AP_Generator::update_starter(float dt)
{
    uint32_t now = AP_HAL::millis();

    if (!_should_run && state != ICE_OFF) {
        gcs().send_text(MAV_SEVERITY_INFO, "Stopping generator");
        state = ICE_OFF;
    }

    switch (state) {
    case ICE_OFF:
        if (_should_run)
            state = ICE_START_DELAY;
        break;
    case ICE_START_DELAY:
        if (now - starter_last_run_ms > starter_delay * 1000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Starting generator");
            starter_start_time_ms = now;
            state = ICE_STARTING;
        }
        break;
    case ICE_STARTING:
        starter_last_run_ms = now;
        if (now - starter_start_time_ms > starter_time * 1000)
            state = ICE_RUNNING;
        break;
    case ICE_RUNNING:
        if (_rpm < AP_GENERATOR_MIN_RPM)
            state = ICE_OFF;
        break;
    }

    update_rectifier(dt);

    bool ignition_on = state == ICE_RUNNING ||
        (state == ICE_STARTING && (now - starter_start_time_ms > starter_time * 1000 * 0.5f));

    float starter_target;
    if (ignition_on)
        starter_target = 1.0f - _rectifier;
    else if (state == ICE_STARTING)
        starter_target = param_pwm_to_value(pwm_starter_low);
    else
        starter_target = 0.0f;

    _starter = constrain_float(starter_target, _starter - 0.7f * dt, _starter + 1.4f * dt);

    AP_UAVCAN::act_write(AEROXO_UAVCAN_STARTER_ID, _starter);

    float ignition_act;
    switch (manual_ignition) {
    case IGNITION_ON:
        ignition_act = 1.0f;
        break;
    case IGNITION_OFF:
        ignition_act = 0.0f;
        break;
    default:
        ignition_act = ignition_on ? 1.0f : 0.0f;
        break;
    }
    AP_UAVCAN::act_write(AEROXO_UAVCAN_IGNITION_ID, ignition_act);
}

float AP_Generator::get_gen_target()
{
    auto& batt = AP::battery();
    if (!batt.healthy())
        return 0;
    const float v_min = 48, v_max = 50;
    float v = constrain_float(batt.voltage(), v_min, v_max);
    float k = (v - v_min) / (v_max - v_min);
    float c_min = 0, c_max = _gen_current + batt.current_amps() + charge_target;
    float gen_target = k * c_min + (1 - k) * c_max;
    if (gen_max > 0)
        gen_target = MIN(gen_target, gen_max * _limit);
    return gen_target;
}

void AP_Generator::update_throttle(float dt)
{
    float error = get_gen_target() - _gen_current;
    const float max_step = 1.f; // 100% per second
    float step = constrain_float(error * throttle_gain, -max_step, max_step);
    _throttle = constrain_float(_throttle + step * dt, 0.f, 1.f);

    if (_ice_temp < choke_temp) // wait for engine warm up
        _throttle = 0;

    if (state != ICE_RUNNING)
        _throttle = 0;

    uint16_t _pwm_throttle = pwm_throttle_min + _throttle * (pwm_throttle_max - pwm_throttle_min);
    AP_UAVCAN::act_write(AEROXO_UAVCAN_THROTTLE_ID, AP_UAVCAN::calc_servo_output(_pwm_throttle));
}

void AP_Generator::update_fuel()
{
    uint32_t now = AP_HAL::millis();

    if (now > _last_update.tank_status + AP_GENERATOR_STALE_AFTER_MS) {
        _fuel_lvl_pct = 0;
    }
    else {
        float accel_g = constrain_float(-AP::ahrs().get_accel_ef_blended().z / GRAVITY_MSS, 0.3f, 2.0f);
        _fuel_lvl_pct = constrain_int32(_fuel_lvl_raw / accel_g * 100, 0, 100);
    }
}

void AP_Generator::send_telemetry(mavlink_channel_t chan) const
{
    if (enable && HAVE_PAYLOAD_SPACE(chan, GEN_STATUS))
        mavlink_msg_gen_status_send(chan, _gen_current, _rpm, _ice_temp, _gen_temp, _fuel_lvl_pct, _cooler * 100, _starter * 100, _throttle * 100, _vsi_temp);
}

bool AP_Generator::engine_control(float start_control, float cold_start, float height_delay)
{
    _should_run = !_should_run;
    // for now just restart whole stuff

    /*    if (start_control <= 0) {
    state = ICE_OFF;
    return true;
    }
    if (start_chan != 0) {
    // get starter control channel
    if (RC_Channels::get_radio_in(start_chan-1) <= 1300) {
    gcs().send_text(MAV_SEVERITY_INFO, "Engine: start control disabled");
    return false;
    }
    }
    state = ICE_STARTING;*/

    return true;
}
