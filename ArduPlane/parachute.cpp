#include "Plane.h"


/* 
   call parachute library update, check if we need deploy parachute automatically
*/
void Plane::parachute_check()
{
#if PARACHUTE == ENABLED
    static uint16_t control_loss_count;	// number of iterations we have been out of control

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if parachute watchdog is not configured
    if (parachute.watchdog_ms() <= 0 || parachute.max_sink_rate() <= 0) {
        return;
    }

    // return immediately if motors are not armed
    if (!hal.util->get_soft_armed()) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying
    if (!is_flying()) {
        control_loss_count = 0;
        return;
    }

    // ensure the first control_loss event is from above the min altitude
    if (control_loss_count == 0 && parachute.alt_min() > 0 && relative_ground_altitude(false) < parachute.alt_min()) {
        return;
    }

    // check for critical sink rate
    if (auto_state.sink_rate <= parachute.max_sink_rate()) {
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }

    const int dt = 1000 / 10;
    const int parachute_check_trigger = (parachute.watchdog_ms() + dt - 1) / dt;

    // increment counter
    if (control_loss_count < parachute_check_trigger) {
        control_loss_count++;
    }

    // check if we are out of control for longer than watchdog delay
    if (control_loss_count >= parachute_check_trigger) {
        // reset control loss counter
        control_loss_count = 0;
        // release parachute
        parachute_release();
    }
#endif
}

#if PARACHUTE == ENABLED

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.release_in_progress()) {
        return;
    }
    // send message to gcs and dataflash
    if (parachute.released()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released again");
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    }

    // disarm motors
    disarm_motors();

    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    if (parachute.alt_min() > 0 && relative_ground_altitude(false) < parachute.alt_min() &&
            auto_state.last_flying_ms > 0) {
        // Allow manual ground tests by only checking if flying too low if we've taken off
        gcs().send_text(MAV_SEVERITY_WARNING, "Parachute: Too low");
        return false;
    }

    // if we get this far release parachute
    parachute_release();

    return true;
}

#endif
