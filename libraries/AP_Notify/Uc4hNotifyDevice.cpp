//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include "Uc4hNotifyDevice.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;


Uc4hNotifyDevice::Uc4hNotifyDevice()
{
    _task_time_last = 0;
    _flags_updated = false;
    _text_updated = false;
    _sync_time_last = 0;
    _sync_updated = false;

    strcpy(_text_data, "");
}    


bool Uc4hNotifyDevice::init()
{
    return true; //the logic has changed, needs to return true to be added as backend
}


// update - updates led according to timed_updated.  Should be called at 50Hz
void Uc4hNotifyDevice::update()
{
    update_slow();
    send_CAN_notify_message();
}


void Uc4hNotifyDevice::send_CAN_notify_message(void)
{
    //don't send out more than one message per 50 Hz tick
    // give sync priority

    if (_sync_updated) {
        _sync_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
            if (ap_uavcan != nullptr) {

                ap_uavcan->uc4hnotify_send( UC4HNOTIFYTYPE_SYNC, 0, (uint8_t*)(&_sync_data), sizeof(_sync_data) );

            }
        }

    } else
    if (_flags_updated) {
        _flags_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
            if (ap_uavcan != nullptr) {

                //subtype is the version of the FLAGS field
                ap_uavcan->uc4hnotify_send( UC4HNOTIFYTYPE_FLAGS, UC4H_APNOTIFYTYPE_V001, (uint8_t*)(&_flags_data), sizeof(_flags_data) );

            }
        }

    } else
    if (_text_updated) {
        _text_updated = false;
        if (AP_Notify::flags.armed) return; //should never happen, but play it safe

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
            if (ap_uavcan != nullptr) {

                ap_uavcan->uc4hnotify_send( UC4HNOTIFYTYPE_TEXT, 0, (uint8_t*)(&_text_data), strlen(_text_data) );

            }
        }
    }
}


// _scheduled_update - updates _red, _green, _blue according to notify flags
void Uc4hNotifyDevice::update_slow(void)
{
    // slow rate from 50 Hz to 4 Hz, make it a bit odd to distribute load
    uint64_t now_ms = AP_HAL::millis64();
    if ((now_ms - _task_time_last) < 234) {
        return;
    }
    _task_time_last = now_ms; //precision is not required

    if ((now_ms - _sync_time_last) > 4000) {
        _sync_time_last = now_ms; //precision is not required
        update_sync();
    }

    update_flags();
    update_text();
}


void Uc4hNotifyDevice::update_sync(void)
{
    _sync_data.current_time_ms = AP_HAL::millis64();
    _sync_updated = true;
}


void Uc4hNotifyDevice::update_flags(void)
{
    _flags_data.flags.initialising          = AP_Notify::flags.initialising;
    _flags_data.flags.firmware_update       = AP_Notify::flags.firmware_update;
    _flags_data.flags.gps_status            = AP_Notify::flags.gps_status;
    _flags_data.flags.gps_num_sats          = AP_Notify::flags.gps_num_sats;
    _flags_data.flags.gps_fusion            = AP_Notify::flags.gps_fusion;
    _flags_data.flags.have_pos_abs          = AP_Notify::flags.have_pos_abs;
    _flags_data.flags.ekf_bad               = AP_Notify::flags.ekf_bad;
    _flags_data.flags.pre_arm_check         = AP_Notify::flags.pre_arm_check;
    _flags_data.flags.pre_arm_gps_check     = AP_Notify::flags.pre_arm_gps_check;
    _flags_data.flags.armed                 = AP_Notify::flags.armed;
    _flags_data.flags.autopilot_mode        = AP_Notify::flags.autopilot_mode;
    _flags_data.flags.flight_mode           = AP_Notify::flags.flight_mode;
    _flags_data.flags.failsafe_radio        = AP_Notify::flags.failsafe_radio;
    _flags_data.flags.failsafe_battery      = AP_Notify::flags.failsafe_battery;
    _flags_data.flags.battery_voltage       = AP::battery().voltage(); // not available anymore AP_Notify::flags.battery_voltage;

    _flags_updated = true;
}


void Uc4hNotifyDevice::update_text(void)
{
    if (AP_Notify::flags.armed) return; //don't send text when armed

    const char* p = pNotify->get_text();
    if (!p) return;
    if (strlen(p) == 0) return;

    if (strcmp(p,_text_data) == 0) return; //check if text has changed

    strcpy(_text_data, p);
    _text_updated = true;
}





/*
the position_ok() sequence is this: (without poptical flow)
    if (failsafe.ekf) return false;
    if (!ahrs.have_inertial_nav()) return false;
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!motors->armed())
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    else
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);

ekf_check.cpp:
failsafe.ekf, ekf_check_state.bad_variance, AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
these seem to all do the same thing (it's a bit weird programming though)

AP_AHRS_NavEKF.cpp
AP_Notify::flags.have_pos_abs = filt_state.flags.horiz_pos_abs;
*/

