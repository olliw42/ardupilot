/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "Uc4hNotifyDevice.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;


Uc4hNotifyDevice::Uc4hNotifyDevice():
    _healthy(false),
    _task_time_last(0),
    _flags_updated(false),
    _3rgbleds_updated(false),
    _text_updated(false),
    _sync_time_last(0),
    _sync_updated(false)
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        _ap_uavcan[i] = nullptr;
    }

//    memset(&_notify_message_data, 0, sizeof(_notify_message_data));
    strcpy(_text_data, "");
}    


bool Uc4hNotifyDevice::init()
{
    _healthy = false;
    return _healthy;
}


// update - updates led according to timed_updated.  Should be called at 50Hz
void Uc4hNotifyDevice::update()
{
    // return immediately if not yet found
    if (!_healthy) {
        find_CAN();
        return;
    }

    update_slow();
    send_CAN_notify_message();
}


void Uc4hNotifyDevice::find_CAN(void)
{
    //see BP_Mount_STorM32 class as example

    if (_healthy) { //we have found the CAN, so no need to do again, should never happen, but play it safe
        return;
    }

    if (hal.can_mgr == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (hal.can_mgr[i] != nullptr) {
            AP_UAVCAN *ap_uavcan = hal.can_mgr[i]->get_UAVCAN();
            if (ap_uavcan != nullptr) {
                _ap_uavcan[i] = ap_uavcan;
                _healthy = true;

                //here we can register any listeners
            }
        }
    }
}


void Uc4hNotifyDevice::send_CAN_notify_message(void)
{
    //see BP_Mount_STorM32 class as example

    //don't send out more than one message per 50 Hz tick
    // give sync priority

    if (_sync_updated) {
        _sync_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_SYNC, 0, (uint8_t*)(&_sync_data), sizeof(_sync_data) );

            }
        }

    } else
    if (_flags_updated) {
        _flags_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                //subtype is the version of the FLAGS field
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_FLAGS, 0, (uint8_t*)(&_flags_data), sizeof(_flags_data) );

            }
        }

    } else
/*    if (_3rgbleds_updated) {
        _3rgbleds_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                //subtype is the number of leds
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_RGBLEDS, 3, (uint8_t*)(&_rgb3leds_data), sizeof(_rgb3leds_data) );

            }
        }

    } else */
    if (_text_updated) {
        _text_updated = false;
        if (AP_Notify::flags.armed) return; //should never happen, but play it safe

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_TEXT, 0, (uint8_t*)(&_text_data), strlen(_text_data) );

            }
        }
    }
}


// _scheduled_update - updates _red, _green, _blue according to notify flags
void Uc4hNotifyDevice::update_slow(void)
{
    // slow rate from 50 Hz to 4 Hz, make it a bit odd to distribute load
    uint64_t current_time_ms = AP_HAL::millis64();
    if ((current_time_ms - _task_time_last) < 234) {
        return;
    }
    _task_time_last = current_time_ms; //precision is not required

    if ((current_time_ms - _sync_time_last) > 4000) {
        _sync_time_last = current_time_ms; //precision is not required
        update_sync();
    }

    update_flags();
//    update_3rgbleds();
    update_text();
}


void Uc4hNotifyDevice::update_sync(void)
{
    _sync_data.current_time_ms = AP_HAL::millis64();
    _sync_updated = true;
}


void Uc4hNotifyDevice::update_flags(void)
{
    _flags_data.number_of_arms = 4; //ONLY QUAD SUPPORTED currently
    _flags_data.flags = AP_Notify::flags;
    _flags_data.events = AP_Notify::events;
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


//helper
void Uc4hNotifyDevice::set_led_rgb(uint8_t lednr, uint8_t r, uint8_t g, uint8_t b)
{
    //no checks, it's the user's responsibility !
    _rgb3leds_data.rgb[lednr][0] = r;
    _rgb3leds_data.rgb[lednr][1] = g;
    _rgb3leds_data.rgb[lednr][2] = b;
}


void Uc4hNotifyDevice::update_3rgbleds(void)
{
/*// was just for test
    _led_task_count++;

    uint16_t b = _led_task_count % 3; //0,1,2

    uint16_t i = (b) % 3;
    set_led_rgb( i, 0, 0x3F, 0); //green

    i = (b+1) % 3;
    set_led_rgb( i, 0x3F, 0x3F, 0); //yellow

    i = (b+2) % 3;
    set_led_rgb( i, 0x3F, 0, 0); //red
*/

    //pre arm check light: red - yellow - green
    if( AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
        set_led_rgb( 0, 0, 0x3F, 0); //green
    } else
    if( AP_Notify::flags.gps_status == AP_GPS::GPS_OK_FIX_2D) {
        set_led_rgb( 0, 0x1F, 0x3F, 0); //yellow
    } else {
        set_led_rgb( 0, 0x3F, 0, 0); //red
    }

    //mimic position_ok()
    // this is it not yet exactly, but I don't want to pollute the vehicle library, so let's go with this
    if (AP_Notify::flags.ekf_bad) {
        set_led_rgb( 1, 0x3F, 0, 0); //red
    } else
    if (AP_Notify::flags.have_pos_abs) {
        set_led_rgb( 1, 0, 0x3F, 0); //green
    } else {
        set_led_rgb( 1, 0x1F, 0x3F, 0); //yellow
    }

    //pre arm check light: red - green
    if (AP_Notify::flags.pre_arm_check) {
        set_led_rgb( 2, 0, 0x3F, 0); //green
    } else {
        set_led_rgb( 2, 0x3F, 0, 0); //red
    }

    _3rgbleds_updated = true;
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

