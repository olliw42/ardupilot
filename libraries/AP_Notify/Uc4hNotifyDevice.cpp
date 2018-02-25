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
    _updated(false)
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        _ap_uavcan[i] = nullptr;
    }

    notify_mode = UC4HNOTIFYMODE_3RGBLEDS; //we currently only support this, sends out 3 rgb values
    memset(&_notify_data, 0, sizeof(_notify_data));
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

    if (!_updated) {
        return;
    }
    _updated = false;

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (_ap_uavcan[i] != nullptr) {

            switch (notify_mode) {
            case UC4HNOTIFYMODE_3RGBLEDS:
                //subtype is the number of leds
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_RGBLEDS, 3, (uint8_t*)(&_notify_data.rgb3leds), 9 );
                break;
            default:
                break;
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
    _task_time_last = current_time_ms;

    switch (notify_mode) {
        case UC4HNOTIFYMODE_3RGBLEDS:
            update_mode_3rgbleds();
            break;
        default:
            break;
    }
}


void Uc4hNotifyDevice::set_led_rgb(uint8_t lednr, uint8_t r, uint8_t g, uint8_t b)
{
    //no checks, it's the user's responsibility !

    _notify_data.rgb3leds.rgb[lednr][0] = r;
    _notify_data.rgb3leds.rgb[lednr][1] = g;
    _notify_data.rgb3leds.rgb[lednr][2] = b;
}


void Uc4hNotifyDevice::update_mode_3rgbleds(void)
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
        set_led_rgb( 0, 0x3F, 0x3F, 0); //yellow
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
        set_led_rgb( 1, 0x3F, 0x3F, 0); //yellow
    }

    //pre arm check light: red - green
    if (AP_Notify::flags.pre_arm_check) {
        set_led_rgb( 2, 0, 0x3F, 0); //green
    } else {
        set_led_rgb( 2, 0x3F, 0, 0); //red
    }

    _updated = true;
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

