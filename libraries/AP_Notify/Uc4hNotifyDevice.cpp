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
    _3rgbleds_updated(false),
    _oreoleds_updated(false),
    _sync_time_last(0),
    _sync_updated(false)
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        _ap_uavcan[i] = nullptr;
    }

    memset(&_notify_message_data, 0, sizeof(_notify_message_data));

    //clear the oreoleds
    set_oreoled(0, UC4HOREOLOEDTYPE_CLEAR, 0, 0,0,0);
    _oreoleds_updated = true;
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

                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_SYNC, 0, (uint8_t*)(&_notify_message_data.sync), SYNC_SIZE );

            }
        }

    } else
    if (_flags_updated) {
        _flags_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                //subtype is the version of the FLAGS field
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_FLAGS, 0, (uint8_t*)(&_notify_message_data.flags), FLAGS_SIZE );

            }
        }

    } else
    if (_3rgbleds_updated) {
        _3rgbleds_updated = false;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                //subtype is the number of leds
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_RGBLEDS, 3, (uint8_t*)(&_notify_message_data.rgb3leds), RGB3LEDS_SIZE );

            }
        }

    } else
    if (_oreoleds_updated) {
        _oreoleds_updated = false;
/*
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (_ap_uavcan[i] != nullptr) {

                //subtype is the number of arms //XX only QUAD supported currently
                _ap_uavcan[i]->uc4hnotify_send( UC4HNOTIFYTYPE_OREOLEDS, 4, (uint8_t*)(&_notify_message_data.oreoleds), OREO_SIZE );

            }
        }
*/
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

    if ((current_time_ms - _sync_time_last) > 4000) {
        _sync_time_last = current_time_ms;
        update_sync();
    }

    update_flags();
    update_3rgbleds();
    update_oreoleds();
}


void Uc4hNotifyDevice::update_sync(void)
{
    _notify_message_data.sync.current_time_ms = AP_HAL::millis64();
    _sync_updated = true;
}


void Uc4hNotifyDevice::update_flags(void)
{
    _notify_message_data.flags.flags = AP_Notify::flags;
    _notify_message_data.flags.events = AP_Notify::events;
    _flags_updated = true;
}


//helper
void Uc4hNotifyDevice::set_led_rgb(uint8_t lednr, uint8_t r, uint8_t g, uint8_t b)
{
    //no checks, it's the user's responsibility !
    _notify_message_data.rgb3leds.rgb[lednr][0] = r;
    _notify_message_data.rgb3leds.rgb[lednr][1] = g;
    _notify_message_data.rgb3leds.rgb[lednr][2] = b;
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
    #define SET_RED     set_led_rgb( 0, 0x3F, 0, 0); //red
    #define SET_GREEN   set_led_rgb( 0, 0, 0x3F, 0); //green
    #define SET_YELLOW  set_led_rgb( 0, 0x1F, 0x3F, 0) //yellow

    //pre arm check light: red - yellow - green
    if( AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
        SET_GREEN; //set_led_rgb( 0, 0, 0x3F, 0); //green
    } else
    if( AP_Notify::flags.gps_status == AP_GPS::GPS_OK_FIX_2D) {
        SET_YELLOW; //set_led_rgb( 0, 0x1F, 0x3F, 0); //yellow
    } else {
        SET_RED; //set_led_rgb( 0, 0x3F, 0, 0); //red
    }

    //mimic position_ok()
    // this is it not yet exactly, but I don't want to pollute the vehicle library, so let's go with this
    if (AP_Notify::flags.ekf_bad) {
        SET_RED; //set_led_rgb( 1, 0x3F, 0, 0); //red
    } else
    if (AP_Notify::flags.have_pos_abs) {
        SET_GREEN; //set_led_rgb( 1, 0, 0x3F, 0); //green
    } else {
        SET_YELLOW; //set_led_rgb( 0, 0x1F, 0x3F, 0); //yellow
    }

    //pre arm check light: red - green
    if (AP_Notify::flags.pre_arm_check) {
        SET_GREEN; //set_led_rgb( 2, 0, 0x3F, 0); //green
    } else {
        SET_RED; //set_led_rgb( 2, 0x3F, 0, 0); //red
    }

    _3rgbleds_updated = true;
}


//helper
void Uc4hNotifyDevice::set_oreoled(uint8_t lednr, uint8_t type, uint8_t period, uint8_t r, uint8_t g, uint8_t b)
{
    set_oreoled(lednr, type, period, 0, r, g, b);
}


void Uc4hNotifyDevice::set_oreoled(uint8_t lednr, uint8_t type, uint8_t period, uint8_t phase, uint8_t r, uint8_t g, uint8_t b)
{
    //no checks, it's the user's responsibility !
    _notify_message_data.oreoleds.oreo[lednr][0] = type;
    _notify_message_data.oreoleds.oreo[lednr][1] = period;
    _notify_message_data.oreoleds.oreo[lednr][2] = phase;
    _notify_message_data.oreoleds.oreo[lednr][3] = r;
    _notify_message_data.oreoleds.oreo[lednr][4] = g;
    _notify_message_data.oreoleds.oreo[lednr][5] = b;
}


void Uc4hNotifyDevice::update_oreoleds(void)
{
// was just for test
/*
_led_task_count++;

uint16_t b = _led_task_count % 4; //0,1,2,3

uint16_t i = (b) % 4;
set_oreoled(i,  UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, 255, 0, 0);
i = (b+1) % 4;
set_oreoled(i, UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, 0, 255, 0);
i = (b+2) % 4;
set_oreoled(i,   UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, 0, 0, 255);
i = (b+3) % 4;
set_oreoled(i,  UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, 128, 128, 128);
*/

//mimic behavior of OreoLED_PX4
//    if (mode_firmware_update()) { return; } // don't go any further if the Pixhawk is in firmware update
//    if (mode_init()) { return; }            // don't go any further if the Pixhawk is initializing
//    if (mode_failsafe_radio()) { return; }  // don't go any further if the Pixhawk is is in radio failsafe
//    set_standard_colors();                  // set the rear LED standard colors as described above
//    if (mode_failsafe_batt()) { return; }   // stop here if the battery is low.
//    if (_pattern_override) { return; }      // stop here if in mavlink LED control override.
//    if (mode_auto_flight()) { return; }     // stop here if in an autopilot mode.
//    mode_pilot_flight();                    // stop here if in an pilot controlled mode.

    #define RED     255, 0, 0
    #define BLUE    0, 0, 255
    #define GREEN   0, 255, 0

    // Procedure for when Pixhawk is in FW update / bootloader
    // Makes all LEDs go into color cycle mode
    if (AP_Notify::flags.firmware_update) { //XX ignore that case
        //set_macro(OREOLED_INSTANCE_ALL, OREOLED_PARAM_MACRO_COLOUR_CYCLE);
        //return;
    }

    // Makes all LEDs rapidly strobe blue while gyros initialize.
    if (AP_Notify::flags.initialising) {
        set_oreoled(UC4HQUAD_FRONTLEFT,  UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, BLUE); //0, 0, 255);
        set_oreoled(UC4HQUAD_FRONTRIGHT, UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, BLUE); //0, 0, 255);
        set_oreoled(UC4HQUAD_BACKLEFT,   UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, BLUE); //0, 0, 255);
        set_oreoled(UC4HQUAD_BACKRIGHT,  UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, BLUE); //0, 0, 255);
        return;
    }

    // Procedure for when Pixhawk is in radio failsafe
    // LEDs perform alternating Red X pattern
    if (AP_Notify::flags.failsafe_radio) {
        set_oreoled(UC4HQUAD_FRONTLEFT,  UC4HOREOLOEDTYPE_STROBE_INPHASE,    UC4HOREOLOEDPERIOD_SLOW, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_FRONTRIGHT, UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_SLOW, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_BACKLEFT,   UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_SLOW, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_BACKRIGHT,  UC4HOREOLOEDTYPE_STROBE_INPHASE,    UC4HOREOLOEDPERIOD_SLOW, RED); //255, 0, 0);
        return;
    }

    // Procedure to set standard rear LED colors in aviation theme
    // Back LEDS White for normal, yellow for GPS not usable, purple for EKF bad
    uint8_t _rear_r, _rear_g, _rear_b;

    if (!(AP_Notify::flags.gps_fusion)) {
        _rear_r = 255;
        _rear_g = 128; //50;
        _rear_b = 0;

    } else if (AP_Notify::flags.ekf_bad) {
        _rear_r = 255;
        _rear_g = 0;
        _rear_b = 255;
    } else {
        _rear_r = 255;
        _rear_g = 255;
        _rear_b = 255;
    }

    // Procedure to set low battery LED output
    // Colors standard
    // Fast strobe alternating front/back
    if (AP_Notify::flags.failsafe_battery) {
        set_oreoled(UC4HQUAD_FRONTLEFT,  UC4HOREOLOEDTYPE_STROBE_INPHASE,    UC4HOREOLOEDPERIOD_FAST, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_FRONTRIGHT, UC4HOREOLOEDTYPE_STROBE_INPHASE,    UC4HOREOLOEDPERIOD_FAST, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_BACKLEFT,   UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_FAST, _rear_r, _rear_g, _rear_b);
        set_oreoled(UC4HQUAD_BACKRIGHT,  UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_FAST, _rear_r, _rear_g, _rear_b);
        return;
    }

//XX    if (_pattern_override) { return; }      // stop here if in mavlink LED control override.

    // Procedure for when Pixhawk is in an autopilot mode
    // Makes all LEDs strobe super fast using standard colors
    if (AP_Notify::flags.autopilot_mode) {
        set_oreoled(UC4HQUAD_FRONTLEFT,  UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, RED); //255, 0, 0);
        set_oreoled(UC4HQUAD_FRONTRIGHT, UC4HOREOLOEDTYPE_STROBE_INPHASE, UC4HOREOLOEDPERIOD_SUPER, GREEN); //0, 255, 0);
        if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
            set_oreoled(UC4HQUAD_BACKLEFT,  UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_SUPER, _rear_r, _rear_g, _rear_b);
            set_oreoled(UC4HQUAD_BACKRIGHT, UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_SUPER, _rear_r, _rear_g, _rear_b);
        } else {
            set_oreoled(UC4HQUAD_BACKLEFT,  UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, _rear_r, _rear_g, _rear_b);
            set_oreoled(UC4HQUAD_BACKRIGHT, UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, _rear_r, _rear_g, _rear_b);
        }
        return;
    }

    // Procedure for when Pixhawk is in a pilot controlled mode
    // All LEDs use standard pattern and colors
    set_oreoled(UC4HQUAD_FRONTLEFT,  UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, RED); //255, 0, 0);
    set_oreoled(UC4HQUAD_FRONTRIGHT, UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, GREEN); //0, 255, 0);
    if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
        set_oreoled(UC4HQUAD_BACKLEFT,  UC4HOREOLOEDTYPE_STROBE_INPHASE,    UC4HOREOLOEDPERIOD_FAST, _rear_r, _rear_g, _rear_b);
        set_oreoled(UC4HQUAD_BACKRIGHT, UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE, UC4HOREOLOEDPERIOD_FAST, _rear_r, _rear_g, _rear_b);
    } else {
        set_oreoled(UC4HQUAD_BACKLEFT,  UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, _rear_r, _rear_g, _rear_b);
        set_oreoled(UC4HQUAD_BACKRIGHT, UC4HOREOLOEDTYPE_SOLID, UC4HOREOLOEDPERIOD_NONE, _rear_r, _rear_g, _rear_b);
    }

    _oreoleds_updated = true;
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

