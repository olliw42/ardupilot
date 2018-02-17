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

    uc4h_notify_type = UC4HNOTIFYTYPE_3RGBLEDS;

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



// _scheduled_update - updates _red, _green, _blue according to notify flags
void Uc4hNotifyDevice::update_slow(void)
{
    // slow rate from 50Hz to 10hz
    uint64_t current_time_ms = AP_HAL::millis64();
    if ((current_time_ms - _task_time_last) < 500) {
        return;
    }
    _task_time_last = current_time_ms;

    _led_task_count++;

    uint16_t b = _led_task_count % 3; //0,1,2

    uint16_t i = (b) % 3;
    _notify_data.rgb3leds.rgb[i][0] = 0x3F;
    _notify_data.rgb3leds.rgb[i][1] = 0;
    _notify_data.rgb3leds.rgb[i][2] = 0;

    i = (b+1) % 3;
    _notify_data.rgb3leds.rgb[i][0] = 0x3F;
    _notify_data.rgb3leds.rgb[i][1] = 0x3F;
    _notify_data.rgb3leds.rgb[i][2] = 0;

    i = (b+2) % 3;
    _notify_data.rgb3leds.rgb[i][0] = 0;
    _notify_data.rgb3leds.rgb[i][1] = 0x3F;
    _notify_data.rgb3leds.rgb[i][2] = 0;

    _updated = true;
}


void Uc4hNotifyDevice::send_CAN_notify_message(void)
{
    if (!_updated) {
        return;
    }
    _updated = false;

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (_ap_uavcan[i] != nullptr) {

            //_ap_uavcan[i]->uc4hnotify_send( 0, 0, (uint8_t*)(&AP_Notify::flags), sizeof(AP_Notify::flags) );

            _ap_uavcan[i]->uc4hnotify_send( uc4h_notify_type, 0, (uint8_t*)(&_notify_data.rgb3leds), 9 );

        }
    }
}

