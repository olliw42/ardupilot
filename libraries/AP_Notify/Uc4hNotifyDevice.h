/*
 *  AP_Notify Library. 
 * based upon a prototype library by David "Buzz" Bussenschutt.
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

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

#ifdef USE_UC4H_UAVCAN


class Uc4hNotifyDevice: public NotifyDevice {
public:
    Uc4hNotifyDevice();

    // init - initialised the LED
    virtual bool init(void);

    // healthy - returns true if the LED is operating properly
    virtual bool healthy() { return _healthy; }

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    virtual void update();

    //this is the type in the UAVCAN message
    enum UC4HNOTIFYTYPEENUM {
        UC4HNOTIFYTYPE_FLAGS = 0, //subtype is the version of the flags structure
        //DEPRECATED UC4HNOTIFYTYPE_RGBLEDS, //subtype is the number of leds
        UC4HNOTIFYTYPE_TEXT = 254,
        UC4HNOTIFYTYPE_SYNC = 255, //send ArduPilots current ms time, to allow the nodes to synchronize
    };

private:

    AP_UAVCAN *_ap_uavcan[MAX_NUMBER_OF_CAN_DRIVERS];
    bool _healthy;
    uint64_t _task_time_last; //to slow down
    bool _flags_updated;
    bool _text_updated;
    uint64_t _sync_time_last;
    bool _sync_updated;

    void find_CAN(void);
    void send_CAN_notify_message(void);

    void update_slow(void);

    struct __attribute__((packed)) {
        uint8_t number_of_arms;
        struct AP_Notify::notify_flags_and_values_type flags;
        struct AP_Notify::notify_events_type events;
    } _flags_data;

    struct {
        uint64_t current_time_ms;
    } _sync_data;

    char _text_data[NOTIFY_TEXT_BUFFER_SIZE];

    void update_flags(void);
    void update_text(void);

    void update_sync(void);

    uint16_t _led_task_count; //for fooling around
};

#endif
