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
        UC4HNOTIFYTYPE_FLAGS = 0,
        UC4HNOTIFYTYPE_RGBLEDS, //subtype is the number of leds
        UC4HNOTIFYTYPE_OREOLEDS, //subtype is the number of legs (not number of motors)
        UC4HNOTIFYTYPE_SYNC = 255, //send ArduPilots current us time, to allow the nodes to synchronize
    };

    enum UC4HOREOLOEDTYPEENUM {
        UC4HOREOLOEDTYPE_CLEAR = 0,
        UC4HOREOLOEDTYPE_SOLID,
        UC4HOREOLOEDTYPE_STROBE_INPHASE,
        UC4HOREOLOEDTYPE_STROBE_OUTOFPHASE,
        UC4HOREOLOEDTYPE_STROBE, //phase determined by phase
    };

    //from here: https://github.com/rmackay9/oreo-led
    // I suspect that the OreoLED period is in ms, which means that
    // PERIOD_SLOW = 800 ms
    // PERIOD_FAST = 500 ms
    // PERIOD_SUPER = 150 ms
    //since the UC4H Oreo lead period is in units of 25ms, we get
    enum UC4HOREOLOEDPERIODENUM {
        UC4HOREOLOEDPERIOD_NONE = 0,  //is equal to SOLID
        UC4HOREOLOEDPERIOD_SLOW = 32, // 800 ms / 25 ms = 32
        UC4HOREOLOEDPERIOD_FAST = 20, // 500 ms / 25 ms = 20
        UC4HOREOLOEDPERIOD_SUPER = 6, // 150 ms / 25 ms =  6
    };

    enum UC4HQUADPOSITIONENUM {
        UC4HQUAD_FRONTLEFT  = 2, //match to the esc indices of the respective arm
        UC4HQUAD_FRONTRIGHT = 0,
        UC4HQUAD_BACKLEFT   = 1,
        UC4HQUAD_BACKRIGHT  = 3,
    };

private:

    AP_UAVCAN *_ap_uavcan[MAX_NUMBER_OF_CAN_DRIVERS];
    bool _healthy;
    uint64_t _task_time_last; //to slow down
    bool _3rgbleds_updated;
    bool _oreoleds_updated;
    uint64_t _sync_time_last;
    bool _sync_updated;

    void find_CAN(void);
    void send_CAN_notify_message(void);

    void update_slow(void);

    struct {
        struct {
            uint8_t rgb[3][3]; //3x rgb
        } rgb3leds;
        struct {
            uint8_t oreo[4][6]; //4x type, period in 25ms, phase, rgb //XX only QUAD supported currently
        } oreoleds;
        struct {
            uint64_t current_time_ms;
        } sync;
    } _notify_message_data;

    void set_led_rgb(uint8_t lednr, uint8_t r, uint8_t g, uint8_t b);
    void update_3rgbleds(void);

    void set_oreoled(uint8_t lednr, uint8_t type, uint8_t period, uint8_t r, uint8_t g, uint8_t b);
    void set_oreoled(uint8_t lednr, uint8_t type, uint8_t period, uint8_t phase, uint8_t r, uint8_t g, uint8_t b);
    void update_oreoleds(void);

    void update_sync(void);

    uint16_t _led_task_count; //for fooling around
};
