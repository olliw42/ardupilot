//OW
//******************************************************
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"
#include "AP_Notify.h"
#include <AP_UAVCAN/AP_UAVCAN.h>


class Uc4hNotifyDevice: public NotifyDevice {
public:
    Uc4hNotifyDevice();

    // init - initialised the LED
    virtual bool init(void) override;

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    virtual void update() override;

    //this is the type in the UAVCAN message
    enum UC4HNOTIFYTYPEENUM {
        UC4HNOTIFYTYPE_FLAGS = 0, //subtype is the version of the flags structure
        //DEPRECATED UC4HNOTIFYTYPE_RGBLEDS, //subtype is the number of leds
        UC4HNOTIFYTYPE_TEXT = 254,
        UC4HNOTIFYTYPE_SYNC = 255, //send ArduPilots current ms time, to allow the nodes to synchronize
    };

private:
    uint64_t _task_time_last; //to slow down
    bool _flags_updated;
    bool _text_updated;
    uint64_t _sync_time_last;
    bool _sync_updated;

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
};

