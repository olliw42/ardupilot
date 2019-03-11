//******************************************************
//OW
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
        UC4HNOTIFYTYPE_TEXT = 254, //subtype has no meaning, set to 0
        UC4HNOTIFYTYPE_SYNC = 255, //send ArduPilot's current ms time, allows nodes to synchronize, subtype has no meaning, set to 0
    };

private:
    uint64_t _task_time_last; //to slow down
    bool _flags_updated;
    bool _text_updated;
    uint64_t _sync_time_last;
    bool _sync_updated;

    void send_CAN_notify_message(void);

    void update_slow(void);

    #define UC4H_APNOTIFYTYPE_V001  001

    struct __attribute__((packed)) uc4h_apnotify_type_v001 {
      uint32_t initialising       : 1;    // 1 if initialising and copter should not be moved
      uint32_t firmware_update    : 1;    // 1 just before vehicle firmware is updated
      uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
      uint32_t gps_num_sats       : 6;    // number of sats
      uint32_t gps_fusion         : 1;    // 0 = GPS fix rejected by EKF, not usable for flight. 1 = GPS in use by EKF, usable for flight
      uint32_t have_pos_abs       : 1;    // 0 = no absolute position available, 1 = absolute position available
      uint32_t ekf_bad            : 1;    // 1 if ekf is reporting problems
      uint32_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
      uint32_t pre_arm_gps_check  : 1;    // 0 = failing pre-arm GPS checks, 1 = passed
      uint32_t armed              : 1;    // 0 = disarmed, 1 = armed
      uint32_t autopilot_mode     : 1;    // 1 if vehicle is in an autopilot flight mode (only used by OreoLEDs)
      uint32_t flight_mode        : 8;    // flight mode
      uint32_t failsafe_radio     : 1;    // 1 if radio failsafe
      uint32_t failsafe_battery   : 1;    // 1 if battery failsafe
      float    battery_voltage       ;    // battery voltage
    };

    struct __attribute__((packed)) {
        struct uc4h_apnotify_type_v001 flags;
    } _flags_data;

    struct {
        uint64_t current_time_ms;
    } _sync_data;

    char _text_data[NOTIFY_TEXT_BUFFER_SIZE];

    void update_flags(void);
    void update_text(void);

    void update_sync(void);
};

