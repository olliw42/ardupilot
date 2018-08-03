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

    /// uc4h = "old" notify_flags_type
    struct uc4h_notify_flags_and_values_type {
        uint32_t initialising       : 1;    // 1 if initialising and copter should not be moved
        uint32_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint32_t gps_num_sats       : 6;    // number of sats
        uint32_t flight_mode        : 8;    // flight mode
        uint32_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint32_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
        uint32_t pre_arm_gps_check  : 1;    // 0 = failing pre-arm GPS checks, 1 = passed
        uint32_t save_trim          : 1;    // 1 if gathering trim data
        uint32_t esc_calibration    : 1;    // 1 if calibrating escs
        uint32_t failsafe_radio     : 1;    // 1 if radio failsafe
        uint32_t failsafe_battery   : 1;    // 1 if battery failsafe
        uint32_t parachute_release  : 1;    // 1 if parachute is being released
        uint32_t ekf_bad            : 1;    // 1 if ekf is reporting problems
        uint32_t autopilot_mode     : 1;    // 1 if vehicle is in an autopilot flight mode (only used by OreoLEDs)
        uint32_t firmware_update    : 1;    // 1 just before vehicle firmware is updated
        uint32_t compass_cal_running: 1;    // 1 if a compass calibration is running
        uint32_t leak_detected      : 1;    // 1 if leak detected
        float    battery_voltage       ;    // battery voltage
        uint32_t gps_fusion         : 1;    // 0 = GPS fix rejected by EKF, not usable for flight. 1 = GPS in use by EKF, usable for flight
        uint32_t gps_glitching      : 1;    // 1 if GPS glitching is affecting navigation accuracy
        uint32_t have_pos_abs       : 1;    // 0 = no absolute position available, 1 = absolute position available

        // additional flags
        uint32_t external_leds      : 1;    // 1 if external LEDs are enabled (normally only used for copter)
        uint32_t vehicle_lost       : 1;    // 1 when lost copter tone is requested (normally only used for copter)
        uint32_t waiting_for_throw  : 1;    // 1 when copter is in THROW mode and waiting to detect the user hand launch
        uint32_t powering_off       : 1;    // 1 when the vehicle is powering off
        uint32_t video_recording    : 1;    // 1 when the vehicle is recording video
    };

    /// uc4h = "old" notify_events_type
    struct uc4h_notify_events_type {
        uint32_t arming_failed          : 1;    // 1 if copter failed to arm after user input
        uint32_t user_mode_change       : 1;    // 1 if user has initiated a flight mode change
        uint32_t user_mode_change_failed: 1;    // 1 when user initiated flight mode change fails
        uint32_t failsafe_mode_change   : 1;    // 1 when failsafe has triggered a flight mode change
        uint32_t autotune_complete      : 1;    // 1 when autotune has successfully completed
        uint32_t autotune_failed        : 1;    // 1 when autotune has failed
        uint32_t autotune_next_axis     : 1;    // 1 when autotune has completed one axis and is moving onto the next
        uint32_t mission_complete       : 1;    // 1 when the mission has completed successfully
        uint32_t waypoint_complete      : 1;    // 1 as vehicle completes a waypoint
        uint32_t initiated_compass_cal  : 1;    // 1 when user input to begin compass cal was accepted
        uint32_t compass_cal_saved      : 1;    // 1 when compass calibration was just saved
        uint32_t compass_cal_failed     : 1;    // 1 when compass calibration has just failed
        uint32_t compass_cal_canceled   : 1;    // 1 when compass calibration was just canceled
        uint32_t tune_started           : 1;    // tuning a parameter has started
        uint32_t tune_next              : 3;    // tuning switched to next parameter
        uint32_t tune_save              : 1;    // tuning saved parameters
        uint32_t tune_error             : 1;    // tuning controller error
    };

    struct __attribute__((packed)) {
        uint8_t number_of_arms;
        struct uc4h_notify_flags_and_values_type flags;
        struct uc4h_notify_events_type events;
    } _flags_data;

    struct {
        uint64_t current_time_ms;
    } _sync_data;

    char _text_data[NOTIFY_TEXT_BUFFER_SIZE];

    void update_flags(void);
    void update_text(void);

    void update_sync(void);
};

