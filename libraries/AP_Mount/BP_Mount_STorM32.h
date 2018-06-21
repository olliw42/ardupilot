/*
  STorM32 mount backend class
 */
#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
//using #if HAL_WITH_UAVCAN doesn't appear to work here, why ???
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "STorM32_lib.h"

#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  90000 //AP's startup has become quite slow, so give it plenty of time, set to 0 to disable

#define STORM32_UAVCAN_NODEID           71 //parameter? can't this be auto-detected?


//undefine to enable only serial support
#define USE_STORM32_UAVCAN


//singleton to communicate events & flags to the STorM32 mount
// resembles AP_Notify
class BP_Mount_STorM32_Notify
{
public:
    // constructor
    BP_Mount_STorM32_Notify();

    // do not allow copies
    BP_Mount_STorM32_Notify(const BP_Mount_STorM32_Notify &other) = delete;
    BP_Mount_STorM32_Notify &operator=(const BP_Mount_STorM32_Notify&) = delete;

    // get singleton instance
    static BP_Mount_STorM32_Notify *instance(void) {
        return _singleton;
    }

    /// bitmask of flags, 'action' is either a flag or an event
    struct bpactions_type {
        //flags
        uint32_t gcs_connection_detected : 1; //this is permanently set once a send_banner() has been done
        uint32_t mount0_armed            : 1; //not used currently, but can be useful in future
        //events
        uint32_t gcs_send_banner         : 1; //this is set by send_banner(), and should be reset by a consumer
        uint32_t camera_trigger_pic      : 1; //this is set by trigger_pic(), and should be reset by a consumer
    };
    struct bpactions_type actions;

private:
    static BP_Mount_STorM32_Notify *_singleton;
};


// that's the main class
class BP_Mount_STorM32 : public AP_Mount_Backend, public STorM32_lib
{

public:
    // constructor
    BP_Mount_STorM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // do not allow copies
    BP_Mount_STorM32(const BP_Mount_STorM32 &other) = delete;
    BP_Mount_STorM32 &operator=(const BP_Mount_STorM32&) = delete;

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();
    virtual void update_fast();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const { return false; }

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    // interface to AP_UAVCAN, receive message
    void handle_storm32status_uavcanmsg_mode(uint8_t mode);
    void handle_storm32status_uavcanmsg_quaternion_frame0(float x, float y, float z, float w);
    void handle_storm32status_uavcanmsg_angles_rad(float roll_rad, float pitch_rad, float yaw_rad);

    // every mount should have this !!!
    virtual bool is_armed(){ return _armed; }

private:
    // BP_Mount_STorM32_Notify instance
    BP_Mount_STorM32_Notify notify_instance;

    // helper to handle corrupt rcin data
    bool is_failsafe(void);

    // interface to STorM32_lib
    size_t _serial_txspace(void) override;
    size_t _serial_write(const uint8_t *buffer, size_t size, uint8_t priority) override;
    uint32_t _serial_available(void) override;
    int16_t _serial_read(void) override;
    uint16_t _rcin_read(uint8_t ch) override;

    // internal variables
#if defined USE_STORM32_UAVCAN && defined USE_UC4H_UAVCAN
    AP_UAVCAN *_ap_uavcan[MAX_NUMBER_OF_CAN_DRIVERS];
#endif
    AP_HAL::UARTDriver *_uart;
    AP_Mount::MountType _mount_type;
    bool _initialised;              // true once the driver has been initialised
    bool _armed;                    // true once the gimbal has reached normal operation state

    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
        TASK_SLOT4,
        TASK_SLOTNUMBER,
    };
    uint32_t _task_time_last;
    uint16_t _task_counter;

    // we want to keep that info, just in case, the fields are 16 in size, so add one to convert it to string
    //  _initialised also indicates that these fields were set
    char versionstr[16+1];
    char namestr[16+1];
    char boardstr[16+1];

    // discovery functions
    void find_CAN(void);
    void find_gimbal_uavcan(void);
    void find_gimbal_native(void);

    // send info to gcs functions
    bool _send_armeddisarmed;       // event: true when a armed/disarmed message should be send out
    void send_text_to_gcs(void);

    // bit mask, allows to enable/disable particular functions/features
    enum BITMASKENUM {
        SEND_STORM32LINK_V2 = 0x01,
        SEND_CMD_SETINPUTS = 0x02,
        GET_PWM_TARGET_FROM_RADIO = 0x04,
        SEND_CMD_DOCAMERA = 0x08,
    };
    uint16_t _bitmask; //this mask is to control some functions

    // storm32.Status in
    struct {
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
    } _status;
    bool _status_updated;

    void set_status_angles_deg(float pitch_deg, float roll_deg, float yaw_deg);
    void get_status_angles_deg(float* pitch_deg, float* roll_deg, float* yaw_deg);

    // target out
    enum ANGLESTYPEENUM {
        ANGLES_DEG = 0, //the STorM32 convention is angles in deg, not rad!
        ANGLES_PWM
    };

    struct {
        enum MAV_MOUNT_MODE mode;
        enum ANGLESTYPEENUM type;
        union {
            struct {
                float pitch;
                float roll;
                float yaw;
            } deg;
            struct {
                uint16_t pitch;
                uint16_t roll;
                uint16_t yaw;
            } pwm;
        };
    } _target;
    bool _target_to_send;
    enum MAV_MOUNT_MODE _target_mode_last;

    void set_target_angles_bymountmode(void);
    void get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm);
    void get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm);
    void set_target_angles_deg(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode); //NOT USED??
    void set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode);
    void set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode);
    void send_target_angles(void);
};
