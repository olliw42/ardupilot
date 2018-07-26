//OW
// *****************************************************
// (c) olliw, www.olliw.eu, GPL3
// *****************************************************
// STorM32 mount backend class

//20180717: with the UAVCAN Tunnel code we can remove all CAN dependencies in here, which makes it much simpler

#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include "STorM32_lib.h"

#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  300000 //90000 //AP's startup has become quite slow, so give it plenty of time, set to 0 to disable


//singleton to communicate events & flags to the STorM32 mount
// resembles AP_Notify
class BP_Mount_STorM32_Notify
{
public:
    // constructor
    BP_Mount_STorM32_Notify();

    // do not allow copies
    BP_Mount_STorM32_Notify(const BP_Mount_STorM32_Notify& other) = delete;
    BP_Mount_STorM32_Notify& operator=(const BP_Mount_STorM32_Notify&) = delete;

    // get singleton instance
    static BP_Mount_STorM32_Notify* instance(void) {
        return _singleton;
    }

    // bitmask of flags, 'action' is either a flag or an event
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
    static BP_Mount_STorM32_Notify* _singleton;
};


// that's the main class
class BP_Mount_STorM32 : public AP_Mount_Backend, public STorM32_lib
{

public:
    // constructor
    BP_Mount_STorM32(AP_Mount& frontend, AP_Mount::mount_state& state, uint8_t instance);

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

    // every mount should have this !!!
    virtual bool is_armed(){ return _armed; }

private:
    // BP_Mount_STorM32_Notify instance
    BP_Mount_STorM32_Notify notify_instance;
    BP_Mount_STorM32_Notify* _notify;

    // helper to handle corrupt rcin data
    bool is_failsafe(void);

    // interface to STorM32_lib
    size_t _serial_txspace(void) override;
    size_t _serial_write(const uint8_t* buffer, size_t size, uint8_t priority) override;
    uint32_t _serial_available(void) override;
    int16_t _serial_read(void) override;
    uint16_t _rcin_read(uint8_t ch) override;

    // internal variables
    AP_HAL::UARTDriver* _uart;
    bool _initialised;              // true once the driver has been fully initialised
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
    void find_gimbal_native(void);

    // send info to gcs functions
    bool _send_armeddisarmed;       // event: true when a armed/disarmed message should be send out
    void send_text_to_gcs(void);

    // bit mask, allows to enable/disable particular functions/features
    enum BITMASKENUM {
        GET_PWM_TARGET_FROM_RADIO = 0x01,
        SEND_STORM32LINK_V2 = 0x02,
        SEND_CMD_SETINPUTS = 0x04,
        SEND_CMD_DOCAMERA = 0x08,
        SEND_SOLOGIMBALHEARTBEAT = 0x20,
        PASSTHRU_ALLOWEDINFLIGHT = 0x40,
        PASSTHRU_ALLOWED = 0x80,
    };
    uint16_t _bitmask; //this mask is to control some functions

    // storm32 status in, mount_status out
    struct {
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
    } _status;

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
    enum MAV_MOUNT_MODE _target_mode_last;

    void set_target_angles_bymountmode(void);
    void get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm);
    void get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm);
    void set_target_angles_deg(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode); //not currently used!
    void set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode);
    void set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode);
    void send_target_angles(void);

    // mimic solo gimbal
    uint32_t _sologimbal_send_last;
    void send_sologimbal_heartbeat_msg(mavlink_channel_t chan);

    // passthru
    struct {
        AP_HAL::UARTDriver* uart;
        bool uart_locked;
        uint16_t uart_justhaslocked;
        uint8_t uart_serialno; //only for notification
        bool passthru_installed; //only for notification
        bool send_passthru_installed; //only for notification
    } _pt;
    void passthrough_install(const AP_SerialManager& serial_manager);
    uint8_t passthrough_handler(uint8_t ioctl, uint8_t b, AP_HAL::UARTDriver* gcs_uart);
    void passthrough_readback(void);
}; //end of BP_Mount_STorM32 class
