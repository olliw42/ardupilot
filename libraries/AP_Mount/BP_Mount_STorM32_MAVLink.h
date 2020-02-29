//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink
//*****************************************************

#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include "STorM32_MAVLink_class.h"

#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  300000 //90000 //AP's startup has become quite slow, so give it plenty of time, set to 0 to disable


// that's the main class
class BP_Mount_STorM32_MAVLink : public AP_Mount_Backend, public STorM32_MAVLink_class
{
public:
    // Constructor
    BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;
    void update_fast() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return false; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

    // handle_msg - allows to process messages received from gimbal
    void handle_msg(const mavlink_message_t &msg) override;

    // pre arm checks
    bool pre_arm_checks(void) override;

private:
    // internal variables
    bool _initialised;              // true once the driver has been fully initialised
    bool _armed;                    // true once the gimbal has reached normal operation state
    bool _prearmchecks_ok;          // true when the gimbal stops reporting prearm fail

    // internal MAVLink variables
    uint8_t _sysid;                 // system id of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    void find_gimbal(void);

    // storm32 mount_status in, mount_status out
    struct {
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
        float yaw_deg_absolute;
    } _status;

    struct {
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
        enum MAV_MOUNT_MODE mode;
        enum MAV_MOUNT_MODE mode_last;
    } _target;

    void set_target_angles_bymountmode(void);
    void send_do_mount_control_to_gimbal(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mode);
    void send_target_angles_to_gimbal(void);
    bool is_rc_failsafe(void);
    void send_rc_channels_to_gimbal(void);

    // internal task variables
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

    // interface to STorM32_lib
    enum MAV_TUNNEL_PAYLOAD_TYPE_STORM32_ENUM {
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL1_IN = 200, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL1_OUT, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_IN, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_OUT, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3
    };

    bool _tx_hasspace(const size_t size) override;
    size_t _write(const uint8_t* buffer, size_t size) override;
    uint16_t _rcin_read(uint8_t ch_index) override;
};
