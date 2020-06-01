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

#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  0 //300000 //90000 //AP's startup has become quite slow, so give it plenty of time, set to 0 to disable

#define GIMBAL_MANAGER_STATUS_RATE_MS           3000
#define GIMBAL_MANAGER_STATUS_FAST_RATE_MS      250
#define GIMBAL_MANAGER_STATUS_FAST_RATE_CNT     4


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

    // rc channels
    bool is_rc_failsafe(void);
    void send_rc_channels_to_gimbal(void);

    // storm32 mount_status in, mount_status out
    struct {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        float yaw_deg_absolute;
    } _status;

    struct {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        enum MAV_MOUNT_MODE mode;
        enum MAV_MOUNT_MODE mode_last;
    } _target;

    void set_target_angles(void);
    void send_target_angles_to_gimbal(void);

    void send_mount_status_to_channels(void);
    void send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode);

    // gimbal protocol v2
    bool _use_protocolv2;
    bool _is_gimbalmanager;
    bool _sendonly;
    uint8_t _preconfigured_gimbal_device_flags; //0 = none, can become an option

    struct {
        uint16_t capability_flags;
        float tilt_deg_min, tilt_deg_max, pan_deg_min, pan_deg_max;
        uint16_t flags;
        uint32_t failure_flags;
    } _gimbal_device;

    struct {
        uint32_t capability_flags;
        uint32_t flags;
        //private
        bool gimbal_device_info_received; //we cannot serve a GIMBAL_MANAGER_INFORMATION request without having it
        bool gimbal_device_att_status_received; //we block sending a GIMBAL_MANAGER_STATUS, GIMBAL_DEVICE_SET_ATTITUDE without having it
        uint32_t status_time_last; //0 = has not yet ever been sent
        uint8_t status_fast_rate; //0 = normal rate, else counts down
        uint32_t gimbal_device_info_request_time_last;
        uint16_t current_mission_cmd; //0 = none set
    } _gimbal_manager;

    struct NUDGE {
        float pitch_rad, yaw_rad;
    };
    struct NUDGE _companion_nudge;
    struct NUDGE _mission_nudge;
    struct NUDGE _gcs_nudge;
    struct NUDGE _rc_nudge;

    void set_target_angles_v2();
    void send_target_angles_to_gimbal_v2(void);

    enum CLIENTENUM {
        CLIENT_UNKNOWN = 0,
        CLIENT_MISSION,
        CLIENT_GCS,
        CLIENT_COMPANION,
    };

    uint8_t _determine_client(uint8_t sysid, uint8_t compid);
    bool _is_overriding_client(uint8_t client);
    void _copy_gimbal_device_flags(uint16_t gimbal_device_flags);
    void _update_gimbal_manager_flags_from_gimbal_device_flags(uint16_t gimbal_device_flags);
    void _update_gimbal_manager_flags(uint32_t flags, uint8_t client);
    void _update_gimbal_manager_rc(void);
    void _update_gimbal_manager_override(float roll_rad, float pitch_rad, float yaw_rad);
    void _update_gimbal_manager_nudge(float pitch_rad, float yaw_rad, uint8_t client);

    int8_t handle_gimbal_manager_cmd(const mavlink_command_long_t &payload, uint8_t client);
    void handle_gimbal_manager_msg(const mavlink_message_t &msg);

    void send_gimbal_device_set_attitude_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t flags);
    void send_autopilot_state_for_gimbal_device_to_gimbal(void);
    void send_gimbal_manager_status(uint32_t flags);
    void send_gimbal_manager_information(void);
    void send_request_gimbal_device_information_to_gimbal(void);

    // system time
    uint32_t _send_system_time_last;
    void send_system_time_to_gimbal(void);

    // helper
    void send_to_channels(uint32_t msgid, const char *pkt, bool except_gimbal = false);

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
};
