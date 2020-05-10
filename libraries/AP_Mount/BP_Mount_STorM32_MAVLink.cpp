//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink
//*****************************************************

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include "BP_Mount_STorM32_MAVLink.h"

extern const AP_HAL::HAL& hal;


#define FLAG_USENEWGIMBALMESSAGES   0x01
#define FLAG_ISGIMBALMANAGER        0x02

//missing gimbal protocol v2 flags

#define GIMBAL_MANAGER_CAP_FLAGS_HAS_ROI_WPNEXT_OFFSET  ((uint32_t)1 << 30)
#define GIMBAL_MANAGER_CAP_FLAGS_HAS_ROI_SYSID          ((uint32_t)1 << 31)

//GIMBAL_MANAGER_FLAGS_NUDGE=2097152, 2^20
//GIMBAL_MANAGER_FLAGS_OVERRIDE=4194304, 2^21
//GIMBAL_MANAGER_FLAGS_NONE=8388608, 2^22
#define GIMBAL_MANAGER_FLAGS_RC_NUDGE    ((uint32_t)1 << 30)
#define GIMBAL_MANAGER_FLAGS_RC_OVERRIDE ((uint32_t)1 << 31)




//******************************************************
// BP_Mount_STorM32_MAVLink, that's the main class
//******************************************************

// constructor
BP_Mount_STorM32_MAVLink::BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    _initialised = false;
    _armed = false;
    _prearmchecks_ok = false; //if we don't find a gimbal, it stays false ! This might not always be the desired behavior

    _sysid = 0;
    _compid = 0;
    _chan = MAVLINK_COMM_0; //this is a dummy, will be set correctly by find_gimbal()

    _task_time_last = 0;
    _task_counter = TASK_SLOT0;

    _send_gimbal_manager_status_time_last = 0;

    _target.mode_last = MAV_MOUNT_MODE_RETRACT;

    _use_protocolv2 = false;
    _is_gimbalmanager = false;
    if (_state._zflags & FLAG_USENEWGIMBALMESSAGES) {
        _use_protocolv2 = true;
        if (_state._zflags & FLAG_ISGIMBALMANAGER) _is_gimbalmanager = true;
    }
}


//------------------------------------------------------
// BP_Mount_STorM32_MAVLink interface functions, ArduPilot Mount
//------------------------------------------------------

// init - performs any required initialisation for this instance
void BP_Mount_STorM32_MAVLink::init(void)
{
    _initialised = false; //should be false but can't hurt to ensure that

    // set mode to default value set by user via parameter
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());

    _gimbal_device.flags = -1;
    _gimbal_manager.gimbal_device_info_received = false;
    _gimbal_manager.gimbal_device_status_received = false;

    _gimbal_manager.capability_flags = 0;
    // the section related to the gimbal device capabilities are determined from the gimbal device information
    // this are our gimbal manager's capabilities
//    _gimbal_manager.capability_flags |= GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL;
//    _gimbal_manager.capability_flags |= GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;
//    _gimbal_manager.capability_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_TRACKING_POINT;
//    _gimbal_manager.capability_flags |= GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_NUDGING;
    _gimbal_manager.capability_flags |= GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_OVERRIDE;

    //gimbal manager flags
    // either take that of gimbal device at startup, or use pre-configured ones
}


// update mount position - should be called periodically
// this function must be defined in any case
void BP_Mount_STorM32_MAVLink::update()
{
    if (!_initialised) {
        find_gimbal();
        return;
    }

    if (!_is_gimbalmanager) return;

    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _send_gimbal_manager_status_time_last) >= GIMBAL_MANAGER_STATUS_RATE_MS) {
        _send_gimbal_manager_status_time_last = now_ms;
        if (_gimbal_manager.gimbal_device_status_received) {
            send_gimbal_manager_status( _gimbal_manager.flags);
        }
    }
}


// 400 Hz loop
void BP_Mount_STorM32_MAVLink::update_fast()
{
    if (!_initialised) {
        return;
    }

    //slow down everything to 100 Hz
    // we can't use update(), since 50 Hz isn't compatible with the desired 20 Hz STorM32Link rate
    // this is not totally correct, it seems the loop is slower than 100 Hz, but just a bit?
    // should I use 9 ms, or better use micros64(), and something like 9900 us? (with 10 ms it might be easy to miss a 400 Hz tick)
    // each message is send at 20 Hz i.e. 50 ms, for 5 task slots => 10 ms per task slot
    uint32_t now_us = AP_HAL::micros();
    if ((now_us - _task_time_last) >= 10000) {
        _task_time_last = now_us;

        switch (_task_counter) {
            case TASK_SLOT0:
                if (_use_protocolv2) {
                    send_autopilot_state_for_gimbal_device_to_gimbal();
                } else {
                    send_cmd_storm32link_v2(); //2.3ms
                }
                break;

            case TASK_SLOT1:
                //old: trigger live data
                //old: handle docamera
                break;

            case TASK_SLOT2:
                if (_use_protocolv2) {
                    set_target_angles_v2();
                    send_target_angles_to_gimbal_v2();
                } else {
                    set_target_angles();
                    send_target_angles_to_gimbal();
                }
                break;

            case TASK_SLOT3:
                //old: send_cmd_setinputs(); //2.4ms
                send_rc_channels_to_gimbal();
                break;

            case TASK_SLOT4:
                //old: receive live data
                break;
        }

        _task_counter++;
        if (_task_counter >= TASK_SLOTNUMBER) _task_counter = 0;
    }
}


// set_mode - sets mount's mode
void BP_Mount_STorM32_MAVLink::set_mode(enum MAV_MOUNT_MODE mode)
{
    if (!_initialised) {
        return;
    }

    _state._mode = mode;
}


// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void BP_Mount_STorM32_MAVLink::send_mount_status(mavlink_channel_t chan)
{
    //it doesn't matter if not _initalised, it will then send out zeros

    //this is called by GCS_MAVLINK_Copter::try_send_message() which in turn is called by the streamer
    // so, it is send out with 2Hz, by the SR0_EXTRA3 setting, which per default is 2Hz only

    //mavlink_msg_mount_status_send(chan, 0, 0, _status.pitch_deg*100.0f, _status.roll_deg*100.0f, _status.yaw_deg*100.0f);
}


// handle_msg - allows to process messages received from gimbal
void BP_Mount_STorM32_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialised) {
        return;
    }

    //TODO: should we check here msg.sysid if it is for us? sounds good, but could also cause issues, so not for now

    //TODO: if we capture a COMMAND_LONG here, what happens with COMMAND_ACK outside of here??
    //comment: we do not bother with sending/handling CMD_ACK momentarily

    // this is a bit dirty, should go to GCS_MAVLINK, GCS, but we don't want to pollute, hence here in dirty ways
    // do we actually get all messages here ????
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: { //76
            if (!_is_gimbalmanager) break;
            mavlink_command_long_t payload;
            mavlink_msg_command_long_decode( &msg, &payload );
            switch (payload.command) {

                case MAV_CMD_REQUEST_MESSAGE: {
                    uint32_t param1 = payload.param1;
                    if (param1 ==  MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
                        if (_gimbal_manager.gimbal_device_info_received) {
                            send_gimbal_manager_information();
                        } else {
                            //we can't do it, we need to send an CMD_ACK
                        }
                    }
                    }break;

                case MAV_CMD_DO_GIMBAL_MANAGER_ATTITUDE: {
                    uint8_t gimbal_device_id = payload.param7;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager
                    float tilt_angle_deg = payload.param3;
                    float pan_angle_deg = payload.param4;

                    _gimbal_manager.active_cmd = MAV_CMD_DO_GIMBAL_MANAGER_ATTITUDE;
                    update_gimbal_manager_flags(payload.param5);

                    if ((tilt_angle_deg == NAN) || (pan_angle_deg == NAN)) break;
                    _angle_ef_target_rad.x = 0.0f; //roll
                    _angle_ef_target_rad.y = radians(tilt_angle_deg);
                    _angle_ef_target_rad.z = radians(pan_angle_deg);
                    }break;

                case MAV_CMD_DO_GIMBAL_MANAGER_TRACK_POINT: {
                    uint8_t gimbal_device_id = payload.param7;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_TRACKING_POINT) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_GIMBAL_MANAGER_TRACK_POINT;
                    } else {
                        //we can't do it, we need to send an CMD_ACK
                    }
                    }break;

                case MAV_CMD_DO_GIMBAL_MANAGER_TRACK_RECTANGLE: {
                    uint8_t gimbal_device_id = payload.param7;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_TRACKING_RECTANGLE) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_GIMBAL_MANAGER_TRACK_RECTANGLE;
                    } else {
                        //we can't do it, we need to send an CMD_ACK
                    }
                    }break;

                case MAV_CMD_DO_SET_ROI_LOCATION: {
                    uint8_t gimbal_device_id = payload.param1;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_SET_ROI_LOCATION;
                    } else
                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_SET_ROI_LOCATION;
                    } else {
                        //we can't do it, we need to send an CMD_ACK
                    }
                    }break;

                case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET: {
                    uint8_t gimbal_device_id = payload.param1;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_ROI_WPNEXT_OFFSET) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET;
                    } else {
                        //we can't do it, we need to send an CMD_ACK
                    }
                    }break;

                case MAV_CMD_DO_SET_ROI_SYSID: {
                    uint8_t gimbal_device_id = payload.param2;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    if (_gimbal_manager.capability_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_ROI_SYSID) {
                        _gimbal_manager.active_cmd = MAV_CMD_DO_SET_ROI_SYSID;
                    } else {
                        //we can't do it, we need to send an CMD_ACK
                    }
                    }break;

                case MAV_CMD_DO_SET_ROI_NONE: {
                    uint8_t gimbal_device_id = payload.param1;
                    if ((gimbal_device_id != _compid) && (gimbal_device_id > 0)) break; //not for our gimbal manager

                    _gimbal_manager.active_cmd = 0;
                    }break;
            }
            }break; //end of case MAVLINK_MSG_ID_COMMAND_LONG

        case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE: { //282
            if (!_is_gimbalmanager) break;
            mavlink_gimbal_manager_set_attitude_t payload;
            mavlink_msg_gimbal_manager_set_attitude_decode( &msg, &payload );
            if ((payload.gimbal_device_id != _compid) && (payload.gimbal_device_id > 0)) break; //not for our gimbal manager

            update_gimbal_manager_flags(payload.flags);

            if ((payload.q[0] == NAN) || (payload.q[1] == NAN) || (payload.q[2] == NAN) || (payload.q[3] == NAN)) break;
            float roll_rad, pitch_rad, yaw_rad;
            Quaternion quat(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
            quat.to_euler(roll_rad, pitch_rad, yaw_rad);
            if (_gimbal_manager.active_cmd) {

                if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_OVERRIDE) { //override has precedence over nudge
                    _angle_ef_target_rad.x = roll_rad;
                    _angle_ef_target_rad.y = pitch_rad;
                    _angle_ef_target_rad.z = yaw_rad;
                } else
                if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_NUDGE) {
                    //we can't yet nudge
                }

            } else {
                _angle_ef_target_rad.x = roll_rad;
                _angle_ef_target_rad.y = pitch_rad;
                _angle_ef_target_rad.z = yaw_rad;
            }
            }break;
    }

    if ((msg.sysid != _sysid) || (msg.compid != _compid)) { //this msg is not from our gimbal
        return;
    }

    bool send_mountstatus = false;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload;
            mavlink_msg_heartbeat_decode( &msg, &payload );
            _armed = is_normal_state(payload.custom_mode & 0xFF);
            if( !(payload.custom_mode & 0x80000000) ){ //we don't follow all changes, but just toggle it to true once
                _prearmchecks_ok = true;
            }
            }break;

        case MAVLINK_MSG_ID_ATTITUDE: { //30
            mavlink_attitude_t payload;
            mavlink_msg_attitude_decode( &msg, &payload );
            _status.pitch_deg = degrees(payload.pitch);
            _status.roll_deg = degrees(payload.roll);
            _status.yaw_deg = degrees(payload.yaw);
            _status.yaw_deg_absolute = NAN;
            send_mountstatus = true;
            _gimbal_device.flags = -1;
            }break;

        case MAVLINK_MSG_ID_MOUNT_STATUS: { //158
            mavlink_mount_status_t payload;
            mavlink_msg_mount_status_decode( &msg, &payload );
            _status.pitch_deg = (float)payload.pointing_a * 0.01f;
            _status.roll_deg = (float)payload.pointing_b * 0.01f;
            _status.yaw_deg = (float)payload.pointing_c * 0.01f;
            _status.yaw_deg_absolute = NAN;
            send_mountstatus = true;
            _gimbal_device.flags = -1;
            }break;

        case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION: { //283
            if (!_is_gimbalmanager) break;
            mavlink_gimbal_device_information_t payload;
            mavlink_msg_gimbal_device_information_decode( &msg, &payload );
            _gimbal_device.capability_flags = payload.cap_flags;
            _gimbal_device.tilt_deg_min = degrees(payload.tilt_min); //hopefully handles also NAN correctly
            _gimbal_device.tilt_deg_max = degrees(payload.tilt_max);
            _gimbal_device.pan_deg_min = degrees(payload.pan_min);
            _gimbal_device.pan_deg_max = degrees(payload.pan_max);
            // copy gimbal device capability flags into gimbal manager capability flags
            _gimbal_manager.capability_flags &=~ 0x0000FFFF; //clear
            _gimbal_manager.capability_flags |= _gimbal_device.capability_flags; //set
            _gimbal_manager.gimbal_device_info_received = true; //inform gimbal manager
            }break;

        case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: { //285
            if (!_use_protocolv2) break;
            mavlink_gimbal_device_attitude_status_t payload;
            mavlink_msg_gimbal_device_attitude_status_decode( &msg, &payload );
            float roll_rad, pitch_rad, yaw_rad;
            Quaternion quat(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
            quat.to_euler(roll_rad, pitch_rad, yaw_rad);
            _status.roll_deg = degrees(roll_rad);
            _status.pitch_deg = degrees(pitch_rad);
            _status.yaw_deg = degrees(yaw_rad);
            _status.yaw_deg_absolute = NAN;
            send_mountstatus = true;
            _gimbal_device.flags = payload.flags;
            _gimbal_device.failure_flags = payload.failure_flags;
            // copy gimbal device flags into gimbal manager flags
            _gimbal_manager.flags &=~ 0x0000FFFF; //clear
            _gimbal_manager.flags |= (uint16_t)_gimbal_device.flags; //set
            _gimbal_manager.gimbal_device_status_received = true; //inform gimbal manager
            }break;
    }

    if (send_mountstatus) {
        //forward to other links, to make MissionPlanner happy
        send_mount_status_to_channels();
    }
}


bool BP_Mount_STorM32_MAVLink::pre_arm_checks(void)
{
    return _prearmchecks_ok;
}


//------------------------------------------------------
// private functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::set_target_angles(void)
{
    bool set_target = false;

    enum MAV_MOUNT_MODE mount_mode = get_mode();

    switch (mount_mode) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
                //this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
                const Vector3f &target = _state._retract_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                set_target = true;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
                //this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                set_target = true;
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            {
                // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
                // NO!!: clear yaw since if has_pan == false the copter will yaw, so we must not forward it to the gimbal
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.z = radians(target.z);
                set_target = true;
            }
            break;

        // RC radio manual angle control, but with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            if (is_rc_failsafe()) {
                _angle_ef_target_rad.y = _angle_ef_target_rad.x = _angle_ef_target_rad.z = 0.0f;
            }
            set_target = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
                set_target = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // set target angles, to communicate to send functions
    // I think this is not needed, as it just duplicates _angle_ef_target_rad, but hey, doesn't harm
    if (set_target) {
        _target.roll_deg = degrees(_angle_ef_target_rad.x);
        _target.pitch_deg = degrees(_angle_ef_target_rad.y);
        _target.yaw_deg = degrees(_angle_ef_target_rad.z);
        _target.mode = mount_mode;
    }
}


// is called by task loop at 20 Hz
void BP_Mount_STorM32_MAVLink::send_target_angles_to_gimbal(void)
{
    if (_target.mode <= MAV_MOUNT_MODE_NEUTRAL) { //RETRACT and NEUTRAL
        // only do it once, i.e., when mode has just changed
        if (_target.mode_last != _target.mode) {
            _target.mode_last = _target.mode;
            send_cmd_do_mount_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, _target.mode);
        }
        return;
    }

    // update to current mode, to avoid repeated actions on some mount mode changes
    _target.mode_last = _target.mode;

    send_cmd_do_mount_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, _target.mode);
}


bool BP_Mount_STorM32_MAVLink::is_rc_failsafe(void)
{
    const RC_Channel *roll_ch = rc().channel(_state._roll_rc_in - 1);
    const RC_Channel *tilt_ch = rc().channel(_state._tilt_rc_in - 1);
    const RC_Channel *pan_ch = rc().channel(_state._pan_rc_in - 1);

    if ((roll_ch != nullptr) && (roll_ch->get_radio_in() < 700)) return true;
    if ((tilt_ch != nullptr) && (tilt_ch->get_radio_in() < 700)) return true;
    if ((pan_ch != nullptr) && (pan_ch->get_radio_in() < 700)) return true;

    return false;
}


void BP_Mount_STorM32_MAVLink::set_target_angles_v2(void)
{
    if (!_is_gimbalmanager) return;
    if (!_gimbal_manager.gimbal_device_status_received) return;

    if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_RETRACT) {
        const Vector3f &target = _state._retract_angles.get();
        _angle_ef_target_rad.x = radians(target.x);
        _angle_ef_target_rad.y = radians(target.y);
        _angle_ef_target_rad.z = radians(target.z);
    } else
    if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) {
        const Vector3f &target = _state._retract_angles.get();
        _angle_ef_target_rad.x = radians(target.x);
        _angle_ef_target_rad.y = radians(target.y);
        _angle_ef_target_rad.z = radians(target.z);
    } else
    if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_RC_OVERRIDE) {
        update_targets_from_rc();
        if (is_rc_failsafe()) {
            _angle_ef_target_rad.x = _angle_ef_target_rad.y = _angle_ef_target_rad.z = 0.0f;
        }
    }

    _target.roll_deg = degrees(_angle_ef_target_rad.x);
    _target.pitch_deg = degrees(_angle_ef_target_rad.y);
    _target.yaw_deg = degrees(_angle_ef_target_rad.z);
}


void BP_Mount_STorM32_MAVLink::send_target_angles_to_gimbal_v2(void)
{
    if (!_is_gimbalmanager) return;
    if (!_gimbal_manager.gimbal_device_status_received) return;

    //this converts gimbal manager flags to gimbal device flags, as these are the lower uni16_t
    uint16_t gimbaldevice_flags = _gimbal_manager.flags;
    send_gimbal_device_set_attitude_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, gimbaldevice_flags);
}



//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_mount_status_to_channels(void)
{
    mavlink_mount_status_t msg = {
        pointing_a : (int32_t)(_status.pitch_deg*100.0f),
        pointing_b : (int32_t)(_status.roll_deg*100.0f),
        pointing_c : (int32_t)(_status.yaw_deg*100.0f),
        target_system : 0,
        target_component : 0 };

    send_to_channels(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg, true);
}


void BP_Mount_STorM32_MAVLink::send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode)
{
    //it doesn't matter if not _initalised, it will then send out zeros
    // also: if not initialized it normally wouldn't be called since checked in update_fast()

    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    //convert from ArduPilot to STorM32 convention
    // this need correction p:-1,r:+1,y:-1
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    //this is send out with _sysid, _compid as target
    // so it can be differentiated by the STorM32 controller from routed commands/messages by looking at source and target
    mavlink_msg_command_long_send(
            _chan,
            _sysid,
            _compid,
            MAV_CMD_DO_MOUNT_CONTROL,
            0,        // confirmation of zero means this is the first time this message has been sent
            pitch_deg,
            roll_deg,
            yaw_deg,
            0, 0, 0,  // param4 ~ param6 unused
            mode);
}


void BP_Mount_STorM32_MAVLink::send_rc_channels_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, RC_CHANNELS)) {
        return;
    }

    mavlink_msg_rc_channels_send(
            _chan,
            AP_HAL::millis(),
            16,
            _rcin_read(0), _rcin_read(1), _rcin_read(2), _rcin_read(3),
            _rcin_read(4), _rcin_read(5), _rcin_read(6), _rcin_read(7),
            _rcin_read(8), _rcin_read(9), _rcin_read(10), _rcin_read(11),
            _rcin_read(12),_rcin_read(13), _rcin_read(14), _rcin_read(15),
            0, 0,
            0);
}


void BP_Mount_STorM32_MAVLink::update_gimbal_manager_flags(uint32_t flags)
{

//XX    _gimbal_manager.flags = (flags & 0xFFFF0000) | (_gimbal_manager.flags & 0x0000FFFF); //don't modify the gimbal device part

    _gimbal_manager.flags = flags;

    if (_gimbal_manager.flags & GIMBAL_MANAGER_FLAGS_NONE) {
        _gimbal_manager.active_cmd = 0;
        _gimbal_manager.flags &=~ GIMBAL_MANAGER_FLAGS_NONE;
    }

/*
    //check for

    _gimbal_manager.flags |= GIMBAL_MANAGER_FLAGS_OVERRIDE;

    uint32_t gimbal_manager_flags = payload.param5;
    if (gimbal_manager_flags & GIMBAL_MANAGER_FLAGS_NONE) { _gimbal_manager.active_cmd = 0; break; }
    _gimbal_manager.active_cmd = MAV_CMD_DO_GIMBAL_MANAGER_ATTITUDE;


    if (gimbal_manager_flags & GIMBAL_MANAGER_FLAGS_NONE) _gimbal_manager.active_cmd = 0;

    _gimbal_manager.flags &=~ (GIMBAL_MANAGER_FLAGS_OVERRIDE | GIMBAL_MANAGER_FLAGS_NUDGE); //clear them
*/
}


//the interface is that of do_mount_control, to make it simpler for the moment, so we internally convert
void BP_Mount_STorM32_MAVLink::send_gimbal_device_set_attitude_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t flags)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

//XX    flags |= GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_ROLL_LOCK;
//XX    flags &=~ GIMBAL_DEVICE_FLAGS_YAW_LOCK;

    Quaternion quat;
    quat.from_euler( radians(roll_deg), radians(pitch_deg), radians(yaw_deg) );
    float q[4];
    q[0] = quat.q1;
    q[1] = quat.q2;
    q[2] = quat.q3;
    q[3] = quat.q4;

    mavlink_msg_gimbal_device_set_attitude_send(
            _chan,
            _sysid, _compid,
            flags,
            q,
            NAN, NAN, NAN);
}


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    Quaternion quat;
    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());
    float q[4] = { quat.q1, quat.q2, quat.q3, quat.q4 }; //figure out how to do it correctly

    Vector3f vel;
    // ahrs.get_velocity_NED(vel) returns a bool, so it's a good idea to consider it
    if (!ahrs.get_velocity_NED(vel)) { vel.x = vel.y = vel.z = 0.0f; }

    float yawrate = NAN; //0.0f;

    //TODO: this are the statuses from using storm32link_v2, we need to somehow use and mimic them
    // we need to figure out the proper/best way
    // STorM32 currently exploits only
    //   STORM32LINK_FCSTATUS_AP_AHRSHEALTHY: => Q ok
    //   STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED: => vz ok
    // let's mimic it here by NANs, but this makes it all very unfortunate dependent, MAVLInk could be nicer
    const AP_GPS &gps = AP::gps();

    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    uint8_t status = STORM32LINK_FCSTATUS_ISARDUPILOT;
    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (nav_status.flags.horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }

    if (!(status & STORM32LINK_FCSTATUS_AP_AHRSHEALTHY)) {
        q[0] = q[1] = q[2] = q[3] = NAN;
    }
    if (!(status & STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED)) {
        vel.x = vel.y = vel.z = NAN;
    }

    mavlink_msg_autopilot_state_for_gimbal_device_send(
            _chan,
            AP_HAL::micros64(),
            _sysid, _compid,
            q,
            0, // uint32_t q_estimated_delay_us,
            vel.x, vel.y, vel.z,
            0, // uint32_t v_estimated_delay_us,
            yawrate);
}


void BP_Mount_STorM32_MAVLink::send_gimbal_manager_status(uint32_t flags)
{
    mavlink_gimbal_manager_status_t msg = {
        time_boot_ms : AP_HAL::millis(),
        flags : flags,
        gimbal_device_id : _compid };

    send_to_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char*)&msg);
}


void BP_Mount_STorM32_MAVLink::send_gimbal_manager_information(void)
{
    mavlink_gimbal_manager_information_t msg = {
        time_boot_ms : AP_HAL::millis(),
        cap_flags : _gimbal_manager.capability_flags,
        tilt_max : radians(_gimbal_device.tilt_deg_max),
        tilt_min : radians(_gimbal_device.tilt_deg_min), //hopefully handles also NAN correctly
        tilt_rate_max : NAN,
        pan_max : radians(_gimbal_device.pan_deg_max),
        pan_min : radians(_gimbal_device.pan_deg_min),
        pan_rate_max : NAN,
        gimbal_device_id : _compid };

    send_to_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION, (char*)&msg);
}


//------------------------------------------------------
// helper
//------------------------------------------------------

// this is essentially GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
// but allows us to excempt the gimbal channel
void BP_Mount_STorM32_MAVLink::send_to_channels(uint32_t msgid, const char *pkt, bool except_gimbal)
{
    const mavlink_msg_entry_t* entry = mavlink_get_msg_entry(msgid);

    if (entry == nullptr) {
        return;
    }

    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        GCS_MAVLINK &c = *gcs().chan(i);

        if (except_gimbal && (c.get_chan() == _chan)) continue;

        if (!c.is_active()) continue;
        if (entry->max_msg_len + GCS_MAVLINK::packet_overhead_chan(c.get_chan()) > c.get_uart()->txspace()) {
            continue; // no room on this channel
        }
        c.send_message(pkt, entry);
    }
}


//------------------------------------------------------
// discovery functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
    uint32_t now_ms = AP_HAL::millis();

#if FIND_GIMBAL_MAX_SEARCH_TIME_MS
    if (now_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        return;
    }
#endif

    //TODO: this I think only allows one MAVLink gimbal
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        _initialised = true;
    }

    //proposal:
    // change this function to allow an index, like find_by_mavtype(index, ....)
    // we then can call it repeatedly until it returns false, whereby increasing index as 0,1,...
    // we then can define that the first mavlink mount is that with lowest ID, and so on
}


//------------------------------------------------------
// interfaces to STorM32_MAVLink_class
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::_tx_hasspace(const size_t size)
{
    if (_use_protocolv2) {
        return false;
    }

    // gladly, we can put it into one tunnel message
    // so check if it fits into the tunnel payload, and if there is space

    if ( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) return false;

    return HAVE_PAYLOAD_SPACE(_chan, TUNNEL);
}


size_t BP_Mount_STorM32_MAVLink::_write(const uint8_t* buffer, size_t size)
{
    if (_use_protocolv2) {
        return 0;
    }

    //the buffer holds the STorM32 command, which is now wrapped into a MAVLink tunnel message
    // copy it to a new buffer of proper size, and clear it with zeros
    // gladly, it fits completely into one tunnel message, so we get away with this simple method

    uint8_t payload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN+1];
    memset(payload, 0, sizeof(payload));

    if( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) size = MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN; //should not happen, but play it safe
    memcpy(payload, buffer, size);

    mavlink_msg_tunnel_send(
            _chan,
            _sysid,
            _compid,
            MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_IN,
            size,
            payload);

    return PAYLOAD_SIZE(_chan, TUNNEL); //we don't know the actual written length, so use this as dummy
}


uint16_t BP_Mount_STorM32_MAVLink::_rcin_read(uint8_t ch_index)
{
    //rc().channel(ch)->get_radio_in() or RC_Channels::get_radio_in(ch) and so on
    // is not the same as hal.rcin->read(), since radio_in can be set by override
    return hal.rcin->read(ch_index);
}









//about sending to other components

// we also could use send_message(msgid, pkt), but sadly it isn't doing a size check

/* what about send_to_components(msgid, pkt, pkt_len), does call routing class
mavlink_command_long_t cmd_msg{};
cmd_msg.command = MAV_CMD_POWER_OFF_INITIATED;
cmd_msg.param1 = i+1;
GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
*/

// this doesn't send it to the GCS!!!
//GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (char*)&msg, sizeof(msg));

/* like void GCS::send_to_active_channels(uint32_t msgid, const char *pkt) but without active channels check
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) return;
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (entry->max_msg_len + c.packet_overhead() > c.get_uart()->txspace()) continue; // no room on this channel
        c.send_message(pkt, entry);
    }
*/
//WHAT exactly is a active channel ????

//send_to_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char*)&msg);

//tested it: sends to all, GCS and gimbal
//gcs().send_to_active_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char*)&msg);




// I believe the three alternatives essentially call the same underlying function
// but the first method is preferred since it deactivates the normal streaming, i.e. is really 1to1

//alternative ??
// const mavlink_button_change_t packet{
//        time_boot_ms: AP_HAL::millis(),
//        last_change_ms: uint32_t(last_change_time_ms),
//        state: last_mask
// };
// gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE, (const char *)&packet);
//?? does this also send to the gimbal??? YES, it does

//gcs().send_to_active_channels(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg);

/* // the "same" without is_active check, but is dangerous as there is no size check
const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_MOUNT_STATUS);
if (entry != nullptr) {
for (uint8_t i=0; i<gcs().num_gcs(); i++) gcs().chan(i)->send_message((const char*)&packet, entry);
} */

//alternative ??
// GCS_MAVLINK::try_send_message(MSG_MOUNT_STATUS);
//gcs().send_message(MSG_MOUNT_STATUS);

//alternative ??
// GCS_MAVLINK::send_mount_status();
//for (uint8_t i=0; i<gcs().num_gcs(); i++) gcs().chan(i)->send_mount_status();
