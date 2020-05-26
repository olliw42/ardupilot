//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink
//*****************************************************

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include "BP_Mount_STorM32_MAVLink.h"

extern const AP_HAL::HAL& hal;


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

    _target.mode_last = MAV_MOUNT_MODE_RETRACT;
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
}


// update mount position - should be called periodically
// this function must be defined in any case
void BP_Mount_STorM32_MAVLink::update()
{
    if (!_initialised) {
        find_gimbal();
        return;
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

    // OMG: plane's SCHED_LOOP_RATE is set to 50 Hz, which makes that this loop is also called at just 50 Hz ...
    // do I have to move to 25 Hz ????
    // so, we divide 50 Hz into 4 slots, sand send STorM32Link data at 25 Hz, and gimbal data and rc_channels at 12.5 Hz

    uint32_t now_us = AP_HAL::micros();
    if ((now_us - _task_time_last) >= 10000) {
//        _task_time_last = now_us;
        _task_time_last += 10000;
        if ((now_us - _task_time_last) > 5000) _task_time_last = now_us;

        switch (_task_counter) {
            case TASK_SLOT0:
                send_cmd_storm32link_v2(); //2.3ms
                break;

            case TASK_SLOT1:
                set_target_angles_bymountmode();
                send_target_angles_to_gimbal();
                break;

            case TASK_SLOT2:
                send_cmd_storm32link_v2(); //2.3ms
                break;

            case TASK_SLOT3:
                send_rc_channels_to_gimbal();
                break;
        }

        _task_counter++;
        if (_task_counter > TASK_SLOT3) _task_counter = 0;
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

    if ((msg.sysid != _sysid) || (msg.compid != _compid)) { //this msg is not from our gimbal
        return;
    }

    bool send_status = false;

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
            send_status = true;
            }break;

        case MAVLINK_MSG_ID_MOUNT_STATUS: { //158
            mavlink_mount_status_t payload;
            mavlink_msg_mount_status_decode( &msg, &payload );
            _status.pitch_deg = (float)payload.pointing_a * 0.01f;
            _status.roll_deg = (float)payload.pointing_b * 0.01f;
            _status.yaw_deg = (float)payload.pointing_c * 0.01f;
            _status.yaw_deg_absolute = NAN;
            send_status = true;
            }break;

        case MAVLINK_MSG_ID_MOUNT_ORIENTATION: { //265
            mavlink_mount_orientation_t payload;
            mavlink_msg_mount_orientation_decode( &msg, &payload );
            _status.pitch_deg = payload.pitch;
            _status.roll_deg = payload.roll;
            _status.yaw_deg = payload.yaw;
            _status.yaw_deg_absolute = payload.yaw_absolute;
            send_status = true;
            }break;
    }

    //TODO: I would want to link the emission of send_mount_status() with the incoming status info,
    // but for this one needs to know the chan of the gcs. How to get it?

    // I believe the three alternatives essentially call the same underlying function
    // but the first method is preferred since it deactivates the normal streaming, i.e. is really 1to1

    if (send_status) {
        //const AP_AHRS &ahrs = AP::ahrs();
        //float vehicle_yaw = ahrs.get_yaw();

        //alternative ??
        // const mavlink_button_change_t packet{
        //        time_boot_ms: AP_HAL::millis(),
        //        last_change_ms: uint32_t(last_change_time_ms),
        //        state: last_mask
        // };
        // gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE, (const char *)&packet);
        //?? does this also send to the gimbal???
        //I think it doesn't, from this test:
        // when the STorM32 emits Attitude or MountOrientation, then the GIMBAL component receives Attitude but no MountStatus
        // => I like this !!!
        const mavlink_mount_status_t packet {
            pointing_a: (int32_t)(_status.pitch_deg*100.0f),
            pointing_b: (int32_t)(_status.roll_deg*100.0f),
            pointing_c: (int32_t)(_status.yaw_deg*100.0f),
            target_system: 0,
            target_component: 0
        };
        gcs().send_to_active_channels(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&packet);

        // the "same" without is_active check, but is dangerous as there is no size check
        /*
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
    }
}


bool BP_Mount_STorM32_MAVLink::pre_arm_checks(void)
{
    return _prearmchecks_ok;
}


//------------------------------------------------------
// private functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_do_mount_control_to_gimbal(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mode)
{
    //it doesn't matter if not _initalised, it will then send out zeros
    // also: if not initialized it normally wouldn't be called since checked in update_fast()

    // check we have space for the message
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


void BP_Mount_STorM32_MAVLink::set_target_angles_bymountmode(void)
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
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            set_target = true;
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
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
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
        _target.pitch_deg = degrees(_angle_ef_target_rad.y);
        _target.roll_deg = degrees(_angle_ef_target_rad.x);
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
            send_do_mount_control_to_gimbal(_target.pitch_deg, _target.roll_deg, _target.yaw_deg, _target.mode);
        }
        return;
    }

    // update to current mode, to avoid repeated actions on some mount mode changes
    _target.mode_last = _target.mode;

    send_do_mount_control_to_gimbal(_target.pitch_deg, _target.roll_deg, _target.yaw_deg, _target.mode);
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

    //TODO: this I think only allows one MAVLinbk gimbal
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
    // gladly, we can put it into one tunnel message
    // so check if it fits into the tunnel payload, and if there is space

    if ( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) return false;

    return HAVE_PAYLOAD_SPACE(_chan, TUNNEL);
}


size_t BP_Mount_STorM32_MAVLink::_write(const uint8_t* buffer, size_t size)
{
    //the buffer holds the STorM32 command, which is now wrapped into a MAVLink tunnel message
    // copy it to a new buffer of proper size, and clear it with zeros
    // gladly, it fits completely into one tunnel message, so we get away with this simple method

    uint8_t payload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN+1];
    memset(payload, 0, sizeof(payload));

    if( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) size = MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN; //should not happen, but play it safe
    memcpy(payload, buffer, size);

    mavlink_msg_tunnel_send(_chan,
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



