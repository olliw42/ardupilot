#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include "BP_Mount_STorM32.h"

extern const AP_HAL::HAL& hal;


// key for locking UART for exclusive use, preventing any other writes
#define STORM32_UART_LOCK_KEY 0x32426771


//that's the notify class
// singleton to communicate events & flags to the STorM32 mount
BP_Mount_STorM32_Notify* BP_Mount_STorM32_Notify::_singleton;

// constructor
BP_Mount_STorM32_Notify::BP_Mount_STorM32_Notify()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("BP_Mount_STorM32_Notify must be singleton");
    }
    _singleton = this;
}


//that's the main class
// constructor
BP_Mount_STorM32::BP_Mount_STorM32(AP_Mount& frontend, AP_Mount::mount_state& state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    //these need to be initialized to the following values
// mostly doesn't need to be done explicitly, since they are zero, but I like it better to be explicit
    _initialised = false;
    _armed = false;
    _send_armeddisarmed = false;

    _uart = nullptr;

    _task_time_last = 0;
    _task_counter = TASK_SLOT0;

    _bitmask = SEND_STORM32LINK_V2 | SEND_CMD_SETINPUTS | SEND_CMD_DOCAMERA | PASSTHRU_ALLOWED;

    _status.pitch_deg = _status.roll_deg = _status.yaw_deg = 0.0f;

    _target_mode_last = MAV_MOUNT_MODE_RETRACT;

    _pt.uart = nullptr;
    _pt.uart_locked = false;
    _pt.uart_justhaslocked = 0;
    _pt.uart_serialno = 0;
    _pt.send_passthru_installed = false;

    _notify = nullptr;
}


//------------------------------------------------------
// BP_Mount_STorM32 interface functions, ArduPilot Mount
//------------------------------------------------------

// init - performs any required initialisation for this instance
void BP_Mount_STorM32::init(const AP_SerialManager& serial_manager)
{
    //the parameter bitmask is such that zero is default, but the bitmask flags are both positive and negative active
    // so we need to translate
    // do this first, so that e.g. passthrough_install() gets the correct info
    uint8_t param_bitmask = _state._storm32_bitmask.get();
    if (param_bitmask & GET_PWM_TARGET_FROM_RADIO) _bitmask |= GET_PWM_TARGET_FROM_RADIO; //enable

    if (param_bitmask & SEND_STORM32LINK_V2) _bitmask &=~ SEND_STORM32LINK_V2; //disable
    if (param_bitmask & SEND_CMD_SETINPUTS) _bitmask &=~ SEND_CMD_SETINPUTS; //disable
    if (param_bitmask & SEND_CMD_DOCAMERA) _bitmask &=~ SEND_CMD_DOCAMERA; //disable

    if (param_bitmask & PASSTHRU_ALLOWED) _bitmask &=~ PASSTHRU_ALLOWED; //disable
    if (param_bitmask & PASSTHRU_NOTALLOWEDINFLIGHT) _bitmask |= PASSTHRU_NOTALLOWEDINFLIGHT; //enable

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_STorM32_Native, 0);
    if (_uart) {
        _serial_is_initialised = true; //tell the STorM32_lib class
        //we do not set _initialised = true, since we first need to pass find_gimbal(), which sets it
        passthrough_install(serial_manager);
    } else {
        _serial_is_initialised = false; //tell the STorM32_lib class, also switches off finally, and makes find_gimbal() not searching
    }
    _initialised = false; //should be false but can't hurt to ensure that

    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get()); //set mode to default value set by user via parameter
    _target_mode_last = _state._mode;

    //comment: the default "safe" usage is
    // BP_Mount_STorM32_Notify* notify = BP_Mount_STorM32_Notify::instance();
    // if (notify) { XXX; }
    // however, the notify singleton is inited together with BP_Mount_STorM32, and this thus should not be needed here at all
    // to be super safe we check it here and shut off everything if it fails
    // we thereby also store it in a variable to make it a bit more efficient
    _notify = BP_Mount_STorM32_Notify::instance();
    if (!_notify) _serial_is_initialised = false;
}


// update mount position - should be called periodically
// this function must be defined in any case
void BP_Mount_STorM32::update()
{
    passthrough_readback(); //placed here just for debug, we can remove its initialized check by calling it at the end of update()

    if (!_initialised) {
        find_gimbal_native(); //this searches for a gimbal on serial
        return;
    }

    send_text_to_gcs();
}


// 400 Hz loop
void BP_Mount_STorM32::update_fast()
{
    if (!_initialised) {
        return;
    }

    //slow down everything to 100 Hz
    // we can't use update(), since 50 Hz isn't compatible with the desired 20 Hz STorM32Link rate
    // this is not totally correct, it seems the loop is slower than 100 Hz, but just a bit?
    // should I use 9 ms, or better use micros64(), and something like 9900 us? (with 10 ms it might be easy to miss a 400 Hz tick)
    // each message is send at 20 Hz i.e. 50 ms, for 5 task slots => 10 ms per task slot
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _task_time_last) >= 10) {
        _task_time_last = now_ms;

        const uint16_t LIVEDATA_FLAGS = LIVEDATA_STATUS_V2 | LIVEDATA_ATTITUDE_RELATIVE;

        switch (_task_counter) {
            case TASK_SLOT0:
                if (_bitmask & SEND_STORM32LINK_V2) {
                    send_cmd_storm32link_v2(_frontend._ahrs); //2.3ms
                }

                break;
            case TASK_SLOT1: //bracket to avoid compile error: jump to case label
                // trigger live data
                receive_reset_wflush(); //we are brutal and kill all incoming bytes
                send_cmd_getdatafields(LIVEDATA_FLAGS); //0.6ms
                // handle docamera
                if (_notify->actions.camera_trigger_pic) {
                    _notify->actions.camera_trigger_pic = false;
                    if (_bitmask & SEND_CMD_DOCAMERA) send_cmd_docamera(1); //1.0ms
                }

                break;
            case TASK_SLOT2:
                set_target_angles_bymountmode();
                send_target_angles(); //1.7ms or 1.0ms

                break;
            case TASK_SLOT3:
                if (_bitmask & SEND_CMD_SETINPUTS) {
                    send_cmd_setinputs(); //2.4ms
                }

                break;
            case TASK_SLOT4:
                // receive live data
                do_receive(); //we had now 4/5*50ms = 40ms time, this should be more than enough
                if (message_received() && (_serial_in.cmd == 0x06) && (_serial_in.getdatafields.flags == LIVEDATA_FLAGS)) {
                    // attitude angles are in STorM32 convention
                    // convert from STorM32 to ArduPilot convention, this need correction p:-1,r:+1,y:-1
                    set_status_angles_deg(
                            -_serial_in.getdatafields.livedata_attitude.pitch_deg,
                             _serial_in.getdatafields.livedata_attitude.roll_deg,
                            -_serial_in.getdatafields.livedata_attitude.yaw_deg );
                    // we also can check if gimbal is in normal mode
                    bool _armed_new = is_normal_state(_serial_in.getdatafields.livedata_status.state);
                    if (_armed_new != _armed) _send_armeddisarmed = true;
                    _armed = _armed_new;
                    _notify->actions.mount0_armed = _armed;
                }

                break;
        }

        _task_counter++;
        if (_task_counter >= TASK_SLOTNUMBER) _task_counter = 0;
    }
}


// set_mode - sets mount's mode
void BP_Mount_STorM32::set_mode(enum MAV_MOUNT_MODE mode)
{
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}


// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void BP_Mount_STorM32::status_msg(mavlink_channel_t chan)
{
    //it doesn't matter if not _initalised
    // will then send out zeros
    // check nevertheless
    if (!_initialised) {
        return;
    }

    float pitch_deg, roll_deg, yaw_deg;

    get_status_angles_deg(&pitch_deg, &roll_deg, &yaw_deg);

    // MAVLink MOUNT_STATUS: int32_t pitch(deg*100), int32_t roll(deg*100), int32_t yaw(deg*100)
    mavlink_msg_mount_status_send(chan, 0, 0, pitch_deg*100.0f, roll_deg*100.0f, yaw_deg*100.0f);
}


//------------------------------------------------------
// BP_Mount_STorM32 private function
//------------------------------------------------------

void BP_Mount_STorM32::set_target_angles_bymountmode(void)
{
    uint16_t pitch_pwm, roll_pwm, yaw_pwm;

    // flags to trigger sending target angles to gimbal
    bool send_ef_target = false;
    bool send_pwm_target = false;

    // update based on mount mode
    enum MAV_MOUNT_MODE mount_mode = get_mode();

    switch (mount_mode) {
        // move mount to a "retracted" position.
        case MAV_MOUNT_MODE_RETRACT:
            {
                const Vector3f& target = _state._retract_angles.get();
                _angle_ef_target_rad.x = radians(target.x); //values don't matter since recenter is triggered
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                send_ef_target = true;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
                const Vector3f& target = _state._neutral_angles.get();
                _angle_ef_target_rad.x = radians(target.x); //values don't matter since recenter is triggered
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                send_ef_target = true;
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            send_ef_target = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS (?? is the latter comment correct ??)
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            if (_bitmask & GET_PWM_TARGET_FROM_RADIO) {
                get_pwm_target_angles_from_radio(&pitch_pwm, &roll_pwm, &yaw_pwm);
                send_pwm_target = true;
            } else {
                update_targets_from_rc();
                send_ef_target = true;
            }
            if (is_failsafe()) {
                pitch_pwm = roll_pwm = yaw_pwm = 1500;
                _angle_ef_target_rad.y = _angle_ef_target_rad.x = _angle_ef_target_rad.z = 0.0f;
            }
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
                send_ef_target = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // send target angles
    if (send_ef_target) {
        set_target_angles_rad(_angle_ef_target_rad.y, _angle_ef_target_rad.x, _angle_ef_target_rad.z, mount_mode);
    }

    if (send_pwm_target) {
        set_target_angles_pwm(pitch_pwm, roll_pwm, yaw_pwm, mount_mode);
    }
}


void BP_Mount_STorM32::get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm)
{
    get_valid_pwm_from_channel(_state._tilt_rc_in, pitch_pwm);
    get_valid_pwm_from_channel(_state._roll_rc_in, roll_pwm);
    get_valid_pwm_from_channel(_state._pan_rc_in, yaw_pwm);
}


void BP_Mount_STorM32::get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm)
{
    #define rc_ch(i) RC_Channels::rc_channel(i-1)
/*
    if (rc_in && (rc_ch(rc_in))) {
        *pwm = rc_ch(rc_in)->get_radio_in();
    } else {
        *pwm = 1500;
    }
*/
    *pwm = (rc_in && (rc_ch(rc_in))) ? rc_ch(rc_in)->get_radio_in() : 1500;
}


void BP_Mount_STorM32::set_target_angles_deg(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    _target.deg.pitch = pitch_deg;
    _target.deg.roll = roll_deg;
    _target.deg.yaw = yaw_deg;
    _target.type = ANGLES_DEG;
    _target.mode = mount_mode;
}


void BP_Mount_STorM32::set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode)
{
    _target.deg.pitch = degrees(pitch_rad);
    _target.deg.roll = degrees(roll_rad);
    _target.deg.yaw = degrees(yaw_rad);
    _target.type = ANGLES_DEG;
    _target.mode = mount_mode;
}


void BP_Mount_STorM32::set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode)
{
    _target.pwm.pitch = pitch_pwm;
    _target.pwm.roll = roll_pwm;
    _target.pwm.yaw = yaw_pwm;
    _target.type = ANGLES_PWM;
    _target.mode = mount_mode;
}


void BP_Mount_STorM32::send_target_angles(void)
{
    if (_target.mode <= MAV_MOUNT_MODE_NEUTRAL) { //RETRACT and NEUTRAL
        if (_target_mode_last != _target.mode) { // only do it once, i.e., when mode has just changed
            // trigger a recenter camera, this clears all internal Remote states
            // the camera does not need to be recentered explicitly, thus return
            send_cmd_recentercamera();
            _target_mode_last = _target.mode;
        }
        return;
    }

    // update to current mode, to avoid repeated actions on some mount mode changes
    _target_mode_last = _target.mode;

    if (_target.type == ANGLES_PWM) {
        uint16_t pitch_pwm = _target.pwm.pitch;
        uint16_t roll_pwm = _target.pwm.roll;
        uint16_t yaw_pwm = _target.pwm.yaw;

        const uint16_t DZ = 10; //_rc_target_pwm_deadzone;

        if (pitch_pwm < 10) pitch_pwm = 1500;
        if (pitch_pwm < 1500-DZ) pitch_pwm += DZ; else if (pitch_pwm > 1500+DZ) pitch_pwm -= DZ; else pitch_pwm = 1500;

        if (roll_pwm < 10) roll_pwm = 1500;
        if (roll_pwm < 1500-DZ) roll_pwm += DZ; else if (roll_pwm > 1500+DZ) roll_pwm -= DZ; else roll_pwm = 1500;

        if (yaw_pwm < 10) yaw_pwm = 1500;
        if (yaw_pwm < 1500-DZ) yaw_pwm += DZ; else if (yaw_pwm > 1500+DZ) yaw_pwm -= DZ; else yaw_pwm = 1500;

        send_cmd_setpitchrollyaw(pitch_pwm, roll_pwm, yaw_pwm);
    } else {
        float pitch_deg = _target.deg.pitch;
        float roll_deg = _target.deg.roll;
        float yaw_deg = _target.deg.yaw;

        //convert from ArduPilot to STorM32 convention
        // this need correction p:-1,r:+1,y:-1
        send_cmd_setangles(-pitch_deg, roll_deg, -yaw_deg, 0);
    }
}


//------------------------------------------------------
// status angles handlers, angles are in ArduPilot convention
//------------------------------------------------------

void BP_Mount_STorM32::set_status_angles_deg(float pitch_deg, float roll_deg, float yaw_deg)
{
    _status.pitch_deg = pitch_deg;
    _status.roll_deg = roll_deg;
    _status.yaw_deg = yaw_deg;
}


void BP_Mount_STorM32::get_status_angles_deg(float* pitch_deg, float* roll_deg, float* yaw_deg)
{
    *pitch_deg = _status.pitch_deg;
    *roll_deg = _status.roll_deg;
    *yaw_deg = _status.yaw_deg;
}


//------------------------------------------------------
// discovery functions
//------------------------------------------------------

void BP_Mount_STorM32::find_gimbal_native(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

#if FIND_GIMBAL_MAX_SEARCH_TIME_MS
    if (now_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        _serial_is_initialised = false; //switch off STorM32_lib, also switches off finally, and makes find_gimbal() stop searching
        return;
    }
#endif

    if ((now_ms - _task_time_last) > 100) { //try it every 100ms
        _task_time_last = now_ms;

        switch (_task_counter) {
            case TASK_SLOT0:
                // send GETVERSIONSTR
                receive_reset_wflush(); //we are brutal and kill all incoming bytes
                send_cmd_getversionstr();
                break;
            case TASK_SLOT1:
                // receive GETVERSIONSTR response
                do_receive();
                if (message_received() && (_serial_in.cmd == 0x02)) {
                    for (uint16_t n=0;n<16;n++) versionstr[n] = _serial_in.getversionstr.versionstr[n];
                    versionstr[16] = '\0';
                    for (uint16_t n=0;n<16;n++) boardstr[n] = _serial_in.getversionstr.boardstr[n];
                    boardstr[16] = '\0';
                    _task_counter = TASK_SLOT0;
                    _initialised = true;
                    return; //done, get out of here
                }
                break;
            default:
                // skip
                break;
        }
        _task_counter++;
        if (_task_counter >= 3)  _task_counter = 0;
    }
}


void BP_Mount_STorM32::send_text_to_gcs(void)
{
    // if no gimbal was found yet, we skip
    //  if the gcs_send_banner flag is set, it will not be cleared, so that the banner is send in any case at some later point
    if (!_initialised) {
        return;
    }

    if (_notify->actions.gcs_send_banner) {
        _notify->actions.gcs_send_banner = false;
        gcs().send_text(MAV_SEVERITY_INFO, "  STorM32: found and initialized");
        if (strlen(versionstr)) {
            char s[64];
            strcpy(s, "  STorM32: " ); strcat(s, versionstr ); strcat(s, ", " );  strcat(s, boardstr );
            gcs().send_text(MAV_SEVERITY_INFO, s);
        }
        _send_armeddisarmed = true; //also send gimbal state
    }

    if (!_notify->actions.gcs_connection_detected) { //postpone all further sends until a gcs has been detected
        return;
    }

    if (_send_armeddisarmed) {
        _send_armeddisarmed = false;
        gcs().send_text(MAV_SEVERITY_INFO, (_armed) ? "  STorM32: ARMED" : "  STorM32: DISARMED" );
    }

    if (_pt.send_passthru_installed) {
        _pt.send_passthru_installed = false;
        char s[64] = "  STorM32: Passthrough installed on SR0\0";
        s[38] = _pt.uart_serialno + '0';
        gcs().send_text(MAV_SEVERITY_INFO, s);
    }
}


//------------------------------------------------------
// interfaces to BP_STorM32
//------------------------------------------------------
//the _serial functions don't need if (!_serial_is_initialised) return 0; protectors,
// since that's done already inside the STorM32_lib for the public API

size_t BP_Mount_STorM32::_serial_txspace(void)
{
    if (_pt.uart_locked) return 0;

    return (size_t)_uart->txspace();
}


size_t BP_Mount_STorM32::_serial_write(const uint8_t* buffer, size_t size, uint8_t priority)
{
    if (_pt.uart_locked) return 0;

    return _uart->write(buffer, size);
}


uint32_t BP_Mount_STorM32::_serial_available(void)
{
    if (_pt.uart_locked) return 0;

    return _uart->available();
}


int16_t BP_Mount_STorM32::_serial_read(void)
{
    if (_pt.uart_locked) return 0;

    return _uart->read();
}


uint16_t BP_Mount_STorM32::_rcin_read(uint8_t ch)
{
//    if( hal.rcin->in_failsafe() )
//    if( copter.in_failsafe_radio() )
//        return 0;
//    else
// should/can one use also the field ap.rc_receiver_present in addition to failsafe.radio ?
    //this seems to be zero from startup without transmitter, and failsafe didn't helped at all, so leave it as it is
    return hal.rcin->read(ch);
}


//------------------------------------------------------
// helper
//------------------------------------------------------

bool BP_Mount_STorM32::is_failsafe(void)
{
    #define rc_ch(i) RC_Channels::rc_channel(i-1)

    uint8_t roll_rc_in = _state._roll_rc_in;
    uint8_t tilt_rc_in = _state._tilt_rc_in;
    uint8_t pan_rc_in = _state._pan_rc_in;

    if (roll_rc_in && (rc_ch(roll_rc_in)) && (rc_ch(roll_rc_in)->get_radio_in() < 700)) return true;
    if (tilt_rc_in && (rc_ch(tilt_rc_in)) && (rc_ch(tilt_rc_in)->get_radio_in() < 700)) return true;
    if (pan_rc_in && (rc_ch(pan_rc_in)) && (rc_ch(pan_rc_in)->get_radio_in() < 700)) return true;

    return false;
}


//------------------------------------------------------
// BP_Mount_STorM32 passthrough interface
//------------------------------------------------------

//I tried to simply use e.g. MAVLINK_COMM_1, in a hope this would make it work over Telem1, but it somehow didn't
// Why??
// num_gcs() appears to be identical to MAVLINK_COMM_NUM_BUFFERS, at least for Copter
// and GCS::setup_uarts() sets them all up, as many as Mavlink serials are there
// indeed, the protocol gets installed also for MAVLINK_COMM_1, but somehow it doesn't work??
// it somehow seems as if the handler is simply never called
// I have tested that MAVLINK_COMM_1 is indeed the telem1
// if I don't use the line (now_ms - alternative.last_mavlink_ms > protocol_timeout) it works
// it seems that SiK emits mavlink stuff
// also, there are plenty of late coming Mavlink messages going out, needs to be handled in GUI since no flush functions available

void BP_Mount_STorM32::passthrough_install(const AP_SerialManager& serial_manager)
{
    if ((_bitmask & PASSTHRU_ALLOWED) == 0) {
        return;
    }

    int8_t serial_no = _state._storm32_passthru_serialno.get();

    if ((serial_no < 0) || (serial_no >= SERIALMANAGER_NUM_PORTS)) {
        return;
    }

    _pt.uart_serialno = serial_no;

    mavlink_channel_t mav_chan;
    if (!serial_manager.get_mavlink_channel_for_serial(_pt.uart_serialno, mav_chan)) {
        return;
    }

    bool installed = gcs().install_storm32_protocol(
            mav_chan,
            FUNCTOR_BIND_MEMBER(&BP_Mount_STorM32::passthrough_handler, uint8_t, uint8_t, uint8_t, AP_HAL::UARTDriver*)
        );

    if (installed) _pt.send_passthru_installed = true;
}


uint8_t BP_Mount_STorM32::passthrough_handler(uint8_t ioctl, uint8_t b, AP_HAL::UARTDriver* gcs_uart)
{
const char magicopen[] =  "\xFA\x0E\xD2""STORM32CONNECT""\x33\x34";
const char magicclose[] = "\xF9\x11\xD2""STORM32DISCONNECT""\x33\x34";
static uint16_t buf_pos = 0;

    _pt.uart = gcs_uart;

    if (!_initialised) {
        return GCS_MAVLINK::PROTOCOLHANDLER_NONE;
    }

    // NOO, enable this if passthru should not be allowed in flight
    if ((_bitmask & PASSTHRU_NOTALLOWEDINFLIGHT) && hal.util->get_soft_armed()) {
        buf_pos = 0;
        // don't allow pass-through when armed
        if (_pt.uart_locked) {
            _pt.uart->lock_port(0);
            _pt.uart_locked = false;
            return GCS_MAVLINK::PROTOCOLHANDLER_CLOSE;
        }
        return GCS_MAVLINK::PROTOCOLHANDLER_NONE;
    }

    if (ioctl == GCS_MAVLINK::PROTOCOLHANDLER_IOCTL_UNLOCK) {
        _pt.uart->lock_port(0);
        _pt.uart_locked = false;
        return GCS_MAVLINK::PROTOCOLHANDLER_CLOSE;
    }

    //from here on it is the handler for ioctl == GCS_MAVLINK::PROTOCOLHANDLER_IOCTL_CHARRECEIVED
    // since that's the only possible case left, we don't need an if

    uint8_t valid_packet = GCS_MAVLINK::PROTOCOLHANDLER_NONE;

    if (!_pt.uart_locked) {
        if (b == magicopen[0]) { //this assumes that 0xFA is not in the magic anywhere else, is very effective
            buf_pos = 0;
        }
        if ((buf_pos < (sizeof(magicopen)-1)) && (b == magicopen[buf_pos])) { //TODO: we do not correctly handle the crc
            buf_pos++;
            if ((buf_pos >= (sizeof(magicopen)-1)) && _pt.uart->lock_port(STORM32_UART_LOCK_KEY)) {
                buf_pos = 0;
                _pt.uart_locked = true;
                _pt.uart_justhaslocked = 5; //count down
                //should we set valid_packet = GCS_MAVLINK::PROTOCOLHANDLER_OPEN here?? it's a bit delicate, works as is, so let it be
            }
        } else {
            buf_pos = 0;
        }
    } else {
        valid_packet = GCS_MAVLINK::PROTOCOLHANDLER_VALIDPACKET;
        if (!_pt.uart_justhaslocked) {
            _uart->write(&b, 1); //forward to STorM32
        }

        if (b == magicclose[0]) { //this assumes that 0xF9 is not in the magic anywhere else, is very effective
            buf_pos = 0;
        }
        if ((buf_pos < (sizeof(magicclose)-1)) && (b == magicclose[buf_pos])) { //TODO: we do not correctly handle the crc
            buf_pos++;
            if (buf_pos >= (sizeof(magicclose)-1)) {
                buf_pos = 0;
                _pt.uart->lock_port(0);
                _pt.uart_locked = false;
                valid_packet = GCS_MAVLINK::PROTOCOLHANDLER_CLOSE;
            }
        } else {
            buf_pos = 0;
        }
    }

    return valid_packet;
}


void BP_Mount_STorM32::passthrough_readback(void)
{
const char magicack[] = "\xFB\x01\x96\x00\x62\x2E";

    if (!_initialised) {
        return;
    }

    if (!_pt.uart_locked) {
        return;
    }

    uint32_t available = _uart->available();

    if (_pt.uart_justhaslocked) {
        //sadly, the UartDriver API does not provide functional flush functions, bad API
        // so, we can't clean up late Mavlink messages which occur on e.g. SiK telemetry links, and need to handle that in the GUI
        // also, this stupid loop is needed to at least clean up late messages from the STorM32
        // maybe I should add flushes at some point to UARTDriver.h, at least for PX4&ChibiOS, where it is easily possible with
        // the ByteBuffer functions _writebuf.clear(), _readbuf.clear()
        for (uint32_t i = 0; i < available; i++) { //this is to catch late responses from STorM32
            _uart->read();
        }
        _pt.uart_justhaslocked--;
        if (_pt.uart_justhaslocked == 0) {
            _pt.uart->write_locked((uint8_t*)magicack, (sizeof(magicack)-1), STORM32_UART_LOCK_KEY); //acknowledge connect RCCMD
        }
        return;
    }

    for (uint32_t i = 0; i < available; i++) {
        uint8_t c = _uart->read();
        _pt.uart->write_locked(&c, 1, STORM32_UART_LOCK_KEY);
    }
}




