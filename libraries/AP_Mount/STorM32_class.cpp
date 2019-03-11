//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//*****************************************************

#include <AP_Notify/AP_Notify.h>
#include <AP_Mount/STorM32_class.h>


//******************************************************
// STorM32_class class functions
//******************************************************

/// Constructor
STorM32_class::STorM32_class()
{
    //these need to be initialized to the following values
// but doesn't need to be done explicitly, since they are zero
    _serial_is_initialised = false;
    _storm32link_seq = 0;
    _serial_in.state = SERIALSTATE_IDLE;
}

// determines from the STorM32 state if the gimbal is in a normal operation mode
bool STorM32_class::is_normal_state(uint16_t state)
{
    return storm32_is_normalstate(state) ? true : false;
}


//------------------------------------------------------
// send stuff
//------------------------------------------------------

void STorM32_class::send_cmd_storm32link_v2(const AP_AHRS_TYPE& ahrs)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32LinkV2) +2) {
        return;
    }

    uint8_t status = STORM32LINK_FCSTATUS_ISARDUPILOT;
    //from tests, 2018-02-10/11, I concluded
    // ahrs.healthy():     indicates Q is OK, ca. 15 secs, Q is doing a square dance before, so must wait for this
    // ahrs.initialised(): indicates vz is OK (vx,vy ate OK very early), ca. 30-35 secs
    // ekf_filter_status().flags.vert_vel: ca. 60-XXs, few secs after position_ok() and ca 30-XXs after GPS fix
    //                                     don't know what it really indicates

    //ahrs.get_filter_status(), I think, works only because AP_AHRS_TYPE = AP_AHRS_NavEKF
    // AP_AHRS doesn't have a get_filter_status() method
    // AP_InertialNav_NavEKF:get_filter_status() calls _ahrs_ekf.get_filter_status(status)
    // so I think s = copter.letmeget_ekf_filter_status() and ahrs.get_filter_status(s) should be identical !
    // in a test flight a check for equal never triggered => I assume these are indeed identical !

    // AP_Notify::instance()->flags.initialising: if at all when it is only very briefly at startup true
    // AP_Notify::instance()->armed seems to be identical to copter.letmeget_motors_armed() !!!
    // => copter.letmeget_motors_armed() can be avoided

    // nav_filter_status nav_status = copter.letmeget_ekf_filter_status(); can be replaced, this works identically:
    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    // copter.letmeget_motors_armed(); can be replaced by notify->flags.armed,  works identically:
    AP_Notify* notify = AP_Notify::instance();

    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (nav_status.flags.horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }
    if (notify && (notify->flags.armed)) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }

    int16_t yawrate = 0;

    Quaternion quat;
    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());

    Vector3f vel;
    //ahrs.get_velocity_NED(vel) returns a bool, os it's a good idea to consider it
    if (!ahrs.get_velocity_NED(vel)) { vel.x = vel.y = vel.z = 0.0f; }

    tSTorM32LinkV2 t;
    t.seq = _storm32link_seq; _storm32link_seq++; //this is not really used
    t.status = status;
    t.spare = 0;
    t.yawratecmd = yawrate;
    t.q0 = quat.q1;
    t.q1 = quat.q2;
    t.q2 = quat.q3;
    t.q3 = quat.q4;
    t.vx = vel.x;
    t.vy = vel.y;
    t.vz = vel.z;
    storm32_finalize_STorM32LinkV2(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32LinkV2), PRIORITY_HIGHEST );
}

void STorM32_class::send_cmd_setangles(float pitch_deg, float roll_deg, float yaw_deg, uint16_t flags)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdSetAngles) +2) {
        return;
    }

    tSTorM32CmdSetAngles t;
    t.pitch = pitch_deg;
    t.roll = roll_deg;
    t.yaw = yaw_deg;
    t.flags = flags;
    t.type = 0;
    storm32_finalize_CmdSetAngles(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetAngles) );
}

void STorM32_class::send_cmd_setpitchrollyaw(uint16_t pitch, uint16_t roll, uint16_t yaw)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdSetPitchRollYaw) +2) {
        return;
    }

    tSTorM32CmdSetPitchRollYaw t;
    t.pitch = pitch;
    t.roll = roll;
    t.yaw = yaw;
    storm32_finalize_CmdSetPitchRollYaw(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetPitchRollYaw) );
}

void STorM32_class::send_cmd_recentercamera(void)
{
    send_cmd_setpitchrollyaw(0, 0, 0);
}

void STorM32_class::send_cmd_docamera(uint16_t camera_cmd)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdDoCamera) +2) {
        return;
    }

    tSTorM32CmdDoCamera t;
    t.dummy1 = 0;
    t.camera_cmd = camera_cmd;
    t.dummy2 = 0;
    t.dummy3 = 0;
    t.dummy4 = 0;
    t.dummy5 = 0;
    storm32_finalize_CmdDoCamera(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdDoCamera) );
}

void STorM32_class::send_cmd_setinputs(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdSetInputs) +2) {
        return;
    }

    uint8_t status = 0;

    tSTorM32CmdSetInputs t;
    t.channel0 = _rcin_read(0);
    t.channel1 = _rcin_read(1);
    t.channel2 = _rcin_read(2);
    t.channel3 = _rcin_read(3);
    t.channel4 = _rcin_read(4);
    t.channel5 = _rcin_read(5);
    t.channel6 = _rcin_read(6);
    t.channel7 = _rcin_read(7);
    t.channel8 = _rcin_read(8);
    t.channel9 = _rcin_read(9);
    t.channel10 = _rcin_read(10);
    t.channel11 = _rcin_read(11);
    t.channel12 = _rcin_read(12);
    t.channel13 = _rcin_read(13);
    t.channel14 = _rcin_read(14);
    t.channel15 = _rcin_read(15);
    t.status = status;
    storm32_finalize_CmdSetInputs(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetInputs) );
}

void STorM32_class::send_cmd_sethomelocation(const AP_AHRS_TYPE& ahrs)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdSetHomeTargetLocation) +2) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    if (ahrs.get_position(location)) {
        status = 0x0001; //= LOCATION_VALID
    }

    tSTorM32CmdSetHomeTargetLocation t;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    storm32_finalize_CmdSetHomeLocation(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}

void STorM32_class::send_cmd_settargetlocation(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdSetHomeTargetLocation) +2) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    tSTorM32CmdSetHomeTargetLocation t;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    storm32_finalize_CmdSetTargetLocation(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}

void STorM32_class::send_cmd_getdatafields(uint16_t flags)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdGetDataFields) +2) {
        return;
    }

    tSTorM32CmdGetDataFields t;
    t.flags = flags;
    storm32_finalize_CmdGetDataFields(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdGetDataFields) );
}

void STorM32_class::send_cmd_getversionstr(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32CmdGetVersionStr) +2) {
        return;
    }

    tSTorM32CmdGetVersionStr t;
    storm32_finalize_CmdGetVersionStr(&t);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32CmdGetVersionStr) );
}


//------------------------------------------------------
// receive stuff, only relevant for a real serial, not CAN
//------------------------------------------------------

//------------------------------------------------------
// internal

//reads in one char and processes it
// there is no explicit timeout handling or error handling
// call a flush_rx() and receive_reset() to take care of both of that
void STorM32_class::_do_receive_singlechar(void)
{
    //check for (!_serial_is_initialised) is not needed, since only called from do_receive()

    if (_serial_available() <= 0) { //this broadens the use of the function, play it safe
        return;
    }

    uint8_t c = _serial_read();

    switch (_serial_in.state) {
        case SERIALSTATE_IDLE:
            if (c == 0xFB) { //the outcoming RCcmd start sign was received
                _serial_in.stx = c;
                _serial_in.state = SERIALSTATE_RECEIVE_PAYLOAD_LEN;
            }
            break;

        case SERIALSTATE_RECEIVE_PAYLOAD_LEN:
            _serial_in.len = c;
            _serial_in.state = SERIALSTATE_RECEIVE_CMD;
            break;

        case SERIALSTATE_RECEIVE_CMD:
            _serial_in.cmd = c;
            _serial_in.payload_cnt = 0;
            _serial_in.state = SERIALSTATE_RECEIVE_PAYLOAD;
            break;

        case SERIALSTATE_RECEIVE_PAYLOAD:
            if (_serial_in.payload_cnt >= STORM32_CLASS_RECEIVE_BUFFER_SIZE) {
                _serial_in.state = SERIALSTATE_IDLE; //error, get out of here
                return;
            }

            _serial_in.buf[_serial_in.payload_cnt++] = c;

            if (_serial_in.payload_cnt >= _serial_in.len + 2) { //do expect always a crc
              uint16_t crc = 0; //ignore crc for the moment
              if (crc == 0) {
                  _serial_in.state = SERIALSTATE_MESSAGE_RECEIVED;
              }
            }
            break;

        case SERIALSTATE_MESSAGE_RECEIVED:
        case SERIALSTATE_MESSAGE_RECEIVEDANDDIGESTED:
            break;
    }
}

//------------------------------------------------------
// public

void STorM32_class::receive_reset(void)
{
    _serial_in.state = SERIALSTATE_IDLE;
}

void STorM32_class::receive_reset_wflush(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    //it is claimed a while can't do it, the UARTDriver lib seems to be weak, so do a for
    // the UARTDriver lib doesn't have a rxflush(), and it's flush() is non-functional, hence this stupid looping
    uint32_t available = _serial_available();
    for (uint32_t i = 0; i < available; i++) {
        _serial_read();
    }
    if (_serial_available() > 0) {
        _serial_read();
    }

    _serial_in.state = SERIALSTATE_IDLE;
}

//reads in as many chars as there are there
void STorM32_class::do_receive(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    //it is claimed a while can't do it, the UARTDriver lib seems to be weak, so do a for 
    uint32_t available = _serial_available();
    for (uint32_t i = 0; i < available; i++) {
        _do_receive_singlechar();
    }

    // serial state is reset by a flush_rx() and receive_reset(), so, don't worry further
}

bool STorM32_class::message_received(void)
{
    if (_serial_in.state == SERIALSTATE_MESSAGE_RECEIVED) {
        _serial_in.state = SERIALSTATE_MESSAGE_RECEIVEDANDDIGESTED;
        return true;
    }
    return false;
}

/* Comments as regards to v0.96:

has no F9 handling
=> findGimbal may yield response in case of error, should not matter since ID is checked
=> task loop must be quite different, since every command responds, and commands can't be interlaced

bug in v0.96 (also checked v0.90,v080)
CmdGetVersionStr responds with ID GetData (=5) and not ID GetVersionStr (=2)
might be actually useful to identify version in findGimbal

CmdSetInputs is not existing
CmdStorm32Link is not existing

CmdGetData yields a different data format
*/
