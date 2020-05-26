//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//*****************************************************

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Mount/STorM32_MAVLink_class.h>


//******************************************************
// STorM32_MAVLink_class class functions
//******************************************************

/// Constructor
STorM32_MAVLink_class::STorM32_MAVLink_class()
{
    _storm32link_seq = 0;
}


bool STorM32_MAVLink_class::is_normal_state(uint16_t state)
{
    return storm32_is_normalstate(state) ? true : false;
}


//------------------------------------------------------
// send stuff
//------------------------------------------------------

void STorM32_MAVLink_class::send_cmd_storm32link_v2(void)
{
#if AP_AHRS_NAVEKF_AVAILABLE

    if (!_tx_hasspace(sizeof(tSTorM32LinkV2))) {
        return;
    }

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

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();
    const AP_GPS &gps = AP::gps();
    const AP_Notify &notify = AP::notify();

    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    uint8_t status = STORM32LINK_FCSTATUS_ISARDUPILOT;
    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (nav_status.flags.horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }
    if (notify.flags.armed) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }

    int16_t yawrate = 0;

    Quaternion quat;
    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());

    Vector3f vel;
    // ahrs.get_velocity_NED(vel) returns a bool, so it's a good idea to consider it
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

    _write( (uint8_t*)(&t), sizeof(tSTorM32LinkV2) );

#endif
}


void STorM32_MAVLink_class::send_cmd_setinputs(void)
{
    if (!_tx_hasspace(sizeof(tSTorM32CmdSetInputs))) {
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

    _write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetInputs) );
}


void STorM32_MAVLink_class::send_cmd_sethomelocation(void)
{
    if (!_tx_hasspace(sizeof(tSTorM32CmdSetHomeTargetLocation))) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    const AP_AHRS &ahrs = AP::ahrs();

    if (ahrs.get_position(location)) {
        status = 0x0001; //= LOCATION_VALID
    }

    tSTorM32CmdSetHomeTargetLocation t;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    storm32_finalize_CmdSetHomeLocation(&t);

    _write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}


void STorM32_MAVLink_class::send_cmd_settargetlocation(void)
{
    if (!_tx_hasspace(sizeof(tSTorM32CmdSetHomeTargetLocation))) {
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

    _write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}


