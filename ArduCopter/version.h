#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "APM:Copter V3.6-dev"
#define THISFIRMWARE "BetaCopter V3.6-dev v008"
//OWEND

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV


/*
PAINPOINT:
in order for HAL_MINIMIZE_FEATURES to be respected,
a px4-clean MUST be done when changing from v2 to v3,v4 or vice versa !!!
*/

/*
v0.08:
 - added changes from STorM32nativeonly branch
 - replace AP_Notify mechanism with own singleton -> changes also in: GCS_Common.cpp, AP_Camera.h
 - notify singleton changed to AP_Notify style, needs testing !
 - merge master of 14.06.2018, using master-bcdev-merge method (ensures updated submodules, NO!)
# Conflicts:
#   libraries/AP_BattMonitor/AP_BattMonitor.cpp
#   libraries/AP_BattMonitor/AP_BattMonitor.h
#   libraries/AP_BattMonitor/AP_BattMonitor_Backend.h ->EASY
#   libraries/AP_BattMonitor/AP_BattMonitor_Params.h ->EASY
#   libraries/AP_BattMonitor/AP_BattMonitor_UAVCAN.cpp
#   libraries/AP_BattMonitor/AP_BattMonitor_UAVCAN.h
#   libraries/AP_Notify/AP_Notify.cpp ->EASY
#   libraries/AP_SerialManager/AP_SerialManager.cpp ->EASY
#   libraries/AP_SerialManager/AP_SerialManager.h ->EASY
#   libraries/AP_UAVCAN/AP_UAVCAN.cpp
#   libraries/AP_UAVCAN/AP_UAVCAN.h
#   libraries/GCS_MAVLink/GCS_Common.cpp ->EASY
   merge conflicts mainly in AP_Battmonitor & AP_Uavcan
 - further corrections which appeared during compile test
   AP_UAVCAN::escstatus_update_data()
   => compiles
 - use AP_UAVCAN::get_uavcan()
 - adapt uavcan code to latest style in registers
 - change AP_UAVCAN::escstatus_update_data(), follow AP_BLHeli::read_telemetry_packet()
 - remove _st_
 - 2018.06.21: submodules Chibio, mavlink updated
 - merge master of 20.06.2018, using master-bcdev-merge method
 - two very simple merge conflicts in AP_Battmonitor => compiles
 - BP_Mount_STorM32_Notify bench tested, and is working
 - USE_UC4H_UAVCAN define to disable/enable all extra UAVCAN stuff
 - needs moving #include <AP_UAVCAN/AP_UAVCAN.h> to Uc4hNotifyDevice.h
 - 2018.06.21: testflight flamewheel, passed
 - merged into betacopter-develop
pushed on 21.06.2018as v008-rc1


TODO: GenericBatteryInfo with Wh, or use BatteryInfo in some "dirty" way?


v0.07:
 - UC4HNOTIFYTYPE_FLAGS implemented
 - UC4HNOTIFYTYPE_TEXT, UC4HNOTIFYTYPE_SYNC added
 - UC4HNOTIFYTYPE_RGBLEDS deprecated, and code removed
flight tested with flame wheel 14.Apr.2018
prepare release


v0.06:
APM_Config.h: 1x
GCS_Mavlink.cpp: 1x

AP_Mount.cpp: 2x
AP_Mount.h: 3x
AP_SerialManager.cpp/.h: 1x each to adopt SerialProtocol_STorM32_Native = 84
AP_Camera.cpp: 2x (no change in AP_Camera.h)
AP_Notify.cpp: 3x
AP_Notify.h: 1x
GCS_Common.cpp: 1x

AP_BattMonitor_Backend.h: 1x
AP_BattMonitor_Params.h: 1x
AP_BattMonitor.cpp: 2x
AP_BattMonitor.h: 2x
AP_BattMonitor_UAVCAN.h: added
AP_BattMonitor_UAVCAN.cpp: added
AP_UAVCAN.cpp: 7x
AP_UAVCAN.h: 3x


- AP_Mount_STorM32_UAVCAN renamed to BP_Mount_STorM32
- BP_STorM32.h/cpp: changes to LinkV2
- _frontend._ahrs.get_gps().status() changed to AP::gps().status()
* 2018-02-07: master rebased, master merged to betacopter
- task frequency increased to 20 Hz, with 5 task slots, => 10ms per slot
- letmeget_trigger_pic(),letmeset_trigger_pic(),letmeget_initialised()
- AP_Camera stuff added
- _armed and is_normal() added, to detect if gimbal is operational
the normal serial functions should be complete with that
- ahrs.get_velocity() used to set speed
  what is the difference to inertial_nav???
what is missing a test of the STorM32LinkV2 data
=> accept this as v006-001

- AP_BattMonitor stuff added, needed some rework since code has changed quite a bit
- AP_UAVCAN GenericBatteryInfo message handling added
don't do also the STorM32 UAVCAN messages now, get first the serial up-to-date, can gimbal isn't used anyhow by anyone
- this PR just appeared, https://github.com/ArduPilot/ardupilot/pull/7672, so, don't use inertial_nav but ahrs instead
  => changes made to Mount stuff
flight-tested on flamewheel! passed! 2018-02-08
=> accept this as v006-002

I can't add or commit the changes in /modules/uavcan,
this seems to explain it: https://stackoverflow.com/questions/7726131/git-add-a-is-not-adding-all-modified-files-in-directories
who the f.ck has invented git
- did make some tests to be encapsulate the messages stuff, but *node came in the way, no idea
- AP_UAVCAN EscStatus message handling added
  the telemetry data is logged to the DataFlash
flight-tested on flamewheel! passed! 2018-02-08 evening
=> accept this as v006-003

tried to rebase, but got all sorts of problems, which I associated to having touched the uavcan submodule
thus "new" workaround to place the new .hpp in teh AP_UAVCAN library folder
ATTENTION: it can happen that master doesn't compile, so first check that before merging with betacopter!!!!
* 2018-02-09: master fetch,rebase,push-ed, but master NOT merged to betacopter
- cherry picked 'AP_UAVCAN: refactor RC Out functions', 'AP_UAVCAN: simply do_cyclic'
- increase gimbal find time to 90 secs, startup of ArduPilot is quite slow
- misuse yawrate to send all sorts of ahrs flags for testing, letmeget_ekf_filter_status() reintroduced
flight-tested on flamewheel! passed! 2018-02-10
- bug: the STorM32link frequency seems to be 10 Hz!!! the if ((current_time_ms - _task_time_last) > 10) can't really work
       so, use update_fast() for and slow down to 100Hz
flight-tested on flamewheel! passed! 2018-02-10
- bug: status in STorM32Link message was always zero, ähm, not yet resolved
- played with the various flags, concluded so far
  => wait for ahrs,healthy() to set QFix, takes ca. 15 secs
  => wait for ahrs.initialised() to set AHRSFix, takes ca. 32 secs
  => copter.letmeget_motors_armed() can be useful, since it tells when copter is about to take-off (there is a 2 sec delay)
- played also with the inertial_nac flags
  letmeget_ekf_filter_status(), or copter.inertial_nav.get_filter_status(), and ahrs.get_filter_status() provide identical flags,
  so, this can be removed
  also removed letmeget_position_ok()
- bool return of ahrs.get_velocity_NED(vel) considered now
many flight-tests on flamewheel! passed! 2018-02-10 and 2018-02-11
double-checked with a last short flight
=> accept this as v006-004

- first attempt for final choice of flags
- shut-off protections a bit improved
- STORM32LINKAPFCSTATUSENUM
flight test with flamewheel! storm32link with hdc worked great! 2018-02-12, storm32 firmware v2.36d
  issues with v3, v4, ground speed diverges quickly ????
* 2018-02-12: master fetch,rebase,push,submodule-ed, branch betacoptermerge, master merged to betacoptermerge
  this seems to have resolved the v3,v4 ground speed runaway issue
- for v2 the flash is too small, hence disable nearly everything in APM_Config.h
PAINPOINT: I need to manually switch it off/on, which requires a px4-clean to work !!!!!!!
- test out AP_Notify::instance() flags
test flights with flamewheel (v2) and solo (v3)! all passed! 2018-02-13
=> accept this as v006-005

- use  AP_Notify::instance()->flags.arms instead of letmeget_motors_armed()
- tried to introduce BP_Flags singleton class (following AP_Notify or AP_GPS) to reduce polluting Copter.h,
  but didn't got this to work, no idea why not => init() in system.cpp ???
  thus added a flag to AP_Notify, which works nicely
- AP_Notify flag camera_trigger_pic, changes in AP_Camera
- AP_Notify flag gcs_send_banner, hook into GCS_MAVLINK_Copter::send_banner() in ArduCopter/GCS_Mavlink.cpp
  allows to remove letmeget_initialised(), and is better anyhow
  _startupbanner_status not needed anymore and removed
- => all stuff from Copter.h is now removed !
- added HAL_MINIMIZE_FEATURES to either APM_Config.h or config.h
  tested extensively what happens with APM_Config flags in compiles for different targets
  => in order for HAL_MINIMIZE_FEATURES to be respected, a px4-clean MUST be done when changing from v2 to v3,v4 or vice versa !!!
  => go with changing APM_Config.h, as it is less polluting
=> accept this as v006-006

- STORM32RCCMDENUM added
- armed/disarmed message
- retried BP_Flags approach, and now, for some unclear reason, it works ????
  .init() is not needed, but would probably be better
  I like the approach using AP_Notify better though, since notifiers might want to use the info, it also doesn't pollute the vehicle code
  "my" flags should however probably go into a separate struct
- AP_Notiy approach reworked, uses a  separate struct bpactions now
- move setting bpactions.gcs_connection_detected, bpactions.gcs_send_banner from GCS_MAVLINK_Copter::send_banner() to GCS_MAVLINK::send_banner()
=> the code is totally vehicle independent, zero pollution of vehicle code !!!
I've tested that it compiles fine for ArduPlane
=> should go to BetaPilot ???
the only thing missing now is UAVCAN for STorM32
=> accept this as v006-007

- new STorM32 uavcan dsdl's compiled:
  (i) copy into modules/uavcan/dsdl/uavcan
  (ii) delete folder modules/uavcan/libuavcan/include/dsdlc_generated
  (iii) px4-clean and px4-vX
  (iv) copy .hpp's from dsdlc_generated to AP_UAVCAN folder
  (v) delete folder modules/uavcan/libuavcan/include/dsdlc_generated
  (vi) px4-clean (not required!) and px4-vX
- storm32.Status added
- find_gimbal_uavcan() added, looks for storm32.Status, would be better to look for nodeStatus
- storm32.NodeSpecific added, seems to work on the pix side
- new olliw.uc4h.Notify uavcan dsdl compiled, and placed in AP_UAVCAN
- uc4h.Notify stuff added to AP_UAVCAN, not functional yet, needs an AP_Notify_UAVCAN class
- Uc4HNotifyDevice added, and integrated into Notify as backup, works :)
- uavcan gimbal disabled in mount
I'm not totally happy with the behavior of the 3rd LED
flight-tested several times on 2018-02-18
=> accept this as v006-008

master rebased, 2018-02-21
master-copy compiled, seems to work for v4, so accept
bc-dev-master-merge compiled, seems to work for v4, so accept
merge master into betacopter-develop
- uavcan gimbal enabled
- some improvements in the find_XX() functions
- storm32nodespecific_send() priorities settled
flight test 2018-02-24, flamewheel with uavcan gimbal, passed (needs extra power on CAN 5V!!)
=> accept this as v006-009
- send firmware version only if length>0, i.e., only for serial Mount, not UAVCAN mount

=> accept this as v006
flight-tested 2018-02-24, v2, flame wheel with uavcan gimbal
flight-tested 2018-02-24, v3, solo, without gimbal, with uart gimbal
=> release, 2018.02.25



ap.rc_receiver_present for a better "failsafe" handling ??
how to detect if connected to a GCS ??
copter.ap.initialised_params can this be used to send a banner at the proper time ??
gcs().initialised ???
hook into GCS_MAVLINK_Copter::send_banner() in ArduCopter/GCS_Mavlink.cpp or
hook into GCS_MAVLINK::send_banner() in librariesGCS_MAvlink/GCS_Common.cpp



TODO: do not log packets with error???
TODO: how to autodetect the presence of a CAN STorM32 gimbal, and/or how to get it's UAVCAN node id
TODO: the flags of GenericBatteryInfo should be evaluated
TODO: _rcin_read(), seems to be zero from startup without transmitter, detect failsafe, but how?

Comments:
* SerialManager is also a singleton now, AP_SerialManager *AP_SerialManager::_instance;

*/


/*

ap.initialised:        means that Copter::init_ardupilot() in system.cpp has run through

*/

/*
config.h:
---------

RANGEFINDER_ENABLED     1k
PROXIMITY_ENABLED
OPTFLOW             5k
VISUAL_ODOMETRY_ENABLED     2k
PRECISION_LANDING
PARACHUTE           1k
ADSB_ENABLED

AC_FENCE            2k
AC_RALLY
AC_TERRAIN
AC_AVOID_ENABLED

should already be disabled for v2
SPRAYER
GRIPPER_ENABLED
WINCH_ENABLED
*/


/*
ahrs, nav, gps flags:
---------------------
we need a flag from which we can determine QFIX and AHRSFIX

might be useful to look deeper at Copter::ekf_position_ok(), copter.position_ok()
what is ahrs.healthy() exactly? from  AP_Arming_Copter::gps_checks():
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
    // ensure GPS is ok
    if (!copter.position_ok()) {


    if (ahrs.initialised())                 yawrate |= 0x0001; //this seems to be the main flag to decide when things are OK
                                                               // also vz has settled by then
    if (ahrs.healthy())                     yawrate |= 0x0002; //useful, comes early and third, briefly after Q square dance
    if (ahrs.have_inertial_nav())           yawrate |= 0x0004; //not useful, comes early and second
    if (ahrs.yaw_initialised())             yawrate |= 0x0008; //not useful, comes early and first
    //NO: yaw_initialised() is important as it tells that yaw is OK !!! Q is zero before!!
    //wait also for healthy(), because Q makes a square dance for ca 100 us when have_inertial_nav() raised to true
    //velocity vz takes time to settle to zero, seems to be safely done only when ahrs.initialised() is raised
    //the behavior of these flags is identical indoors as compared to outdors with Stabilize

    //=> wait for healthy(), indicates Q is OK, to set QFix, takes ca. 15 secs
    //=> wait for initialised(), indicates vz is OK, to set AHRSFix, takes ca. 32 secs

    if (copter.letmeget_initialised())      yawrate |= 0x0010; //not useful, comes early and first
    if (copter.letmeget_pream_check())      yawrate |= 0x0020; //not useful, comes "periodically" until arm
    if (copter.letmeget_in_arming_delay())  yawrate |= 0x0040; //can be useful, since it tells when it is in arming delay
    if (copter.letmeget_motors_armed())     yawrate |= 0x0080; //can be useful, since it tells when copter is about to take-off


    status = AP::gps().status() & 0x07; //AP_GPS::GPS_OK_FIX_3D = 3

    //  uint16_t attitude           : 1; // 0 - true if attitude estimate is valid
    //  uint16_t horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
    //  uint16_t vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
    nav_filter_status nav_status = copter.letmeget_ekf_filter_status();
    if (nav_status.flags.attitude)  status |= 0x08; //attitude comes very early, seems to come ca 0.5 sec after ahrs.healthy() is raised
    if (nav_status.flags.horiz_vel) status |= 0x10; //this comes very late, after GPS fix and even few secs after position_ok()
    if (nav_status.flags.vert_vel)  status |= 0x20; //vert_vel comes together with attitude
    if (copter.letmeget_position_ok())  status |= 0x40; //this comes late, e.g. at 60s and ca 30s after GPS fix
    //the behavior of these flags is identical indoors as compared to outdors with Stabilize


=> which flags to use ???  do we really need to wait for vert_vel ??

    ahrs.healthy():     indicates Q is OK, ca. 15 secs
    ahrs.initialised(): indicates vz is OK (vx,vy ate OK very early), ca. 30-35 secs
    ekf_filter_status().flags.vert_vel: ca. 60-XXs, few secs after position_ok() and ca 30-XXs after GPS fix

search on get_velocity_NED() yielded:
            if (_ahrs.get_filter_status(main_ekf_status)) {
                if (main_ekf_status.flags.horiz_vel) {
                    _ahrs.get_velocity_NED(measVelNED);
                }
            }

is _ahrs.get_filter_status(status) identical to copter.letmeget_ekf_filter_status() ??

class AP_AHRS
    virtual bool get_velocity_NED(Vector3f &vec) const
    bool yaw_initialised(void) const
    virtual bool have_inertial_nav(void) const
    virtual bool healthy(void) const = 0;
    virtual bool initialised(void) const
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
        uint8_t likely_flying           : 1;    // 1 if vehicle is probably flying
    } _flags;

class AP_AHRS_DCM : public AP_AHRS
    bool healthy() const override;

//this has the three AHRS, DCM, EKF2, EKF3, this is so to say a container
class AP_AHRS_NavEKF : public AP_AHRS_DCM
    bool have_inertial_nav() const override;
    bool get_velocity_NED(Vector3f &vec) const override;
    bool healthy() const override;
    bool initialised() const override;
    bool get_filter_status(nav_filter_status &status) const;

//AP_InertialNav blends accelerometer data with gps and barometer data to improve altitude and position hold.
class AP_InertialNav
    virtual nav_filter_status get_filter_status() const = 0;
    virtual const Vector3f&    get_velocity() const = 0;

class AP_InertialNav_NavEKF : public AP_InertialNav
    nav_filter_status get_filter_status() const;  => this calls _ahrs_ekf.get_filter_status(status);
    const Vector3f&    get_velocity() const;
    AP_AHRS_NavEKF &_ahrs_ekf;

union nav_filter_status {
    struct {
        uint16_t attitude           : 1; // 0 - true if attitude estimate is valid
        uint16_t horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
        uint16_t vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
        uint16_t horiz_pos_rel      : 1; // 3 - true if the relative horizontal position estimate is valid
        uint16_t horiz_pos_abs      : 1; // 4 - true if the absolute horizontal position estimate is valid
        uint16_t vert_pos           : 1; // 5 - true if the vertical position estimate is valid
        uint16_t terrain_alt        : 1; // 6 - true if the terrain height estimate is valid
        uint16_t const_pos_mode     : 1; // 7 - true if we are in const position mode
        uint16_t pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        uint16_t pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        uint16_t takeoff_detected   : 1; // 10 - true if optical flow takeoff has been detected
        uint16_t takeoff            : 1; // 11 - true if filter is compensating for baro errors during takeoff
        uint16_t touchdown          : 1; // 12 - true if filter is compensating for baro errors during touchdown
        uint16_t using_gps          : 1; // 13 - true if we are using GPS position
        uint16_t gps_glitching      : 1; // 14 - true if the the GPS is glitching
    } flags;


    //from tests, 2018-02-10/11, I concluded
    // ahrs.healthy():     indicates Q is OK, ca. 15 secs
    // ahrs.initialised(): indicates vz is OK (vx,vy ate OK very early), ca. 30-35 secs
    // ekf_filter_status().flags.vert_vel: ca. 60-XXs, few secs after position_ok() and ca 30-XXs after GPS fix
    //                                     don't know what it really indicates
    nav_filter_status nav_status = copter.letmeget_ekf_filter_status();

    //this, I think, works only because AP_AHRS_TYPE = AP_AHRS_NavEKF, AP_AHRS doesn't have get_filter_status() method
    // AP_InertialNav_NavEKF:get_filter_status() calls _ahrs_ekf.get_filter_status(status)
    // so I think nav_status and nav_status2 should be identical !!
    // in a test flight a check for equal never triggered !!!
    // => I assume these are indeed identical !
    nav_filter_status nav_status2;
    ahrs.get_filter_status(nav_status2);

    //in a test flight this never triggered !!!
    //=> I assume these are identical !
    if (nav_status.value != nav_status2.value)        status |= 0x10;
*/




/*
velocity:
----------

velocity vector. I have found two packets which provide that information :
    LOCAL_POSITION_NED ( #32 )
    GLOBAL_POSITION_INT ( #33 )
Local position ned gives you floats with m/s units
global position int gives you int16 with mm/s units
Those are different, but the physical velocity values should be similar.

LOCAL_POSITION_NED
=> sends Vector3f velocity; if (ahrs.get_velocity_NED(velocity)), where ahrs = copter.ahrs
 // return a ground velocity in meters/second, North/East/Down
 // order. This will only be accurate if have_inertial_nav() is
 // true
 virtual bool get_velocity_NED(Vector3f &vec) const {

GLOBAL_POSITION_INT
=> sends  const Vector3f &vel = inertial_nav.get_velocity();  speed in cm/s
in the MAVLink docs it says
vx,vy,vz = Ground speed in NED in cm/s and int16_t


this PR just has appeared today: https://github.com/ArduPilot/ardupilot/pull/7672
important comments

* "inertial-nav library (which we plan to remove and is just a shim on the underlying EKFs)"
=> don't use inertial-nav

* "The GPS position and velocities are already available through the GPS_RAW_INT message
  (albeit using ground course and ground speed instead of a 3D vector)"

=> what does ground speed instead of a 3D vector mean???
answer:
GPS_RAW_INT
it has the field
vel  vel magnitude over ground
cog  course over ground (not heading) in degrees


https://discuss.ardupilot.org/t/velocity-direction-vector-for-the-copter/25330
velocity can be obtained from two different MAVLink messages, LOCAL_POSITION_NED and GLOBAL_POSITION_INT.

LOCAL_POSITION_NED calls ahrs.get_velocity_NED(velocity)
GLOBAL_POSITION_INT calls inertial_nav.get_velocity().
*/

