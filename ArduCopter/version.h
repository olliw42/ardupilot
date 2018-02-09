#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "APM:Copter V3.6-dev"
#define THISFIRMWARE "BetaCopter V3.6-dev v006-003"
//OWEND

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/*
v0.06:
AP_Mount.h: 3x
AP_Mount.cpp: 2x
AP_SerialManager.cpp/.h: 1x each to adopt SerialProtocol_STorM32_Native = 84
AP_Camera.cpp: 2x (no change in AP_Camera.h)
ArduCopter/Copter.h: 1x

AP_BattMonitor_Backend.h: 1x
AP_BattMonitor_Params.h: 1x
AP_BattMonitor.cpp: 2x
AP_BattMonitor.h: 2x
AP_BattMonitor_UAVCAN.h: added
AP_BattMonitor_UAVCAN.cpp: added
AP_UAVCAN.cpp: 5x
AP_UAVCAN.h: 4x

- AP_Mount_STorM32_UAVCAN renamed to BP_Mount_STorM32
- BP_STorM32.h/cpp: changes to LinkV2
- _frontend._ahrs.get_gps().status() changed to AP::gps().status()
- 2018-02-07: master rebased, master merged to betacopter
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
don't do also the STorM32 UAVCAN messages now, get first the serial uptodate, can gimbal isn't used anyhow by anyone
- this PR just appeared, https://github.com/ArduPilot/ardupilot/pull/7672, so, don't use inertial_nav but ahrs instead
  => changes made to Mount stuff
flight-tested on flamewheel! passed! 2018-02-08
=> accept this as v006-002
I can't  add or commit the changes in /modules/uavcan,
this seems to explain it https://stackoverflow.com/questions/7726131/git-add-a-is-not-adding-all-modified-files-in-directories
who the f.ck has invented git
- did make some tests to be encapsulate the messages stuff, but *node came in the way, no idea
- AP_UAVCAN EscStatus message handling added
  the telemetry data is logged to teh DataFlash
flight-tested on flamewheel! passed! 2018-02-08 evening
=> accept this as v006-003
tried to rebase, but got all sorts of problems, which I associated to having touched the uavcan submodule
thus "new" workaround to place the new .hpp in teh AP_UAVCAN library folder
ATTENTION: it can happen that master doesn't compile, so first check that before merging with betacopter!!!!
- 2018-02-09: master fetch,rebase,push-ed, but master NOT merged to betacopter



ap.in_arming_delay instead of motors.armed() ??
ap.rc_receiver_present for a better "failsafe" handling ??
ap.initialised can this be used to send a banner at the proper time ??
how to detect if connected to a GCS ??

TODO: do not log packets with error???
TODO: how to autodetect the presence of a STorM32 gimbal, and/or how to get it's UAVCAN node id
TODO: find_gimbal() also for CAN
TODO: the flags of CircuitStatus an GenericBatteryInfo should be evaluated
*/


/*
Comments:
* SerialManager is also a singleton now, AP_SerialManager *AP_SerialManager::_instance;


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


this PR just has appeared today: https://github.com/ArduPilot/ardupilot/pull/7672
important comments

* "inertial-nav library (which we plan to remove and is just a shim on the underlying EKFs)"
=> don't use inertial-nav

* "The GPS position and velocities are already available through the GPS_RAW_INT message (albeit using ground course and ground speed instead of a 3D vector)"
=> what does ground speed instead of a 3D vector mean???



*/















/*
Hey

a question on the differences of inertial_nav and ahrs, as used in ArduCopter, please

this question is triggered by the recent post
https://discuss.ardupilot.org/t/velocity-direction-vector-for-the-copter/25330
where e.g. the velocity can be obtained from two different MAVLink messages, LOCAL_POSITION_NED and GLOBAL_POSITION_INT.

However, LOCAL_POSITION_NED calls ahrs.get_velocity_NED(velocity), while GLOBAL_POSITION_INT calls inertial_nav.get_velocity().

I would like to better understand now what the differences between these two velocities are (besides the obvious difference in units).

Unfortunately, by scanning a bit through the code this didn't became obvious to me. I in fact find it confusing that in some places ahrs is used while in others inertial_nav, and I failed to see the pattern. I also not that they have quite different approaches towards status flags.
*/

