#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "APM:Copter V3.6-dev"
#define THISFIRMWARE "BetaCopter V3.6-dev v006-001"
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


Comments:
* struct PACKED log_Esc in LogStructure.h has ints, those units are not clear, use floats as elsewhere
* this struct is used in DataFlash_Class::Log_Write_ESC() in LogFile.cpp
  it allows 8 ESCs, LOG_ESC1_MSG...LOG_ESC8_MSG
  it uses a concept of 'esc_address', which needs to be larger than zero for logging, what is this???
  it clarifies the units
                    rpm         : (int16_t)(esc_status.esc[i].esc_rpm/10),
                    voltage     : (int16_t)(esc_status.esc[i].esc_voltage*100.0f + .5f),
                    current     : (int16_t)(esc_status.esc[i].esc_current*100.0f + .5f),
                    temperature : (int16_t)(esc_status.esc[i].esc_temperature*100.0f + .5f)


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

*/


/*
 AP_UAVCAN.h: 3x
 AP_UAVCAN.cpp: 6x
 AP_BattMonitor_Backend.h: 1x
 AP_BattMonitor.h: 1x
 AP_BattMonitor.cpp: 1x
 AP_BattMonitor_UAVCAN.h: 2x
 AP_BattMonitor_UAVCAN.cpp: 2x

ap.in_arming_delay instead of motors.armed() ??
ap.rc_receiver_present for a better "failsafe" handling ??
ap.initialised can this be used to send a banner at the proper time ??
how to detect if connected to a GCS ??

TODO: how to autodetect the presence of a STorM32 gimbal, and/or how to get it's UAVCAN node id
TODO: find_gimbal() also for CAN
TODO: the flags of CircuitStatus an GenericBatteryInfo should be evaluated
*/
