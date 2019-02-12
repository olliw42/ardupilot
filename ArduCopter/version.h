#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "ArduCopter V3.6.6-rc2"
#define THISFIRMWARE "BetaCopter V3.6.6-rc2 v018u rf10"
//OWEND

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,6,FIRMWARE_VERSION_TYPE_RC

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 6
#define FW_TYPE FIRMWARE_VERSION_TYPE_RC

/*
features:
 - STorM32 Native @ 83, complete
 - bitmask parameter handling, MNT_STRM_BM
 - passthrough, MNT_STRM_PTSER
 - Solo gimbal mimicry
 - all UC4H stuff added
 - barometer proper gcs sendtext
 - UC4H uavcan EscStatus: simplified DataFlash, BattMonitor type 84, 3 BattMonitors, MAVLink EscStatus
 - UC4H uavcan RangeFinder: type 83

20181226:
 - cell voltages for UC4H PowerBrick, AP_UAVCAN, AP_BattMonitor_UAVCAN, AP_BattMonitor_Backend, LogStructure, GCS_Common
   workaround to MP bug: send BATTERY_STATUS only for instance 0
 - mag send_banner()
20190112:
 - workaround to MP bug could be removed, since resolved in MP beta
20190202:
 - uc4h rangefinder stuff
20190211:
 - new tunnel stuff merged
 - rename to _singleton, get_singleton merged
20190212:
 - support 10 range finders, PR#8816 integrated, with two bugs corrected


changed files:

    APM_Config.h: low flash for v2, said to be not used anymore, but obviously still works
    AP_Baro.cpp: 2x


RangeFinder stuff:
    RangeFinder_Backend.h: 1x
    RangeFinder.cpp: 3x
    RangeFinder.h: 2x
    PR#8816 integrated, with two bugs corrected: all RangeFinder files affected


MAG stuff:
    AP_Compass.cpp: 2x
    AP_Compass.h: 1x
    GCS_Common.cpp: +1x


STorM32 stuff:
    AP_Camera.cpp: 2x
    AP_HAL_PX4 UARTDriver.cpp: 3x
    AP_HAL_PX4 UARTDriver.h: 2x
    AP_Mount.cpp: 3x
    AP_Mount.h: 4x
    AP_SerialManager.cpp: 2x
    AP_SerialManager.h: 2x
    GCS_Common.cpp: 4x
    GCS.cpp: 1x
    GCS.h: 3x

added files:
    BP_Mount_STorM32.cpp
    BP_Mount_STorM32.h
    STorM32_class.cpp
    STorM32_class.h
    STorM32_lib.h


UC4H stuff:
    ArduCopter.cpp: 1x
    Copter.h: 2x
    AP_BattMonitor_Backend.h: 1x
    AP_BattMonitor_Params.h: 1x
    AP_BattMonitor_UAVCAN.cpp: 4x
    AP_BattMonitor_UAVCAN.h: 3x
    AP_BattMonitor.cpp: 2x
    AP_BattMonitor.h: 1x
    AP_SerialManager.cpp: +3x
    AP_SerialManager.h: +2x
    AP_Notify.cpp: 2x
    AP_UAVCAN.cpp: 6x
    AP_UAVCAN.h: 3x
    DataFlash/LogStructure.h: 3x
    GCS_Common.cpp: +2x
added files:
    Uc4hNotifyDevice.cpp            (AP_Notify/)
    Uc4hNotifyDevice.h              (AP_Notify/)
    BP_Tunnel_Backend.h             (AP_UAVCAN/)
    BP_UavcanEscStatusManager.cpp   (AP_UAVCAN/)
    BP_UavcanEscStatusManager.h     (AP_UAVCAN/)
    BP_UavcanTunnelManager.cpp      (AP_UAVCAN/)
    BP_UavcanTunnelManager.h        (AP_UAVCAN/)
    TunnelUARTDriver.cpp            (AP_UAVCAN/)
    TunnelUARTDriver.h              (AP_UAVCAN/)
    some more in AP_UAVCAN

git submodule uavcan had to be updated to 72d4b9a, to get the new tunnel.Broadcast in
*/
