#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "ArduCopter V3.6.0"
#define THISFIRMWARE "BetaCopter V3.6.0 v013u"
//OW

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

/*
features:
 - STorM32 Native @ 83, complete
 - bitmask parameter handling, MNT_STRM_BM
 - passthrough, MNT_STRM_PTSER
 - any CAN stuff removed, much simplified and streamlined
 - Solo gimbal mimicry

changed files:
    APM_Config.h: low flash for v2, said to be not used anymore, but obviously still works

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
*/
