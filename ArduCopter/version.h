#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

//OW
//#define THISFIRMWARE "APM:Copter V3.6-dev"
#define THISFIRMWARE "BetaCopter V3.6-dev20180207 v006"
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

- 2018-02-07: master rebased, master merged to betacopter








*/
