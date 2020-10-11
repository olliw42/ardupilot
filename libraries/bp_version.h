#pragma once

#define BETAPILOTVERSION " v046"

/*
search for //OW to find all changes

2020.10.03:
 upgraded to Copter4.0.4

ArduCopter specific
- APM_Config.h:         1x
- compassmot.cpp:       1x
- GCS_Mavlink.cpp:      2x
- mode_guided.cpp:      1x
- version.h:            1x

Libraries:
- AP_Arming.cpp:        3x
- AP_Arming.h:          1x
- AP_Mount_Backend.cpp: (+ 3 comments, no change)
- AP_Mount_Backend.h:   1x  (+1 comment, no change)
- AP_Mount.cpp:         4x  (+1 comment, no change)
- AP_Mount.h:           5x
- GCS_Common.cpp:       8x

Additional Files in library:
- bp_version.h
- AP_Mount/BP_Mount_STorM32_MAVLink.cpp
- AP_Mount/BP_Mount_STorM32_MAVLink.h
- AP_Mount/STorM32_lib.h

SET_POSITION_TARGET_GLOBAL_INT  Waiting for 3D fix
EKF_STATUS_REPORT

MSG_LOCATION
MSG_POSITION_TARGET_GLOBAL_INT

WPNAV_SPEED

ACCEL_Z CAPACITY TYPE

RC_CHANNELS_OVERRIDE
RC_CHANNELS

SYSTEM_TIME

STATUSTEXT

ESTIMATOR_STATUS
EXTENDED_SYS_STATE

MAV_CMD_DO_SET_ROI
MAV_CMD_DO_MOUNT_CONTROL

*/
