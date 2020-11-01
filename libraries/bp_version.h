#pragma once

#define BETAPILOTVERSION " v048rc05 "

#define ISV41 1


/*
search for //OW to find all changes

2020.10.27:
 upgraded to Copter4.1.-dev m20201030

ArduCopter specific
- compassmot.cpp:       1x
- GCS_Mavlink.cpp:      1x  (+1 comment, no change)
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

Effect of SYSID_MYGCS parameter value:

rc_channels_override (all)
manual_contol (copter)
these require sysid = SYSID_MYGCS

accept_packet
only true if sysid = SYSID_MYGCS, if SYSID_ENFORCE = 1 (per default it is 0, so all sysid's are accepted)

failsafe_gcs_check
reseted by heartbeat from SYSID_MYGCS

-------

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

BUTTON_CHANGE MANUAL_CONTROL

*/
