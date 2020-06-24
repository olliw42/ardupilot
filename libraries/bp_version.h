#pragma once

#define BETAPILOTVERSION " v046rc02"

/*
search for //OW to find all changes

2020.02.29:
 I've upgraded to Copter4.0.3
 but where is also a new thing in uavcan module, and if I submodule-update it the dsdl compiler fails
 so, I just took the changes without updating the submodule
 this compiles, since there doesn't seem to be a new message missing now, there shouldn't be a difference to original

SET_POSITION_TARGET_GLOBAL_INT  Waiting for 3D fix
EKF_STATUS_REPORT

MSG_LOCATION
MSG_POSITION_TARGET_GLOBAL_INT

WPNAV_SPEED

ACCEL_Z CAPACITY TYPE

RC_CHANNELS_OVERRIDE

SYSTEM_TIME

STATUSTEXT

ESTIMATOR_STATUS
EXTENDED_SYS_STATE

*/
