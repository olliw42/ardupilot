#pragma once

#define BETAPILOTVERSION " v048-13 "

#define ISV41 1  // flag to swap some code pieces in mode handling which are different for V40 and V41


/*
search for //OW to find all changes

2020.10.30:
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

xshot:
- Copter.cpp:           1x
- Copter.h:             1x
- AP_Button.cpp:        1x
- AP_Button.h:          1x
- AP_Vehicle.h:         1x
- AP_Scripting:         1x (bindings.desc)
- GCS_Common.cpp:       + 1x

Additional Files in library:
- bp_version.h
- AP_Mount/BP_Mount_STorM32_MAVLink.cpp
- AP_Mount/BP_Mount_STorM32_MAVLink.h
- AP_Mount/STorM32_lib.h

-------

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

-------

mavsdk.apSimpleGotoPosIntAltRel(pCntrl_lat, pCntrl_lon, pCntrl_alt) -> mavlink_msg_mission_item_int()
mavsdk.apSetGroundSpeed(pCntrl_speed) -> MAV_CMD_DO_CHANGE_SPEED
if mavsdk.gimbalIsReceiving() then
  gimbalSetPitchYawDeg(pCntrl_pitch, pCntrl_yaw)
else
  mavsdk.apSetYawDeg(pCntrl_yaw) -> MAV_CMD_CONDITION_YAW


SET_POSITION_TARGET_LOCAL_NED


SET_POSITION_TARGET_GLOBAL_INT

MAV_CMD_CONDITION_YAW

MAV_CMD_DO_CHANGE_SPEED

singleton AP_Vehicle method set_target_location boolean Location

singleton AP_Vehicle method set_target_velocity_NED boolean Vector3f -> mode_guided.set_velocity();
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);

singleton AP_Vehicle method set_target_angle_and_climbrate boolean float -180 180 float -90 90 float -360 360 float -FLT_MAX FLT_MAX boolean float -FLT_MAX FLT_MAX
singleton AP_Vehicle method set_steering_and_throttle boolean float -1 1 float -1 1


*/
