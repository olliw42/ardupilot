/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <GCS_MAVLink/GCS.h>

#include "BP_RangeFinder_UC4H.h"

extern const AP_HAL::HAL& hal;

#define debug_rf_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)


BP_RangeFinder_UC4H::BP_RangeFinder_UC4H(RangeFinder &_frontend, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_state),
    frontend(_frontend)
{
    _initialized = false;

    _new_distance_received = false;
    _last_reading_ms = 0;

    _my_sem = hal.util->new_semaphore();
}


bool BP_RangeFinder_UC4H::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return false;
    }

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }

        // int4 fixed_axis_pitch         # -PI/2 ... +PI/2 or -6 ... 6
        // int5 fixed_axis_yaw           # -PI ... +PI or -12 ... 12
        // uint4 sensor_sub_id           # Allow up to 16 sensors per orientation
        //uint32_t id = _calc_id(-6, -1, 5);
        uint32_t id = _calc_id(state.orientation.get(), state.address.get());

        if (id == UINT32_MAX) {
            continue;
        }

        if (ap_uavcan->uc4hdistance_register_listener(this, id)) {
           debug_rf_uavcan(2, "UAVCAN RangeFinder Uc4hDistance registered id: %d\n\r", id);
           //_initialized = true; //don't set this here, wait for data to have arrived

           gcs().send_text(MAV_SEVERITY_INFO, "RangeFinder%u: uc4h %u %u %u", _calc_pitch_from_id(id), _calc_yaw_from_id(id), _calc_subid_from_id(id));
           return true;
        }
    }

    return false;
}


//TODO: we could sum up the data, similar to Benewake
void BP_RangeFinder_UC4H::update(void)
{
    if (!_initialized) {
        return;
    }

    if (!_new_distance_received) {
        return;
    }
    _new_distance_received = false;

    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        uint32_t now_ms = AP_HAL::millis();

        if (_range_flag != UC4HDISTANCE_RANGE_INVALID) {
            state.distance_cm = (uint16_t)(_range * 100.0f);
            _last_reading_ms = now_ms;
            update_status();
        } else if ((now_ms - _last_reading_ms) > 200) {
            set_status(RangeFinder::RangeFinder_NoData);
        }

        _my_sem->give();
    }
}


void BP_RangeFinder_UC4H::handle_uc4hdistance_msg(int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id,
        uint8_t range_flag, float range,
        bool sensor_proerties_available, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view)
{
    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

         //this indicates that valid data has arrived
         // this only can happen if AP_RangeFinder_UC4H::init() was successful, so no need for an additional flag
         _initialized = true;

         _range_flag = range_flag;

         switch( range_flag ){
         case UC4HDISTANCE_RANGE_INVALID:
             //skip the data
             break;
         case UC4HDISTANCE_RANGE_VALID:
             _range = range;
             break;
         case UC4HDISTANCE_RANGE_TOOCLOSE:
             _range = 0.1f; //for the moment use something very small, so that AP_RangeFinder_Backend's update_status() can do the job
             break;
         case UC4HDISTANCE_RANGE_TOOFAR:
             _range = 100.00f;  //for the moment use something very large, so that AP_RangeFinder_Backend's update_status() can do the job
             break;
         default:
             //skip the data
             _range_flag = UC4HDISTANCE_RANGE_INVALID; //set this so we can handle it alike
             break;
         }

         _new_distance_received = true; //we set this even if the data was bad, bad data is handled by update()

         _my_sem->give();
    }
}


uint32_t BP_RangeFinder_UC4H::_calc_id(int8_t pitch, int8_t yaw, uint8_t sub_id)
{
    return ((uint32_t)pitch & 0x000000FF) + ( ((uint32_t)yaw & 0x000000FF) << 8 ) + ( ((uint32_t)sub_id & 0x000000FF) << 16 );
}

// @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
uint32_t BP_RangeFinder_UC4H::_calc_id(uint8_t orient, uint8_t sub_id)
{
int8_t pitch, yaw; //the angles are in integer, and 15° steps, i.e. 90° = 6, 180° = 12, -90° = -6

    switch( orient ){
    case ROTATION_NONE: //Forward
        pitch = 0; yaw = 0;
        break;
    case ROTATION_YAW_45: //Forward-Right
        pitch = 0; yaw = 3;
        break;
    case ROTATION_YAW_90: //Right
        pitch = 0; yaw = 6;
        break;
    case ROTATION_YAW_135: //Back-Right
        pitch = 0; yaw = 9;
        break;
    case ROTATION_YAW_180: //Back
        pitch = 0; yaw = 12;
        break;
    case ROTATION_YAW_225: //Back-Left
        pitch = 0; yaw = -9;
        break;
    case ROTATION_YAW_270: //Left
        pitch = 0; yaw = -6;
        break;
    case ROTATION_YAW_315: //Forward-Left
        pitch = 0; yaw = -3;
        break;
//    case ROTATION_ROLL_180:                     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_45:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_90:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_135:             pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_180:                    pitch = 0; yaw = 12; break;
//    case ROTATION_ROLL_180_YAW_225:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_270:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_315:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90:                      pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_45:               pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_90:               pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_135:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270:                     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_45:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_90:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_135:             pitch = 0; yaw = 0; break;
    case ROTATION_PITCH_90: //Up
        pitch = 6;    yaw = 0;
        break;
    case ROTATION_PITCH_270: //Down
        pitch = -6; yaw = 0;
        break;
//    case ROTATION_PITCH_180_YAW_90:             pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_180_YAW_270:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_90:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_PITCH_90:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_90:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_180:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_180:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_270:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_PITCH_270:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_270:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_180_YAW_90:     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_270:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_68_YAW_293:     pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_315:                    pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_315:            pitch = 0; yaw = 0; break;
//    case ROTATION_MAX:                          pitch = 0; yaw = 0; break;
//    case ROTATION_CUSTOM:                       pitch = 0; yaw = 0; break;
    default:
        return UINT32_MAX; //don't accept
    };

    return _calc_id(pitch, yaw, sub_id);
}


int8_t BP_RangeFinder_UC4H::_calc_pitch_from_id(uint32_t id)
{
    return (int8_t)(id & 0x000000FF);
}

int8_t BP_RangeFinder_UC4H::_calc_yaw_from_id(uint32_t id)
{
    return (int8_t)((id >> 8) & 0x000000FF);
}

uint8_t BP_RangeFinder_UC4H::_calc_subid_from_id(uint32_t id)
{
    return (int8_t)((id >> 16) & 0x000000FF);
}

#endif

