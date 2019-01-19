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

        uint16_t id = 0;  //, _params._serial_number)) { //XXXX

        if (ap_uavcan->uc4hdistance_register_listener(this, id)) {
           debug_rf_uavcan(2, "UAVCAN RangeFinder Uc4hDistance registered id: %d\n\r", id);
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
             _range = 0.0f; //for the moment use something very small, so that AP_RangeFinder_Backend's update_status() can do the job
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

#endif
