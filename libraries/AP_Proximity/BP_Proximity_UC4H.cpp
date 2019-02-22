//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "BP_Proximity_UC4H.h"

extern const AP_HAL::HAL& hal;

#define BP_PROXIMITY_UC4HDISTANCE_TIMEOUT_MS 200 // a new distance messages must arrive within this many milliseconds


/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
BP_Proximity_UC4H::BP_Proximity_UC4H(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    _last_update_ms = 0;
    _last_upward_update_ms = 0;

    //we momentarily only have VL53L1x horizontal sensors, so use what UC4H RangeFinder uses itself
    _distance_max = 3.00f; //it is quite noisy above 3 m, so cut it down to 3 m
    _distance_min = 0.02f;
}


// update the state of the sensor
void BP_Proximity_UC4H::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0) || ((AP_HAL::millis() - _last_update_ms) > BP_PROXIMITY_UC4HDISTANCE_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}


// get distance upwards in meters. returns true on success
bool BP_Proximity_UC4H::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms == 0) || ((AP_HAL::millis() - _last_upward_update_ms) > BP_PROXIMITY_UC4HDISTANCE_TIMEOUT_MS)) {
        return false;
    }

    distance = _distance_upward;
    return true;
}


//the angles are in integer, and 15° steps, i.e. 90° = 6, 180° = 12, -90° = -6
//no sensor properties available
void BP_Proximity_UC4H::handle_uc4hdistance_msg(uint32_t ext_id, int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id, uint8_t range_flag, float range)
{
    if (fixed_axis_pitch == -6) {
        //downwards sensor, yaw is irrelevant, gimbal lock

        //we don't handle that in proxi
    }else
    if (fixed_axis_pitch == +6) {
        //upwards sensor, yaw is irrelevant, gimbal lock

        if (range_flag == UC4HDISTANCE_RANGE_VALID) {
            _distance_upward = range;
            _last_upward_update_ms = AP_HAL::millis();
        }
    }else
    if (fixed_axis_pitch == 0) {
        //horizontal sensor
        uint8_t sector = (fixed_axis_yaw >= -1) ? (fixed_axis_yaw + 1) / 3 : (25 - fixed_axis_yaw) / 3; //sector 0...7

        _angle[sector] = sector * 45.0f; //we could be more precise by taking the 15° info into account, but it doesn't matter so ignore
        _distance[sector] = range;
        _distance_valid[sector] = (range_flag == UC4HDISTANCE_RANGE_VALID);

        _last_update_ms = AP_HAL::millis();
        update_boundary_for_sector(sector);
    }
}


void BP_Proximity_UC4H::handle_uc4hdistance_msg_sensorproperties(uint32_t ext_id, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view)
{

}

#endif
