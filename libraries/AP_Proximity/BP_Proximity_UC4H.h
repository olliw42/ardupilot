//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// Proximity backend to handle uc4.Distance UAVCAN messages

#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

class BP_Proximity_UC4H : public AP_Proximity_Backend
{

public:
    // constructor
    BP_Proximity_UC4H(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const { return _distance_max; }
    float distance_min() const { return _distance_min; };

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const;

    // callback for UAVCAN messages
    void handle_uc4hdistance_msg(uint32_t ext_id, int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id, uint8_t range_flag, float range);
    void handle_uc4hdistance_msg_sensorproperties(uint32_t ext_id, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view);

private:
    enum uc4hdistance_flags {
      UC4HDISTANCE_RANGE_INVALID    = 0,
      UC4HDISTANCE_RANGE_VALID      = 1,
      UC4HDISTANCE_RANGE_TOOCLOSE   = 2,
      UC4HDISTANCE_RANGE_TOOFAR     = 3,
    };

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last update distance
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward;             // upward distance in meters
};
