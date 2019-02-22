//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// Rangefinder backend to handle uc4.Distance UAVCAN messages

#pragma once

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// UC4H
/*
TODO's:
- set the parameters according to the UC4H message info
    such as RNGFND_MAX_CM, RNGFND_MIN_CM, RNGFND_ORIENT
*/

class BP_RangeFinder_UC4H : public AP_RangeFinder_Backend {

public:
    BP_RangeFinder_UC4H(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    //the other RangeFinders use a static detect() function
    // we don't, since we can then access ap_uavcan easier
    bool init(void);

    void update(void) override;

    // this reports the registered compasses to the ground station
    void send_banner(uint8_t instance) override;

    // callback for UAVCAN message
    void handle_uc4hdistance_msg(uint32_t ext_id, int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id, uint8_t range_flag, float range);
    void handle_uc4hdistance_msg_sensorproperties(uint32_t ext_id, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

private:
    uint8_t _instance;

    bool _initialized;
    uint8_t _node_id;
    uint32_t _our_id;
    bool _send_banner;

    void _do_send_banner(void);

    enum uc4hdistance_flags {
      UC4HDISTANCE_RANGE_INVALID    = 0,
      UC4HDISTANCE_RANGE_VALID      = 1,
      UC4HDISTANCE_RANGE_TOOCLOSE   = 2,
      UC4HDISTANCE_RANGE_TOOFAR     = 3,
    };

    uint8_t _range_flag;
    float _range; //in Meters

    bool _new_distance_received;

    //note, the BattMonitor class has no such semaphore, the GPS, Baro, and Compass have, why?
    // because the latter are supposed to support multiple nodes? but there is no such systematics in AP_UAVCAN
    // seems to be either a bug, or some inconsistent programming

    AP_HAL::Semaphore* _my_sem;

    uint32_t _calc_id(int8_t pitch, int8_t yaw, uint8_t sub_id);
    uint32_t _calc_id(uint8_t orientation, uint8_t sub_id);
    int8_t _calc_pitch_from_id(uint32_t id);
    int8_t _calc_yaw_from_id(uint32_t id);
    uint8_t _calc_subid_from_id(uint32_t id);
};
