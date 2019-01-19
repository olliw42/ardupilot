#pragma once

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// UC4H

class BP_RangeFinder_UC4H : public AP_RangeFinder_Backend {

public:
    BP_RangeFinder_UC4H(RangeFinder &_frontend, RangeFinder::RangeFinder_State &_state);

    //the other RangeFinders use a static detect() function
    // we don't here, since we can then access ap_uavcan easier
    bool init(void);

    void update(void) override;

    // callback for UAVCAN message
    void handle_uc4hdistance_msg(
            int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id,
            uint8_t range_flag, float range,
            bool sensor_proerties_available, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view
            ) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

private:

    RangeFinder &frontend;

    bool _initialized;

    enum uc4hdistance_flags {
      UC4HDISTANCE_RANGE_INVALID    = 0,
      UC4HDISTANCE_RANGE_VALID      = 1,
      UC4HDISTANCE_RANGE_TOOCLOSE   = 2,
      UC4HDISTANCE_RANGE_TOOFAR     = 3,
    };

    uint8_t _range_flag;
    float _range; //in Meters

    bool _new_distance_received;
    uint32_t _last_reading_ms;

    //note, the BattMonitor class has no such semaphore, the GPS, Baro, and Compass have, why?
    // because the latter are supposed to support multiple nodes? but there is no such systematics in AP_UAVCAN
    // seems to be either a bug, or some inconsistent programming

    AP_HAL::Semaphore* _my_sem;
};
