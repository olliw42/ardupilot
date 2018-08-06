#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>


class AP_Airspeed_UAVCAN : public AP_Airspeed_Backend
{
public:
    /// Constructor
    AP_Airspeed_UAVCAN(AP_Airspeed& frontend, uint8_t _instance);

    /// Destructor
    //~AP_Airspeed_UAVCAN() override;  //THIS class is hopefully never destructed!!! why would it

    // probe and initialise the sensor
    bool init(void) override;
    
    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float& pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float& temperature) override;
    
    virtual void handle_rawairdata_msg(float differential_pressure, float differential_pressure_sensor_temperature) override;

private:
    bool _initialized;

    uint32_t _last_sample_time_ms;

    float _pressure; // in Pascal
    float _temperature; // in Celsius

    //note, the BattMonitor class has no such semaphore, the GPS, Baro, and Compass have, why?
    // because the latter are supposed to support multiple nodes? but there is no such systematics in AP_UAVCAN
    // seems to be either a bug, or some inconsistent programming

    AP_HAL::Semaphore* _my_sem; //the frontend has a semaphore called sem, avoid confusion
};
