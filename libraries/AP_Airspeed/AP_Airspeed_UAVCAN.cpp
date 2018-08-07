#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/AP_Common.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL &hal;

#define debug_as_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)


/// Constructor
AP_Airspeed_UAVCAN::AP_Airspeed_UAVCAN(AP_Airspeed& _frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    _initialized = false;
    _last_sample_time_ms = 0;

    _my_sem = hal.util->new_semaphore();
}


bool AP_Airspeed_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return false;
    }
    
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        const uint8_t id = 49; //TODO: this should be adjustable or auto adjusted !!!
        
        if (ap_uavcan->rawairdata_register_listener(this, id)) {
            debug_as_uavcan(2, "UAVCAN Airspeed registered id: %d\n\r", id);
            //_initialized = true; //don't set this here, wait for data to have arrived
            return true;
        }
    }
    
    return false;
}


//the MS5525 class has a mechanism to collect more data and average it, should we do too?
// it also has a timeout mechanism, i.e., when there has been no new data for longer than 100ms, it returns false, we this here too to be "compatible"
// for some reason the averaging is protected by the frontend semaphore, why that?

bool AP_Airspeed_UAVCAN::get_differential_pressure(float& pressure)
{
    if (!_initialized) {
        return false;
    }

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        pressure = _pressure;
        _my_sem->give();
        return true;
    }

    return false;
}


bool AP_Airspeed_UAVCAN::get_temperature(float& temperature)
{
    if (!_initialized) {
        return false;
    }

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        temperature = _temperature;
        _my_sem->give();
        return true;
    }

    return false;
}


void AP_Airspeed_UAVCAN::handle_rawairdata_msg(float differential_pressure, float differential_pressure_sensor_temperature)
{
    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        //this indicates that valid data has arrived
        // this only can happen if AP_Airspeed_UAVCAN::init() was successful, so no need for an additional flag
        _initialized = true;

        _pressure = differential_pressure; //UAVCAN is in Pascal, no conversion needed
        _temperature = differential_pressure_sensor_temperature - 273.15f; //UAVCAN is in Kelvin, convert to Celsius

        _last_sample_time_ms = AP_HAL::millis();

        _my_sem->give();
    }
}

#endif // HAL_WITH_UAVCAN
