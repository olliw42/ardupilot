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


//similar to destructors of Baro, GPS, etc.
// these IMHO CANNOT be correct and are quite buggy!!
AP_Airspeed_UAVCAN::~AP_Airspeed_UAVCAN()
{
    delete _my_sem;

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan != nullptr) {
            ap_uavcan->rawairdata_remove_listener(this);
        }
    }

    debug_as_uavcan(2, "AP_Airspeed_UAVCAN destructed\n\r");
}


bool AP_Airspeed_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return false;
    }
    
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        //there is a duplicate of the id list in each ap_uavcan/each driver
        // the first instance picks the smallest, the next instance the 2nd smallest, etc
        // the only issue will be that if there is a new node seen inbetween the init() calls, as this would give unpredictable
        // orderings, so, one should kind of freeze the addition of nodes when the first init() is called
        uint8_t id = ap_uavcan->rawairdata_find_smallest_nottaken_id();
        if (id == UINT8_MAX) {
            continue;
        }
        
        if (ap_uavcan->rawairdata_register_listener(this, id)) {
            debug_as_uavcan(2, "AP_Airspeed_UAVCAN registered id: %d\n\r", id);
            //_initialized = true; //don't set this here, wait for data to have arrived
            return true;
        }
    }

    return false;
}

//IMPORTANT:
//the node currently sends at a rate slightly slower than 10 Hz, thus this triggers every once in a while

// how fast is read() of the frontend called??
// airspeed.read() is called in void Plane::read_airspeed(void) in sensors.cpp
// SCHED_TASK(read_airspeed,          10,    100), in ArduPlane.cpp
// => is called at 10 Hz !!!

// => if we don't want decimation problems we need to send significantly faster!!!
//    and maybe do the filtering/averaging in here, similar to e.g. MS5525??
// this would be really sad, since we would have to flood the CAN bus with messages just for nothing
// it would be much better if there would be a way to synchronize the read() with the incoming data stream
// one should ask/figure out how sensitive the whole thing is to jitter/delay in the airspeed
// if it doesn't matter much, one just could go with twice the rate

//for comparison: GPS
// gps.update() called in
//    Plane::update_GPS_50Hz(void), SCHED_TASK(update_GPS_50Hz,        50,    300),
//    Copter::update_GPS(), SCHED_TASK(update_GPS,            50,    200),
// => here it is called much faster than data comes in, which avoids the rate issue

// this is how it should also be done for AirSpeed
// needs however significant changes to Airspeed class, and could seriously affect other classes such as EKF etc. pp => one needs to be careful!!
// this rate problem could also be true for other slowly sending UAVCAN sensors, Baro?

// => thus, for the moment the only way is to send airspeed faster, and discard

//the MS5525 class has a mechanism to collect more data and average it, should we do too?
// for some reason the averaging is protected by the frontend semaphore, why that?
// it also has a timeout mechanism, i.e., when there has been no new data for longer than 100ms, it returns false
// I think that's a bit dodgy in combination with the 10Hz update rate
//   it should be larger or smaller than 100ms, if it should be synchronized a bool flag should be used

//a false will set the sensor to unhealthy
bool AP_Airspeed_UAVCAN::get_differential_pressure(float& pressure)
{
    if (!_initialized) {
        return false;
    }

//    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
//        return false;
//    }

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

//    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
//        return false;
//    }

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
