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

/*
 *   AP_RangeFinder_analog.cpp - rangefinder for analog source
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "RangeFinder.h"
#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder_analog.h"

extern const AP_HAL::HAL& hal;

//OW
const AP_Param::GroupInfo AP_RangeFinder_analog::var_info[] = {
    // @Param: PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 11:PX4-airspeed port, 15:Pixhawk-airspeed port
    // @User: Standard
    AP_GROUPINFO("PIN",     2, AP_RangeFinder_analog, pPin, -1),

    // @Param: SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("SCALING", 3, AP_RangeFinder_analog, pScaling, 3.0f),

    // @Param: OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("OFFSET",  4, AP_RangeFinder_analog, pOffset, 0.0f),

    // @Param: FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("FUNCTION", 5, AP_RangeFinder_analog, pFunction, 0),

    // @Param: STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("STOP_PIN", 8, AP_RangeFinder_analog, pStop_pin, -1),

    // @Param: SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SETTLE", 9, AP_RangeFinder_analog, pSettle_time_ms, 0),

    // @Param: RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("RMETRIC", 10, AP_RangeFinder_analog, pRatiometric, 1),

    AP_GROUPEND
};
//OWEND

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_analog::AP_RangeFinder_analog(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
//OW
    AP_Param::setup_object_defaults(this, var_info);

    // register analog backend specific parameters, do this before source is checked
    state.var_info = var_info;
//OWEND
    source = (pPin < 0) ? nullptr : hal.analogin->channel(pPin); //OW //OWEND //since pin might be -1 now, see comment below, catch this possibility
    if (source == nullptr) {
        // failed to allocate a ADC channel? This shouldn't happen
        set_status(RangeFinder::RangeFinder_NotConnected);
        return;
    }
    source->set_stop_pin((uint8_t)pStop_pin); //OW //OWEND
    source->set_settle_time((uint16_t)pSettle_time_ms); //OW //OWEND
    set_status(RangeFinder::RangeFinder_NoData);
}

/* 
   detect if an analog rangefinder is connected. The only thing we
   can do is check if the pin number is valid. If it is, then assume
   that the device is connected
*/
bool AP_RangeFinder_analog::detect(AP_RangeFinder_Params &_params)
{
//OW
    // having moved pin to the backend, it isn't available anymore here, so just ignore it and always return with true. This is not a big practical issue.
    return true;

//    if (_params.pin != -1) {
//        return true;
//    }
//    return false;
//OWEND
}


/*
  update raw voltage state
 */
void AP_RangeFinder_analog::update_voltage(void)
{
   if (source == nullptr) {
       state.voltage_mv = 0;
       return;
   }
   // cope with changed settings
   source->set_pin(pPin); //OW //OWEND
   source->set_stop_pin((uint8_t)pStop_pin); //OW //OWEND
   source->set_settle_time((uint16_t)pSettle_time_ms); //OW //OWEND
   if (pRatiometric) { //OW //OWEND
       state.voltage_mv = source->voltage_average_ratiometric() * 1000U;
   } else {
       state.voltage_mv = source->voltage_average() * 1000U;
   }
}

/*
  update distance_cm 
 */
void AP_RangeFinder_analog::update(void)
{
    update_voltage();
    float v = state.voltage_mv * 0.001f;
    float dist_m = 0;
    float scaling = pScaling; //OW //OWEND
    float offset  = pOffset; //OW //OWEND
    RangeFinder::RangeFinder_Function function = (RangeFinder::RangeFinder_Function)pFunction.get(); //OW //OWEND
    int16_t _max_distance_cm = params.max_distance_cm;

    switch (function) {
    case RangeFinder::FUNCTION_LINEAR:
        dist_m = (v - offset) * scaling;
        break;
	  
    case RangeFinder::FUNCTION_INVERTED:
        dist_m = (offset - v) * scaling;
        break;

    case RangeFinder::FUNCTION_HYPERBOLA:
        if (v <= offset) {
            dist_m = 0;
        } else {
            dist_m = scaling / (v - offset);
        }
        if (isinf(dist_m) || dist_m > _max_distance_cm * 0.01f) {
            dist_m = _max_distance_cm * 0.01f;
        }
        break;
    }
    if (dist_m < 0) {
        dist_m = 0;
    }
    state.distance_cm = dist_m * 100.0f;
    state.last_reading_ms = AP_HAL::millis();

    // update range_valid state based on distance measured
    update_status();
}

