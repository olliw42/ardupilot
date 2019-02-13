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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_PX4_PWM.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <uORB/topics/pwm_input.h>
#include <stdio.h>
#include <errno.h>
#include <cmath>

extern const AP_HAL::HAL& hal;

extern "C" {
    int pwm_input_main(int, char **);
};

//OW
const AP_Param::GroupInfo AP_RangeFinder_PX4_PWM::var_info[] = {
    // @Param: SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("SCALING", 3, AP_RangeFinder_PX4_PWM, pScaling, 3.0f),

    // @Param: OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("OFFSET",  4, AP_RangeFinder_PX4_PWM, pOffset, 0.0f),

    // @Param: STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("STOP_PIN", 8, AP_RangeFinder_PX4_PWM, pStop_pin, -1),

    // @Param: SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SETTLE", 9, AP_RangeFinder_PX4_PWM, pSettle_time_ms, 0),

    // @Param: PWRRNG
    // @DisplayName: Powersave range
    // @Description: This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
    // @Units: m
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("PWRRNG", 11, AP_RangeFinder_PX4_PWM, pPowersave_range, 0),

    AP_GROUPEND
};
//OWEND

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4_PWM::AP_RangeFinder_PX4_PWM(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, float &_estimated_terrain_height) :
    AP_RangeFinder_Backend(_state, _params),
    estimated_terrain_height(_estimated_terrain_height)
{
//OW
    AP_Param::setup_object_defaults(this, var_info);

    // register PX4_PWM specific parameters, should maybe come at the end if one follows WASP, but why not here, the user wants to see them
    state.var_info = var_info;
//OWEND
    _fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        hal.console->printf("Unable to open PX4 PWM rangefinder\n");
        set_status(RangeFinder::RangeFinder_NotConnected);
        return;
    }

    // keep a queue of 20 samples
    if (ioctl(_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
        hal.console->printf("Failed to setup range finder queue\n");
        set_status(RangeFinder::RangeFinder_NotConnected);
        return;
    }

    // initialise to connected but no data
    set_status(RangeFinder::RangeFinder_NoData);
}

/* 
   close the file descriptor
*/
AP_RangeFinder_PX4_PWM::~AP_RangeFinder_PX4_PWM()
{
    if (_fd != -1) {
        close(_fd);
    }
    set_status(RangeFinder::RangeFinder_NotConnected);
}

/* 
   see if the PX4 driver is available
*/
bool AP_RangeFinder_PX4_PWM::detect()
{
#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V1) && \
    !defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    if (AP_BoardConfig::px4_start_driver(pwm_input_main, "pwm_input", "start")) {
        hal.console->printf("started pwm_input driver\n");
    }
#endif
    int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}

void AP_RangeFinder_PX4_PWM::update(void)
{
    if (_fd == -1) {
        set_status(RangeFinder::RangeFinder_NotConnected);
        return;
    }

    struct pwm_input_s pwm;
    float sum_cm = 0;
    uint16_t count = 0;
    const float scaling = pScaling; //OW //OWEND
    uint32_t now = AP_HAL::millis();

    while (::read(_fd, &pwm, sizeof(pwm)) == sizeof(pwm)) {
        // report the voltage as the pulse width, so we get the raw
        // pulse widths in the log
        state.voltage_mv = pwm.pulse_width;

        _last_pulse_time_ms = now;

        // setup for scaling in meters per millisecond
        float _distance_cm = pwm.pulse_width * 0.1f * scaling + pOffset; //OW //OWEND

        float distance_delta_cm = fabsf(_distance_cm - _last_sample_distance_cm);
        _last_sample_distance_cm = _distance_cm;

        if (distance_delta_cm > 100) {
            // varying by more than 1m in a single sample, which means
            // between 50 and 100m/s vertically - discard
            _good_sample_count = 0;
            continue;
        }

        if (_good_sample_count > 1) {
            count++;
            sum_cm += _distance_cm;
            _last_timestamp = pwm.timestamp;
        } else {
            _good_sample_count++;
        }
    }

    // if we haven't received a pulse for 1 second then we may need to
    // reset the timer
    int8_t stop_pin = pStop_pin; //OW //OWEND
    uint16_t settle_time_ms = (uint16_t)pSettle_time_ms; //OW //OWEND

    if (stop_pin != -1 && out_of_range()) {
        // we are above the power saving range. Disable the sensor
        hal.gpio->pinMode(stop_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(stop_pin, false);
        set_status(RangeFinder::RangeFinder_NoData);
        state.distance_cm = 0;
        state.voltage_mv = 0;
        return;
    }

    // if we have not taken a reading in the last 0.2s set status to No Data
    if (AP_HAL::micros64() - _last_timestamp >= 200000) {
        set_status(RangeFinder::RangeFinder_NoData);
    }

    /* if we haven't seen any pulses for 0.5s then the sensor is
       probably dead. Try resetting it. Tests show the sensor takes
       about 0.2s to boot, so 500ms offers some safety margin
    */
    if (now - _last_pulse_time_ms > 500U && _disable_time_ms == 0) {
        ioctl(_fd, SENSORIOCRESET, 0);
        _last_pulse_time_ms = now;

        // if a stop pin is configured then disable the sensor for the
        // settle time
        if (stop_pin != -1) {
            hal.gpio->pinMode(stop_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(stop_pin, false);            
            _disable_time_ms = now;
        }
    }

    /* the user can configure a settle time. This controls how 
       long the sensor is disabled for using the stop pin when it is
       reset. This is used both to make sure the sensor is properly
       reset, and also to allow for power management by running a low
       duty cycle when it has no signal 
    */
    if (stop_pin != -1 && _disable_time_ms != 0 && 
        (now - _disable_time_ms > settle_time_ms)) {
        hal.gpio->write(stop_pin, true);        
        _disable_time_ms = 0;
        _last_pulse_time_ms = now;
    }

    if (count != 0) {
        state.distance_cm = sum_cm / count;

        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    }
}

#endif // CONFIG_HAL_BOARD
