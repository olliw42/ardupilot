//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#include <AP_HAL/AP_HAL.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "BP_UavcanEscStatusManager.h"

extern const AP_HAL::HAL& hal;


// singleton instance
BP_UavcanEscStatusManager* BP_UavcanEscStatusManager::_instance;


// Constructor
BP_UavcanEscStatusManager::BP_UavcanEscStatusManager()
{
    if (_instance != nullptr) {
        AP_HAL::panic("BP_UavcanEscStatusManager must be singleton");
    }
    _instance = this;
}


void BP_UavcanEscStatusManager::write_to_escindex(uint16_t esc_index,
        uint32_t error_count, float voltage, float current, float temperature,
        int32_t rpm, uint8_t power_rating_pct)
{
    if (esc_index >= 8) return;

    _escstatus[esc_index].rpm = rpm;
    _escstatus[esc_index].voltage = voltage;
    _escstatus[esc_index].current = current;
    _escstatus[esc_index].temperature = temperature;

    _escstatus[esc_index].timestamp = AP_HAL::micros64();

    if (esc_index >= _escstatus_maxindex) _escstatus_maxindex = esc_index + 1; //this is the number of motors, assuming that esc_index is continuous
}


void BP_UavcanEscStatusManager::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
/*    
    if (num_motors == 0) {
        return;
    }
    uint8_t temperature[4] {};
    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    uint32_t now = AP_HAL::millis();
    for (uint8_t i=0; i<num_motors; i++) {
        uint8_t idx = i % 4;
        if (last_telem[i].timestamp_ms && (now - last_telem[i].timestamp_ms < 1000)) {
            temperature[idx]  = last_telem[i].temperature;
            voltage[idx]      = last_telem[i].voltage;
            current[idx]      = last_telem[i].current;
            totalcurrent[idx] = last_telem[i].consumption;
            rpm[idx]          = last_telem[i].rpm;
            count[idx]        = last_telem[i].count;
        } else {
            temperature[idx] = 0;
            voltage[idx] = 0;
            current[idx] = 0;
            totalcurrent[idx] = 0;
            rpm[idx] = 0;
            count[idx] = 0;
        }
        if (i % 4 == 3 || i == num_motors - 1) {
            if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
                return;
            }
            if (i < 4) {
                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
            } else {
                mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
            }
        }
    }
*/    
}

