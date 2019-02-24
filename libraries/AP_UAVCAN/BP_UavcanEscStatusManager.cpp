//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#include <AP_HAL/AP_HAL.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "BP_UavcanEscStatusManager.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


// singleton instance
BP_UavcanEscStatusManager* BP_UavcanEscStatusManager::_singleton;


// Constructor
BP_UavcanEscStatusManager::BP_UavcanEscStatusManager()
{
    if (_singleton != nullptr) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"%s must be singleton", __FUNCTION__);
    }
    _singleton = this;
}


void BP_UavcanEscStatusManager::write_to_escindex(uint16_t esc_index,
        uint32_t error_count, float voltage, float current, float temperature,
        int32_t rpm, uint8_t power_rating_pct)
{
    if (esc_index >= 12) return;

    uint64_t now64_us = AP_HAL::micros64();
    uint32_t now_us = now64_us; //AP_HAL::micros();

    //calculate the consumed charge in mAh and energy in Wh, for later use by whoever might be interested
    uint32_t dt = now_us - _esc_status[esc_index].timestamp_us; //this is the time of the previously received msg
    if ((_esc_status[esc_index].timestamp_us != 0) && (dt < 2000000)) {
        float mah = (float) ((double) current * (double) dt * (double) 0.0000002778f);
        _esc_status[esc_index].consumed_charge_mah += mah;
        _esc_status[esc_index].consumed_energy_wh  += 0.001f * mah * voltage;
    }

    //calculate the temperature in C°, for later use by whoever might be interested
    //TODO: use is_equal(), but check if it does exactly what is the intention here
//??    _esc_status[esc_index].temperature_degC = (!uavcan::isNaN(temperature) && (!temperature != 0.0f)) ? temperature - C_TO_KELVIN : 0.0f;
    _esc_status[esc_index].temperature_degC = (!uavcan::isNaN(temperature) && !is_equal(temperature,0.0f)) ? temperature - C_TO_KELVIN : 0.0f;

    _esc_status[esc_index].rpm = rpm;
    _esc_status[esc_index].voltage = voltage;
    _esc_status[esc_index].current = current;
    _esc_status[esc_index].temperature = temperature;

    _esc_status[esc_index].timestamp_us = now_us;
    _esc_status[esc_index].timestamp64_us = now64_us;
    _esc_status[esc_index].rx_count++;

    if (esc_index >= _esc_maxindex) _esc_maxindex = esc_index + 1; //this is the number of motors, assuming that esc_index is continuous
}


void BP_UavcanEscStatusManager::log_to_dataflash(uint16_t esc_index)
{
    if (esc_index >= 8) return; //DataFlash supports only up to 8 ESCs

    DataFlash_Class* df = DataFlash_Class::instance();
    if (df && df->logging_enabled()) {
        struct log_Esc pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ESC1_MSG + esc_index)),
            time_us     : _esc_status[esc_index].timestamp64_us,
            rpm         : (int32_t)(_esc_status[esc_index].rpm * 100), //see comment in LogStructure.h !!!
            voltage     : (uint16_t)(_esc_status[esc_index].voltage * 100.0f + 0.5f),
            current     : (uint16_t)(_esc_status[esc_index].current * 100.0f + 0.5f),
            temperature : (int16_t)(_esc_status[esc_index].temperature_degC * 100.0f + 0.5f),
            current_tot : (uint16_t)(_esc_status[esc_index].consumed_charge_mah + 0.5f)
        };
        df->WriteBlock(&pkt, sizeof(pkt));
    }
}


//uint16_t voltage[4]; /*< [cV] Voltage*/
//uint16_t current[4]; /*< [cA] Current*/
//uint16_t totalcurrent[4]; /*< [mAh] Total current*/
//uint16_t rpm[4]; /*< [rpm] RPM (eRPM)*/
//uint16_t count[4]; /*<  count of telemetry packets received (wraps at 65535)*/
//uint8_t temperature[4]; /*< [degC] Temperature*/
//taken from AP_BLHeli as reference
void BP_UavcanEscStatusManager::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    if (_esc_maxindex == 0) { //nothing ever received
        return;
    }

    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t rpm[4] {};
    uint16_t count[4] {};
    uint8_t temperature[4] {};

    uint32_t now_us = AP_HAL::micros();

    if (_esc_maxindex >= 12) _esc_maxindex = 12;

    for (uint8_t i=0; i<_esc_maxindex; i++) {
        uint8_t idx = i % 4;

        if (_esc_status[i].timestamp_us && ((now_us - _esc_status[i].timestamp_us) < 1000000)) {
            voltage[idx]      = (uint16_t)(_esc_status[i].voltage*100.0f + 0.5f);
            current[idx]      = (uint16_t)(_esc_status[i].current*100.0f + 0.5f);
            totalcurrent[idx] = _esc_status[i].consumed_charge_mah;
            rpm[idx]          = _esc_status[i].rpm;
            count[idx]        = _esc_status[i].rx_count;

            float temp_degC = _esc_status[i].temperature_degC;
            if (temp_degC < 0.0f) temp_degC = 0.0f;
            temperature[idx]  = (int8_t)(temp_degC + 0.5f);

        } else {
            temperature[idx] = voltage[idx] = current[idx] = totalcurrent[idx] = rpm[idx] = count[idx] = 0;
        }
        if ((i % 4 == 3) || (i == _esc_maxindex - 1)) {
            if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
                return;
            }
            if (i < 4) {
                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
            } else
            if (i < 8) {
                mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
            } else {
                mavlink_msg_esc_telemetry_9_to_12_send((mavlink_channel_t)mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
            }
        }
    }
}

