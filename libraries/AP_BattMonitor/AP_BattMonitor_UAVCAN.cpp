#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_bm_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return;
    }

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        
        switch (_type) {
            case UAVCAN_BATTERY_INFO:
                if (ap_uavcan->register_BM_bi_listener_to_id(this, _params._serial_number)) {
                    debug_bm_uavcan(2, "UAVCAN BattMonitor BatteryInfo registered id: %d\n\r", _params._serial_number);
                }
                break;
//OW
            case UAVCAN_UC4HGENERICBATTERY_INFO:
                if (ap_uavcan->uc4hgenericbatteryinfo_register_listener(this, _params._serial_number)) {
                    debug_bm_uavcan(2, "UAVCAN BattMonitor Uc4hGenericBatteryInfo registered id: %d\n\r", _params._serial_number);
                }
                break;
            case UAVCAN_ESCSTATUS:
//XX                if (ap_uavcan->escstatus_register_listener(this, _params._serial_number)) {
//XX                    debug_bm_uavcan(2, "UAVCAN BattMonitor EscStatus registered id: %d\n\r", _params._serial_number);
//XX                }
                break;
//OWEND
        }
    }
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

void AP_BattMonitor_UAVCAN::handle_bi_msg(float voltage, float current, float temperature)
{
    _state.temperature = temperature;
    _state.voltage = voltage;
    _state.current_amps = current;

    uint32_t tnow = AP_HAL::micros();
    uint32_t dt = tnow - _state.last_time_micros;

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000) {
        // .0002778 is 1/3600 (conversion to hours)
        float mah = (float) ((double) _state.current_amps * (double) dt * (double) 0.0000002778f);
        _state.consumed_mah += mah;
        _state.consumed_wh  += 0.001f * mah * _state.voltage;
    }

    // record time
    _state.last_time_micros = tnow;

    _state.healthy = true;
}

//OW
void AP_BattMonitor_UAVCAN::handle_uc4hgenericbatteryinfo_msg(float voltage, float current, float charge, float energy)
{
    _state.voltage = voltage;
    _state.current_amps = current;
//    _state.current_total_mah = charge;
//    _state.energy_total_Wh = energy;

    // much of the following is not really needed for uc4h.genericbatteryinfo
    // but we want to be able to fallback whenever needed, and to avoid timeout

    uint32_t tnow = AP_HAL::micros();

    if (!uavcan::isNaN(charge) && !uavcan::isNaN(energy)) { //this should never happen, but play it safe
        _state.consumed_mah = charge;
        _state.consumed_wh = energy;
    } else {
        float dt = tnow - _state.last_time_micros;

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = (float) ((double) _state.current_amps * (double) dt * (double) 0.0000002778f);
            _state.consumed_mah += mah;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }
    }

    // record time
    _state.last_time_micros = tnow;

    _state.healthy = true;
}

void AP_BattMonitor_UAVCAN::handle_escstatus_msg(uint16_t esc_index, float voltage, float current)
{
    escstatus[esc_index].voltage = voltage;
    escstatus[esc_index].current = current;

    uint32_t tnow = AP_HAL::micros();
    uint32_t dt = tnow - escstatus[esc_index].time_micros;

    if (escstatus[esc_index].time_micros != 0 && dt < 2000000) {
        float mah = (float) ((double) escstatus[esc_index].current * (double) dt * (double) 0.0000002778f);
        escstatus[esc_index].consumed_mah += mah;
        escstatus[esc_index].consumed_wh  += 0.001f * mah * escstatus[esc_index].voltage;
    }

    escstatus[esc_index].time_micros = tnow;

    if( esc_index >= escstatus_maxindex ) escstatus_maxindex = esc_index + 1; //this is the number of motors, assuming that esc_index is continuous
}
//OWEND

#endif
