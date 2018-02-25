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


AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    //the type is now stored in _params.type()

    // starts with not healthy
    _state.healthy = false;
}


void AP_BattMonitor_UAVCAN::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    switch (_params.type()) {
//OW
                        case AP_BattMonitor_Params::BattMonitor_TYPE_UAVCAN_GenericBatteryInfo:
                            if (uavcan->genericbatteryinfo_register_listener(this, _params._serial_number)) {
                                debug_bm_uavcan(2, "UAVCAN BattMonitor[?] GenericBatteryInfo registered id: %d\n\r", _params._serial_number);
                            }
                            break;
//OWEND
                        default:
                            break;
                    }
                }
            }
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


//OW
void AP_BattMonitor_UAVCAN::handle_genericbatteryinfo_msg(float voltage, float current, float charge)
{
    _state.voltage = voltage;
    _state.current_amps = current;
//XX    _state.current_total_mah = charge;

    // much of the following is not really needed for genericbatteryinfo
    // but we want to be able to fallback to circuitstatus whenever needed, and to avoid timeout

    uint32_t tnow = AP_HAL::micros();

    if (!uavcan::isNaN(charge)) {
        _state.current_total_mah = charge;
    } else {
        float dt = tnow - _state.last_time_micros;

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            _state.current_total_mah += _state.current_amps * dt * 0.0000002778f;
        }
    }

    _state.last_time_micros = tnow;

    _state.healthy = true;
}
//OWEND

#endif
