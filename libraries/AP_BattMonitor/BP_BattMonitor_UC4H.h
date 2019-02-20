#pragma once

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class BP_BattMonitor_UC4H : public AP_BattMonitor_Backend
{
public:

    enum BattMonitor_UC4H_Type {
        UC4H_UC4HGENERICBATTERYINFO = 83,
        UC4H_ESCSTATUS = 84,
    };

    /// Constructor
    BP_BattMonitor_UC4H(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params, BattMonitor_UC4H_Type type);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    void init() override;

    bool has_current() const override {
        return true;
    }

    void handle_uc4hgenericbatteryinfo_msg(uint32_t ext_id, float voltage, float current, float charge, float energy, uint16_t cells_num, float* cells);
    void handle_escstatus_msg(uint16_t esc_index, float voltage, float current) override;

    //see AP_BattMonitorSMBus_Maxell as example for cell handling
    bool has_cell_voltages() const override { return _has_cell_voltages; }

protected:
    BattMonitor_UC4H_Type _type;

    bool _has_cell_voltages; //backends flag this as true once they have received a valid cell voltage report

    struct escstatus_data { //this is as received from uavcan.equipment.esc.Status
        float voltage;
        float current;
        //private
        uint32_t time_micros;
        float consumed_charge_mah;
        float consumed_energy_wh;
        bool healthy;
        bool detected;
    };
    struct escstatus_data _escstatus[8];
    uint16_t _escstatus_maxindex;

private:
    uint32_t _own_id;
};
