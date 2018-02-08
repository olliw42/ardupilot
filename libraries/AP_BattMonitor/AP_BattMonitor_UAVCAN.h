#pragma once

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    void init() override;

    bool has_current() const override {
        return true;
    }

//OW
    void handle_genericbatteryinfo_msg(float voltage, float current, float charge) override;
//OWEND
};
