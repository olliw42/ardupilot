//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// this is the listener for the uavcan.EscStatus message
// it holds the latest status data
// it can be called form GCS_Common.cpp, MSG_ESC_TELEMETRY, to send out the status data per MAVLINK
// see AP_BLHeli class for reference



#pragma once


class BP_UavcanEscStatusManager {
    
public:
    BP_UavcanEscStatusManager();

    // Do not allow copies
    BP_UavcanEscStatusManager(const BP_UavcanEscStatusManager& other) = delete;
    BP_UavcanEscStatusManager& operator=(const BP_UavcanEscStatusManager&) = delete;

    // get singleton instance
    static BP_UavcanEscStatusManager* instance(void) { return _instance; }

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);
    
    // callback for UAVCAN messages
    void handle_escstatus_msg(uint16_t esc_index, float rpm,  float voltage, float current, float temperature);

private:
    static BP_UavcanEscStatusManager* _instance;

    struct escstatus_data {
        float rpm;
        float voltage;
        float current;
        float temperature;
        //private
        uint64_t timestamp;
        float consumed_mah;
        float consumed_wh;
    };
    struct escstatus_data _escstatus[8];
    uint16_t _escstatus_maxindex;

    AP_HAL::Semaphore *_sem;
};
