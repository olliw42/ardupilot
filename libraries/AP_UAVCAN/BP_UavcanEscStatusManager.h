//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// this is the listener for the uavcan.EscStatus message
// it holds the latest status data
// it is called from GCS_Common.cpp, MSG_ESC_TELEMETRY, to send out the status data per MAVLINK
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
    
    // is called in AP_UAVCAN tunnel message in-coming handler, and writes to the specified esc_index
    void write_to_escindex(uint16_t esc_index,
            uint32_t error_count, float voltage, float current, float temperature,
            int32_t rpm, uint8_t power_rating_pct);

private:
    static BP_UavcanEscStatusManager* _instance;

    struct escstatus_data {
        uint32_t error_count;
        float voltage;
        float current;
        float temperature;
        int32_t rpm;
        uint8_t power_rating_pct;
        //private
        uint32_t timestamp_us;
        float consumed_mah;
        uint32_t rx_count;
    };
    struct escstatus_data _escstatus[12];
    uint16_t _esc_maxindex;
};
