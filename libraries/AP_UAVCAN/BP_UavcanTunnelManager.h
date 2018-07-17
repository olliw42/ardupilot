#pragma once

#include "BP_Tunnel_Backend.h"


#define TUNNELMANAGER_NUM_CHANNELS   3 //forced to be identical to SERIALMANAGER_NUM_TUNNELPORTS in AP_SerialManager

#define TUNNELMANAGER_RXTIMEOUT_MS   10


class BP_UavcanTunnelManager {
    
public:
    BP_UavcanTunnelManager();

    // Do not allow copies
    BP_UavcanTunnelManager(const BP_UavcanTunnelManager& other) = delete;
    BP_UavcanTunnelManager& operator=(const BP_UavcanTunnelManager&) = delete;

    // get singleton instance
    static BP_UavcanTunnelManager* instance(void) { return _instance; }

    // is called in AP_SerialManager, to provide tunnel serials
    AP_HAL::UARTDriver* register_uart(uint8_t channel_id); //find a free channel_id and register this channel for an uart

    // is called in AP_UAVCAN tunnel message in-coming handler, and writes to the specified channel
    void write_to_channel(uint8_t channel_id, const uint8_t* buffer, size_t size);

    // is called periodically in vehicle class, similar to e.g. mount.update_fast()
    void update_fast(void);

private:
    static BP_UavcanTunnelManager* _instance;

    enum TunnelType {
        TunnelType_None = 0,
        TunnelType_UART = 1,
    };

    uint8_t _num_channels; // number of tunnels/channels instantiated
    
    struct tunnel_channel {
        uint8_t type;
        uint8_t channel_id;
        BP_Tunnel_Backend* backend;
        uint32_t last_received_ms;
    } _channel[TUNNELMANAGER_NUM_CHANNELS];
    

    struct tunnel_frame { //this also could be the real uavcan tunnel
        uint8_t protocol;
        uint8_t channel_id;
        uint8_t buffer[60];
        uint8_t buffer_len;
    } ;
    struct tunnel_frame _frame;

    void send_to_CAN(uint8_t tunnel_index, tunnel_frame* frameptr);
};

