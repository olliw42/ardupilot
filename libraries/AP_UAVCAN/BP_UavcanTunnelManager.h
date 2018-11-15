//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// This class allows handling several virtual serial data streams, and link them to UAVCAN tunnel.Broadcast messages with respective channel_id.
// The data stream can be "anything", and is abstracted by the BP_Tunnel_Backend class. Typically it would be a virtual serial port, but could be I2C.
// The function update_fast() needs to be periodically called at a relatively high rate, e.g. 400 Hz.
// Currently, this function is hooked into the Copter vehicle loop, like the camera_mount.update_fast(), but better places might be found.

#pragma once

#include "BP_Tunnel_Backend.h"


#define TUNNELMANAGER_NUM_CHANNELS   3 //forced to be identical to SERIALMANAGER_NUM_TUNNELPORTS in AP_SerialManager

#define TUNNELMANAGER_RXTIMEOUT_MS   10

#define UAVCAN_TUNNELBROADCAST_BUFFER_MAX   60

//this is for data which is emitted via the tunnel message on the CAN bus
// the size is such that the data fits perfectly into a multiframe message, in order to optimize bandwidth
// it can be calculate such:
// 1st frame:        2xcrc 1xprotocol 1xchannel_id 3xdata 1xtailbyte
// following frames: 7xdata 1xtailbyte
// => 3 + (n-1)*7 = 3, 10, 17, 24, 31, 38, 45, 52, 59, 66, ...
//thus, 60 bytes just fails to fit by one byte, hence use 59
#define UAVCAN_TUNNELBROADCAST_TUNNELOUT_MAX   59


//BP_UavcanTunnelManager tunnel_manager; is called in Copter.h


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

    uint8_t _num_channels; // number of tunnels/channels instantiated
    
    struct tunnel_channel {
        uint8_t channel_id;
        BP_Tunnel_Backend* backend;
        uint32_t last_received_ms;
    } _channel[TUNNELMANAGER_NUM_CHANNELS];
    

    struct tunnel_frame { //this also could be the real uavcan tunnel
        uint8_t protocol;
        uint8_t channel_id;
        uint8_t buffer[UAVCAN_TUNNELBROADCAST_BUFFER_MAX];
        uint8_t buffer_len;
    } ;
    struct tunnel_frame _frame;

    bool is_to_send(uint8_t tunnel_index);
    void send_to_CAN(uint8_t tunnel_index, tunnel_frame* frameptr);
};

