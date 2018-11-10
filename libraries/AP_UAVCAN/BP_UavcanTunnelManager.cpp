//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#include <AP_HAL/AP_HAL.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include "BP_UavcanTunnelManager.h"
#include "TunnelUARTDriver.h"

extern const AP_HAL::HAL& hal;


// singleton instance
BP_UavcanTunnelManager* BP_UavcanTunnelManager::_instance;


// Constructor
BP_UavcanTunnelManager::BP_UavcanTunnelManager()
{
    if (_instance != nullptr) {
        AP_HAL::panic("BP_UavcanTunnelManager must be singleton");
    }
    _instance = this;

    _num_channels = 0;
    
    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {
        _channel[i].backend = nullptr; //this is used to indicate a free/unused slot!! we don't use type, as it makes it easier
    }
}


AP_HAL::UARTDriver* BP_UavcanTunnelManager::register_uart(uint8_t channel_id)
{
    //check if channel_id is already in use
    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {
        if ((_channel[i].backend != nullptr) && (_channel[i].channel_id == channel_id)) { //slot is in use and has same channel_id
            return nullptr;
        }
    }

    //find empty place and register
    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {
        if (_channel[i].backend == nullptr) { //no channel registered in this slot, a free place has been found

            TunnelUARTDriver* uart = new TunnelUARTDriver(); //created on heap, long lived

            if (uart == nullptr) return nullptr; //something went wrong
            
            _channel[i].channel_id = channel_id;
            _channel[i].last_received_ms = AP_HAL::millis();

            _channel[i].backend = uart;

           _num_channels++;
            return uart;
        }
    }
    
    return nullptr; //no free channel found
}


// this is called in AP_UAVCAN tunnel message in
// it writes the received buffer to the specified channel
void BP_UavcanTunnelManager::write_to_channel(uint8_t channel_id, const uint8_t* buffer, size_t size)
{
    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {

        if (_channel[i].backend == nullptr) continue; //no channel registered in this slot

        if (_channel[i].channel_id == channel_id) {
            _channel[i].backend->write_to_rx(buffer, size);
            return; //channel found, no need to continue searching
        }
    }
}


// this calls read_from_tx() of each backend, reads in data and puts it into a frame
// it thereby respects the timeout and max buffer size
// give every channel a slot and chance!

//this can lose frames if the CANbus is too busy to send out the max TUNNELMANAGER_NUM_CHANNELS frames within 2.5 ms
// the current approach is quite robust, but probably isn't 100% foolproof

//one frame per update is ca 400 Hz x 60 bytes = 24000 bytes/s
// a CAN frame is 131 bits max => 7633 frames/s max = ca 61 kbytes/s max
// => this consumes already 39% of the available bandwidth !!!
//115200 bps are 11.5 kbytes/s => consumes 19% at full rate
//ESC for quad is 400 Hz x 1 frame

void BP_UavcanTunnelManager::update_fast(void)
{
    //crude approach: if only one frame has not yet been send, postpone all. THIS WORKS
    // I tested, the finer method works too with 'p'

    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {

        if (_channel[i].backend == nullptr) continue; //no channel registered in this slot

        uint32_t now_ms = AP_HAL::millis();

        if (is_to_send(i)) {
            _channel[i].last_received_ms = now_ms;
            continue;
        }

        if (_channel[i].backend->uart_baudrate_has_changed()) {
            _frame.protocol = 255;
            _frame.channel_id = _channel[i].channel_id;
            *((uint32_t*)_frame.buffer) = _channel[i].backend->uart_baudrate();
            _frame.buffer_len = 4;
            send_to_CAN(i, &_frame);
            continue;
        }

        uint32_t available = _channel[i].backend->tx_num_available();

        if (!available) {
            //nothing received
            _channel[i].last_received_ms = now_ms; //clear it, so that the timeout starts with a first received char after a blank period
        } else {
            if ((available >= UAVCAN_TUNNELBROADCAST_BUFFER_MAX) || ((now_ms - _channel[i].last_received_ms) >= TUNNELMANAGER_RXTIMEOUT_MS)) {

                if (available > UAVCAN_TUNNELBROADCAST_BUFFER_MAX) available = UAVCAN_TUNNELBROADCAST_BUFFER_MAX; //limit to 60 chars max

                _frame.protocol = 254;
                _frame.channel_id = _channel[i].channel_id;
                _channel[i].backend->read_from_tx(_frame.buffer, available);
                _frame.buffer_len = available;

                _channel[i].last_received_ms = now_ms;

                send_to_CAN(i, &_frame);
            }
        }
    }
}


void BP_UavcanTunnelManager::send_to_CAN(uint8_t tunnel_index, tunnel_frame* frameptr)
{
    for (uint8_t ci = 0; ci < MAX_NUMBER_OF_CAN_DRIVERS; ci++) {
        AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(ci);
        if (ap_uavcan != nullptr) {
            ap_uavcan->tunnelbroadcast_send(tunnel_index, frameptr->protocol, frameptr->channel_id, frameptr->buffer, frameptr->buffer_len, 0);
        }
    }
}


bool BP_UavcanTunnelManager::is_to_send(uint8_t tunnel_index)
{
    for (uint8_t ci = 0; ci < MAX_NUMBER_OF_CAN_DRIVERS; ci++) {
        AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(ci);
        if (ap_uavcan != nullptr) {
            if (ap_uavcan->tunnelbroadcast_is_to_send(tunnel_index)) return true; //one of the channels is not yet done
        }
    }
    return false;
}

