//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// This class provides an abstract virtual serial data stream for the BP_UavcanTunnelManager class.
// It can be "anything", such as a virtual serial port or I2C.
// Currently, it is the base for the TunnelUARTDriver class, to bridge between a virtual UART and a UAVCAN tunnel.

#pragma once


class BP_Tunnel_Backend {

public:
    // Constructor
    BP_Tunnel_Backend() {}

    //general API, must be supported by all childs
    virtual void write_to_rx(const uint8_t* buffer, size_t size) = 0;

    virtual uint32_t tx_num_available() = 0; //the UartDRIVER API should/could have a tx_available(), so chose a different name here

    virtual void read_from_tx(uint8_t* buffer, size_t size) = 0;

    //to keep track of the type, can be 'anything'
    enum TunnelType : uint8_t {
        TunnelType_Unspecified = 0,
        TunnelType_UART = 1,
    } type;

    //uart specific API, should be overridden by uart childs
    virtual uint32_t uart_baudrate(void) { return 0; }
    virtual bool uart_baudrate_has_changed(void) { return false; }
};
