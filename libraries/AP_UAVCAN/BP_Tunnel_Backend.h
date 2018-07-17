#pragma once


class BP_Tunnel_Backend {

public:
    // Constructor
    BP_Tunnel_Backend() {}

    //general API, must be supported by all childs
    virtual void write_to_rx(const uint8_t *buffer, size_t size) = 0;

    virtual uint32_t tx_num_available() = 0; //the UartDRIVER API should/could have a tx_available(), so chose a different name here

    virtual void read_from_tx(uint8_t *buffer, size_t size) = 0;

    //uart specific API, should be overridden by uart childs
    virtual uint32_t uart_baudrate(void) { return 0; }
    virtual bool uart_baurate_has_changed(void) { return false; }

protected:
};
