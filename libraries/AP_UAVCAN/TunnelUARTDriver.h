//OW
// follows very closely the implementation by MagicRub
// https://github.com/magicrub/ardupilot/commit/e223add8c4b18e08a127d40268a1cb986bcb1f3d
// THX!

#pragma once

#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "BP_Tunnel_Backend.h"


#define TUNNELUARTDRIVER_RXBUF_SIZE_MIN            (512*4)
#define TUNNELUARTDRIVER_TXBUF_SIZE_MIN            (512*4)


class TunnelUARTDriver : public AP_HAL::UARTDriver, public BP_Tunnel_Backend {

public:
    TunnelUARTDriver();

    // Empty implementations of UARTDriver virtual methods
    void begin(uint32_t baud) override { begin(baud, 0, 0); }
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void end() override;
    void flush() override {}
    bool is_initialized() override { return _initialised; }
    void set_blocking_writes(bool blocking) override { (void)blocking; }
    bool tx_pending() override;

    // Empty implementations of BetterStream virtual methods
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    uint32_t available() override;
    int16_t read() override; // return value for read(): -1 if nothing available, uint8_t value otherwise.
    uint32_t txspace() override;

    // Empty implementations of BP_Tunnel_Backend virtual methods
    void write_to_rx(const uint8_t *buffer, size_t size) override;
    uint32_t tx_num_available(void) override;
    void read_from_tx(uint8_t *buffer, size_t size) override;
    
    // override default implementations of virtual methods
    uint32_t uart_baudrate(void) override { return _baud; } //{ return (_initialised) ? _baud : 0; }
    bool uart_baudrate_has_changed(void) override { if (_baud_changed) {_baud_changed = false; return _initialised;} else {return false;} }

private:
    bool _initialised;

    uint32_t _baud;
    bool _baud_changed;

    AP_HAL::Semaphore *_write_mutex;
    AP_HAL::Semaphore *_read_mutex;

    ByteBuffer _read_buf{0};
    ByteBuffer _write_buf{0};
};
