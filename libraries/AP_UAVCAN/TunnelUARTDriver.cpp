//OW
// follows very closely the implementation by MagicRub
// https://github.com/magicrub/ardupilot/commit/e223add8c4b18e08a127d40268a1cb986bcb1f3d
// THX!

#include <AP_HAL/AP_HAL.h>
#include "TunnelUARTDriver.h"


extern const AP_HAL::HAL& hal;


// Constructor
TunnelUARTDriver::TunnelUARTDriver()
{
    _write_mutex = hal.util->new_semaphore();
    _read_mutex = hal.util->new_semaphore();

    _initialised = false;
    _baud = 0;
    _baud_changed = false;
}


//------------------------------------------------------
// Empty implementations of UARTDriver virtual methods
//------------------------------------------------------

// open virtual port
void TunnelUARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    _initialised = false;

    _baud = baud;
    _baud_changed = true;

    // enforce minimum sizes
    if (rxSpace < TUNNELUARTDRIVER_RXBUF_SIZE_MIN) rxSpace = TUNNELUARTDRIVER_RXBUF_SIZE_MIN;
    if (txSpace < TUNNELUARTDRIVER_TXBUF_SIZE_MIN) txSpace = TUNNELUARTDRIVER_TXBUF_SIZE_MIN;

    if (_read_buf.set_size(rxSpace) && _write_buf.set_size(txSpace)) {
        _initialised = true;
    }
}


// shutdown a Virtual UART
void TunnelUARTDriver::end()
{
    _initialised = false;
    _read_buf.set_size(0);
    _write_buf.set_size(0);
}


// do we have any bytes pending transmission?
bool TunnelUARTDriver::tx_pending()
{
    return (_write_buf.available() > 0);
}


//------------------------------------------------------
// Empty implementations of BetterStream virtual methods
//------------------------------------------------------

// implementation of write virtual method
size_t TunnelUARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }

    if (!_write_mutex->take(2)) {
        return 0;
    }

    uint32_t len = _write_buf.write(&c, 1);

    _write_mutex->give();
    return len;
}


// write size bytes to the write buffer
size_t TunnelUARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }

    if (!_write_mutex->take(2)) {
        return 0;
    }

    uint32_t len = _write_buf.write(buffer, size);

    _write_mutex->give();
    return len;
}


// return the number of bytes available to be read
uint32_t TunnelUARTDriver::available()
{
    return _read_buf.available();
}


// implementation of read virtual method
int16_t TunnelUARTDriver::read()
{
    if (!_initialised) {
        return -1;
    }

    if ( !_read_mutex->take(2)) {
        return -1;
    }

    uint8_t c;
    if (!_read_buf.read_byte(&c)) {
        _read_mutex->give();
        return -1;
    }

    _read_mutex->give();
    return c;
}


// how many bytes can be placed at maximum in the output buffer?
uint32_t TunnelUARTDriver::txspace()
{
    return _write_buf.space();
}


//------------------------------------------------------
// Empty implementations of BP_Tunnel_Backend virtual methods
//------------------------------------------------------

void TunnelUARTDriver::write_to_rx(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return;
    }

    if (!_read_mutex->take(2)) {
        return;
    }

    _read_buf.write(buffer, size);

    _read_mutex->give();
}


uint32_t TunnelUARTDriver::tx_num_available(void)
{
    return _write_buf.available();
}


//we assume that it was tested before that there are indeed size bytes available for reading!!!
void TunnelUARTDriver::read_from_tx(uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return;
    }

    if (!_write_mutex->take(2)) {
        return;
    }

    uint8_t c;
    for (size_t n=0; n<size; n++) {
        if (!_write_buf.read_byte(&c)) c = 0; //if something went wrong, fill it up
        if (buffer != nullptr) buffer[n] = c; //this allows to read and discard
    }

    _write_mutex->give();
}



