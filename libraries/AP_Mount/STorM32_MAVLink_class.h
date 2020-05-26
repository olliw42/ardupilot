//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//*****************************************************
#pragma once

#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_Mount/STorM32_lib.h>


//******************************************************
// STorM32_MAVLink_class
//******************************************************

class STorM32_MAVLink_class
{
public:
    STorM32_MAVLink_class();

    // various helper functions
    bool is_normal_state(uint16_t state);

    // functions for sending to the STorM32, using RCcmds
    void send_cmd_storm32link_v2(void);
    void send_cmd_sethomelocation(void);
    void send_cmd_settargetlocation(void);

    // it is easier to provide them in a child class, as the y need chan etc.
    virtual bool _tx_hasspace(const size_t size) = 0;
    virtual size_t _write(const uint8_t* buffer, size_t size) = 0;

protected:
    uint8_t _storm32link_seq;
};
