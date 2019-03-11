//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//*****************************************************
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_Mount/STorM32_lib.h>


//******************************************************
// STorM32_class
//******************************************************

#define STORM32_CLASS_RECEIVE_BUFFER_SIZE      96 //the largest RCcmd response can be 77


class STorM32_class
{

public:
    /// Constructor
    STorM32_class();

    // interface to write and read from a serial stream (serial or CAN or else)
    enum PRIORITYENUM {
        PRIORITY_DEFAULT = 0,
        PRIORITY_HIGHER = 1,
        PRIORITY_HIGHEST = 2
    };
    
    bool _serial_is_initialised;

    virtual size_t _serial_txspace(void) = 0;
    virtual size_t _serial_write(const uint8_t* buffer, size_t size, uint8_t priority) = 0;
    size_t _serial_write(const uint8_t* buffer, size_t size){ return _serial_write(buffer, size, PRIORITY_DEFAULT); }
    virtual uint32_t _serial_available(void) = 0;
    virtual int16_t _serial_read(void) = 0;

    // interface to read the raw receiver values
    virtual uint16_t _rcin_read(uint8_t ch){ return 0; };

    // various helper functions
    bool is_normal_state(uint16_t state);

    // functions for sending to the STorM32, using RCcmds
    void send_cmd_storm32link_v2(const AP_AHRS_TYPE& ahrs);
    void send_cmd_setangles(float pitch_deg, float roll_deg, float yaw_deg, uint16_t flags);
    void send_cmd_setpitchrollyaw(uint16_t pitch, uint16_t roll, uint16_t yaw);
    void send_cmd_recentercamera(void);
    void send_cmd_docamera(uint16_t trigger_value);
    void send_cmd_setinputs(void);
    void send_cmd_sethomelocation(const AP_AHRS_TYPE& ahrs);
    void send_cmd_settargetlocation(void);
    void send_cmd_getdatafields(uint16_t flags);
    void send_cmd_getversionstr(void);

    // functions for handling reception of data from the STorM32
    void receive_reset(void);
    void receive_reset_wflush(void);
    void do_receive(void);
    bool message_received(void);

protected:
    // functions for handling reception of data from the STorM32
    void _do_receive_singlechar(void);

    uint8_t _storm32link_seq;

    // for handling reception of data from the STorM32, received from a serial-UART
    enum SERIALSTATEENUM {
        SERIALSTATE_IDLE = 0, //waits for something to come
        SERIALSTATE_RECEIVE_PAYLOAD_LEN,
        SERIALSTATE_RECEIVE_CMD,
        SERIALSTATE_RECEIVE_PAYLOAD,
        SERIALSTATE_MESSAGE_RECEIVED,
        SERIALSTATE_MESSAGE_RECEIVEDANDDIGESTED,
    };

    typedef struct { //structure to process incoming serial data
        // auxiliary fields to handle reception
        enum SERIALSTATEENUM state;
        uint16_t payload_cnt;
        // RCcmd message fields, without crc
        uint8_t stx;
        uint8_t len;
        uint8_t cmd;
        union {
            uint8_t buf[STORM32_CLASS_RECEIVE_BUFFER_SIZE+8]; //have some overhead
            tSTorM32CmdGetDataFieldsAckPayload getdatafields;
            tSTorM32CmdGetVersionStrAckPayload getversionstr;
        };
    } tSerial;
    tSerial _serial_in;

}; //end of class STorM32_class
