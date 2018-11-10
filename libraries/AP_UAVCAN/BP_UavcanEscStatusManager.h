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
    
    // write to DataFlash
    void log_to_dataflash(uint16_t esc_index);

    // is called in AP_UAVCAN EscStatus message in-coming handler, and writes to the specified esc_index
    void write_to_escindex(uint16_t esc_index,
            uint32_t error_count, float voltage, float current, float temperature,
            int32_t rpm, uint8_t power_rating_pct);

private:
    static BP_UavcanEscStatusManager* _instance;

    struct escstatus_data { //this is as received from uavcan.equipment.esc.Status
        uint32_t error_count;
        float voltage;
        float current;
        float temperature;
        int32_t rpm;
        uint8_t power_rating_pct;
        //private
        uint64_t timestamp64_us;
        uint32_t timestamp_us;
        float consumed_charge_mah;
        float consumed_energy_wh;
        float temperature_degC;
        uint32_t rx_count;
    };
    struct escstatus_data _escstatus[12];
    uint16_t _esc_maxindex;
};









/*
uavcan.equipment.esc.Status
-------------------------------------------------
Default data type ID: 1034
#
# Generic ESC status.
# Unknown fields should be set to NAN.
#

uint32  error_count         # Resets when the motor restarts
float16 voltage             # Volt
float16 current             # Ampere. Can be negative in case of a regenerative braking.
float16 temperature         # Kelvin
int18   rpm                 # Negative value indicates reverse rotation
uint7   power_rating_pct    # Instant demand factor in percent (percent of maximum power); range 0% to 127%.
uint5   esc_index


KISS ESC 32-bit series onewire telemetry protocol
-------------------------------------------------
- Temperature (resolution 1°C)
- Voltage (resolution 0.01V)
- Current (resolution 0.01A)
- Consumption (resolution 1mAh)
- Electrical Rpm (resolution 100Rpm)

Byte 0: Temperature
Byte 1: Voltage high byte
Byte 2: Voltage low byte
Byte 3: Current high byte
Byte 4: Current low byte
Byte 5: Consumption high byte
Byte 6: Consumption low byte
Byte 7: Rpm high byte
Byte 8: Rpm low byte
Byte 9: 8-bit CRC

int8_t Temperature = Temperature in 1°C
uint16_t Voltage = Volt *100 so 1000 are 10.00V
uint16_t Current = Ampere * 100 so 1000 are 10.00A
uint16_t Consumption = Consumption in 1mAh
uint16_t ERpm = Electrical Rpm /100 so 100 are 10000 Erpm


MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4 11030
------------------------------------------
MAVPACKED(
typedef struct __mavlink_esc_telemetry_1_to_4_t {
 uint16_t voltage[4]; //< [cV] Voltage
 uint16_t current[4]; //< [cA] Current
 uint16_t totalcurrent[4]; //< [mAh] Total current
 uint16_t rpm[4]; //< [rpm] RPM (eRPM)
 uint16_t count[4]; //<  count of telemetry packets received (wraps at 65535)
 uint8_t temperature[4]; //< [degC] Temperature
}) mavlink_esc_telemetry_1_to_4_t;


LOG_ESC1_MSG
------------------------------------------
#define ESC_LABELS "TimeUS,RPM,Volt,Curr,Temp,CTot"
#define ESC_FMT   "QeCCcH"
#define ESC_UNITS "sqvAO-"
#define ESC_MULTS "FBBBB-"

{ LOG_ESC1_MSG, sizeof(log_Esc), \
  "ESC1",  ESC_FMT, ESC_LABELS, ESC_UNITS, ESC_MULTS }, \

    { '-', "" },              // no units e.g. Pi, or a string
    { 'A', "A" },             // Ampere
    { 'O', "degC" },          // degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    { 's', "s" },             // seconds
    { 'q', "rpm" },           // rounds per minute. Not SI, but sometimes more intuitive than Hertz
    { 'v', "V" },             // Volt

            struct log_Esc pkt = {
                LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ESC1_MSG + msg.esc_index)),
                time_us     : time_us,
                rpm         : (int32_t)(msg.rpm),
                voltage     : (uint16_t)(msg.voltage*100.0f + 0.5f),
                current     : (uint16_t)(msg.current*100.0f + 0.5f),
                temperature : (int16_t)(temp_degC*100.0f + 0.5f),
                current_tot : (uint16_t)(0)
            };
*/

