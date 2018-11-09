/*
 * AP_UAVCAN.cpp
 *
 *      Author: Eugene Shamaev
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

// Zubax GPS and other GPS, baro, magnetic sensors
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>

#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/SingleLightCommand.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>

#include <uavcan/equipment/power/BatteryInfo.hpp>

//OW
// 1. place the .dsdl datatype file in \modules\uavcan\dsdl\uavcan
// 2. delete folder \modules\uavcan\libuavcan\include\dsdl_generated to invoke the dsdl_compiler
//sadly, only the subfolder \modules\uavcan\dsdl\uavcan is scanned by the build system
//so vendor-specific messages are not possible, we thus must fake our messages into the standard dataset
// 3. doing this causes all sorts of issues with git, which I couldn't work out,
//    they're mainly because I can't add/commit and thus can't checkout to e.g. master
//the workaround is to "somehow" get the .hpp file generated, without affecting the uavcan submodule in any way,
//and to place the new .hpp into the AP_UAVCAN library folder
#include "bp_dsdl_generated/olliw/uc4h/GenericBatteryInfo.hpp"
#include <uavcan/equipment/esc/Status.hpp>
//OWEND

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// TODO: temperature can come not only from baro. There should be separation on node ID
// to check where it belongs to. If it is not baro that is the source, separate layer
// of listeners/nodes should be added.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _uavcan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 0),

    // @Param: SRV_RT
    // @DisplayName: Servo output rate
    // @Description: Maximum transmit rate for servo outputs
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 4, AP_UAVCAN, _servo_rate_hz, 50),
    
    AP_GROUPEND
};

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
#define CAN_PERIODIC_TX_TIMEOUT_MS 2

static void gnss_fix_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());
    if (state == nullptr) {
        return;
    }

    bool process = false;

    if (msg.status == uavcan::equipment::gnss::Fix::STATUS_NO_FIX) {
        state->status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (msg.status == uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY) {
            state->status = AP_GPS::GPS_Status::NO_FIX;
        } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_2D_FIX) {
            state->status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_3D_FIX) {
            state->status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (msg.gnss_time_standard == uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC) {
            uint64_t epoch_ms = uavcan::UtcTime(msg.gnss_timestamp).toUSec();
            epoch_ms /= 1000;
            uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
            state->time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            state->time_week_ms = (uint32_t)(gps_ms - (state->time_week) * AP_MSEC_PER_WEEK);
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = msg.latitude_deg_1e8 / 10;
        loc.lng = msg.longitude_deg_1e8 / 10;
        loc.alt = msg.height_msl_mm / 10;
        state->location = loc;
        state->location.options = 0;

        if (!uavcan::isNaN(msg.ned_velocity[0])) {
            Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
            state->velocity = vel;
            state->ground_speed = norm(vel.x, vel.y);
            state->ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
            state->have_vertical_velocity = true;
        } else {
            state->have_vertical_velocity = false;
        }

        float pos_cov[9];
        msg.position_covariance.unpackSquareMatrix(pos_cov);
        if (!uavcan::isNaN(pos_cov[8])) {
            if (pos_cov[8] > 0) {
                state->vertical_accuracy = sqrtf(pos_cov[8]);
                state->have_vertical_accuracy = true;
            } else {
                state->have_vertical_accuracy = false;
            }
        } else {
            state->have_vertical_accuracy = false;
        }

        const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
        if (!uavcan::isNaN(horizontal_pos_variance)) {
            if (horizontal_pos_variance > 0) {
                state->horizontal_accuracy = sqrtf(horizontal_pos_variance);
                state->have_horizontal_accuracy = true;
            } else {
                state->have_horizontal_accuracy = false;
            }
        } else {
            state->have_horizontal_accuracy = false;
        }

        float vel_cov[9];
        msg.velocity_covariance.unpackSquareMatrix(vel_cov);
        if (!uavcan::isNaN(vel_cov[0])) {
            state->speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
            state->have_speed_accuracy = true;
        } else {
            state->have_speed_accuracy = false;
        }

        state->num_sats = msg.sats_used;
    } else {
        state->have_vertical_velocity = false;
        state->have_vertical_accuracy = false;
        state->have_horizontal_accuracy = false;
        state->have_speed_accuracy = false;
        state->num_sats = 0;
    }

    state->last_gps_time_ms = AP_HAL::millis();

    // after all is filled, update all listeners with new data
    ap_uavcan->update_gps_state(msg.getSrcNodeID().get());
}

static void gnss_fix_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 0); }
static void gnss_fix_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 1); }
static void (*gnss_fix_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
        = { gnss_fix_cb0, gnss_fix_cb1 };

static void gnss_aux_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());
    if (state == nullptr) {
        return;
    }

    if (!uavcan::isNaN(msg.hdop)) {
        state->hdop = msg.hdop * 100.0;
    }

    if (!uavcan::isNaN(msg.vdop)) {
        state->vdop = msg.vdop * 100.0;
    }
}

static void gnss_aux_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 0); }
static void gnss_aux_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 1); }
static void (*gnss_aux_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
        = { gnss_aux_cb0, gnss_aux_cb1 };

static void magnetic_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_UAVCAN::Mag_Info *state = ap_uavcan->find_mag_node(msg.getSrcNodeID().get(), 0);
    if (state == nullptr) {
        return;
    }

    state->mag_vector[0] = msg.magnetic_field_ga[0];
    state->mag_vector[1] = msg.magnetic_field_ga[1];
    state->mag_vector[2] = msg.magnetic_field_ga[2];

    // after all is filled, update all listeners with new data
    ap_uavcan->update_mag_state(msg.getSrcNodeID().get(), 0);
}

static void magnetic_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 0); }
static void magnetic_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 1); }
static void (*magnetic_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
        = { magnetic_cb0, magnetic_cb1 };
        
static void magnetic_cb_2(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength2>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_UAVCAN::Mag_Info *state = ap_uavcan->find_mag_node(msg.getSrcNodeID().get(), msg.sensor_id);
    if (state == nullptr) {
        return;
    }

    state->mag_vector[0] = msg.magnetic_field_ga[0];
    state->mag_vector[1] = msg.magnetic_field_ga[1];
    state->mag_vector[2] = msg.magnetic_field_ga[2];

    // after all is filled, update all listeners with new data
    ap_uavcan->update_mag_state(msg.getSrcNodeID().get(), msg.sensor_id);
}

static void magnetic_cb_2_0(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength2>& msg)
{   magnetic_cb_2(msg, 0); }
static void magnetic_cb_2_1(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength2>& msg)
{   magnetic_cb_2(msg, 1); }
static void (*magnetic_cb_2_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength2>& msg)
        = { magnetic_cb_2_0, magnetic_cb_2_1 };

static void air_data_sp_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(msg.getSrcNodeID().get());
    if (state == nullptr) {
        return;
    }

    state->pressure = msg.static_pressure;
    state->pressure_variance = msg.static_pressure_variance;

    // after all is filled, update all listeners with new data
    ap_uavcan->update_baro_state(msg.getSrcNodeID().get());
}

static void air_data_sp_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 0); }
static void air_data_sp_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 1); }
static void (*air_data_sp_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
        = { air_data_sp_cb0, air_data_sp_cb1 };

// Temperature is not main parameter so do not update listeners when it is received
static void air_data_st_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(msg.getSrcNodeID().get());
    if (state == nullptr) {
        return;
    }

    state->temperature = msg.static_temperature;
    state->temperature_variance = msg.static_temperature_variance;
}

static void air_data_st_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 0); }
static void air_data_st_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 1); }
static void (*air_data_st_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
        = { air_data_st_cb0, air_data_st_cb1 };

static void battery_info_st_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    AP_UAVCAN::BatteryInfo_Info *state = ap_uavcan->find_bi_id((uint16_t) msg.battery_id);
    if (state == nullptr) {
        return;
    }

    state->temperature = msg.temperature;
    state->voltage = msg.voltage;
    state->current = msg.current;
    state->full_charge_capacity_wh = msg.full_charge_capacity_wh;
    state->remaining_capacity_wh = msg.remaining_capacity_wh;
    state->status_flags = msg.status_flags;

    // after all is filled, update all listeners with new data
    ap_uavcan->update_bi_state((uint16_t) msg.battery_id);
}

static void battery_info_st_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo>& msg)
{   battery_info_st_cb(msg, 0); }
static void battery_info_st_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo>& msg)
{   battery_info_st_cb(msg, 1); }
static void (*battery_info_st_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo>& msg)
        = { battery_info_st_cb0, battery_info_st_cb1 };

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>* act_out_array[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::esc::RawCommand>* esc_raw[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::indication::LightsCommand>* rgb_led[MAX_NUMBER_OF_CAN_DRIVERS];

//OW
//--- uc4h.GenericBatteryInfo ---
// incoming message, by id

static void uc4hgenericbatteryinfo_cb_func(const uavcan::ReceivedDataStructure<uavcan::olliw::uc4h::GenericBatteryInfo>& msg, uint8_t mgr)
{
    AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }

    uint8_t id = msg.battery_id; //by device id

    AP_UAVCAN::Uc4hGenericBatteryInfo_Data* data = ap_uavcan->uc4hgenericbatteryinfo_getptrto_data(id); //i is in data->i
    if (data != nullptr) {
        data->voltage = msg.voltage;
        data->current = msg.current;
        data->charge_consumed_mAh = msg.charge_consumed_mAh;
        data->energy_consumed_Wh = msg.energy_consumed_Wh;
        data->status_flags = msg.status_flags;
        data->cell_voltages_num = msg.cell_voltages.size();
        for (uint16_t i = 0; i < 12; i++) { //TODO: fund and use the macro for that
            data->cell_voltages[i] = (i < data->cell_voltages_num ) ? msg.cell_voltages[i] : 0.0f; //NAN;
        }
        ap_uavcan->uc4hgenericbatteryinfo_update_i(data->i);
    }
}

static void uc4hgenericbatteryinfo_cb0(const uavcan::ReceivedDataStructure<uavcan::olliw::uc4h::GenericBatteryInfo>& msg){ uc4hgenericbatteryinfo_cb_func(msg, 0); }
static void uc4hgenericbatteryinfo_cb1(const uavcan::ReceivedDataStructure<uavcan::olliw::uc4h::GenericBatteryInfo>& msg){ uc4hgenericbatteryinfo_cb_func(msg, 1); }
static void (*uc4hgenericbatteryinfo_cb[2])(const uavcan::ReceivedDataStructure<uavcan::olliw::uc4h::GenericBatteryInfo>& msg) = { uc4hgenericbatteryinfo_cb0, uc4hgenericbatteryinfo_cb1 };

//--- EscStatus ---
// incoming message, by id

static void escstatus_cb_func(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }

    //different to usual: we do write here directly to the BP_UavcanEscStatusManager class
    BP_UavcanEscStatusManager* escstatusmgr = BP_UavcanEscStatusManager::instance();
    if (escstatusmgr && (msg.esc_index < 8)) {
        escstatusmgr->write_to_escindex(msg.esc_index,
                msg.error_count, msg.voltage, msg.current, msg.temperature,
                msg.rpm, msg.power_rating_pct);
    }

    // only 8 LOG_ESC1_MSG are defined, see /libraries/DataFlash/LogStructure.h
    //TODO: do not log packets with error???
    // no, it would be better to extend the ESC log message, and to drop wrong packages on the node side
    if (msg.esc_index < 8) {
        DataFlash_Class* df = DataFlash_Class::instance();
        if (df && df->logging_enabled()) {
            uint64_t time_us = AP_HAL::micros64();
            float temp_degC = (!uavcan::isNaN(msg.temperature) && (msg.temperature > 0.1f)) ? msg.temperature - 273.15f : 0.0f;
            struct log_Esc pkt = {
                LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ESC1_MSG + msg.esc_index)),
                time_us     : time_us,
                rpm         : (int32_t)(msg.rpm),
                voltage     : (uint16_t)(msg.voltage*100.0f + 0.5f),
                current     : (uint16_t)(msg.current*100.0f + 0.5f),
                temperature : (int16_t)(temp_degC*100.0f + 0.5f),
                current_tot : (uint16_t)(0)
            };
            df->WriteBlock(&pkt, sizeof(pkt));
        }
    }

    uint8_t id = msg.esc_index; //by device id

    AP_UAVCAN::EscStatus_Data *data = ap_uavcan->escstatus_getptrto_data(id); //i is in data->i
    if (data != nullptr) {
        data->error_count = msg.error_count;
        data->voltage = msg.voltage;
        data->current = msg.current;
        data->temperature = msg.temperature;
        data->rpm = msg.rpm;
        data->power_rating_pct = msg.power_rating_pct;

        ap_uavcan->escstatus_update_i(data->i);
    }
}

static void escstatus_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg){ escstatus_cb_func(msg, 0); }
static void escstatus_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg){ escstatus_cb_func(msg, 1); }
static void (*escstatus_cb[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg) = { escstatus_cb0, escstatus_cb1 };

// --- tunnel.Broadcast ---
// incoming message

static void tunnelbroadcast_cb_func(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast>& msg, uint8_t mgr)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return;
    }

    //different to usual: we do write here directly to the BP_UavcanTunnelManager class
    BP_UavcanTunnelManager* tunnelmgr = BP_UavcanTunnelManager::instance();
    if (tunnelmgr) {
        tunnelmgr->write_to_channel(msg.channel_id, &(msg.buffer[0]), msg.buffer.size());
    }
}

static void tunnelbroadcast_cb0(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast>& msg){ tunnelbroadcast_cb_func(msg, 0); }
static void tunnelbroadcast_cb1(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast>& msg){ tunnelbroadcast_cb_func(msg, 1); }
static void (*tunnelbroadcast_cb[2])(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast>& msg) = { tunnelbroadcast_cb0, tunnelbroadcast_cb1 };


// publisher interfaces
//--- uc4h.Notify ---
// outgoing message
static uavcan::Publisher<uavcan::olliw::uc4h::Notify>* uc4hnotify_array[MAX_NUMBER_OF_CAN_DRIVERS];
// --- tunnel.Broadcast ---
// outgoing message
static uavcan::Publisher<uavcan::tunnel::Broadcast>* tunnelbroadcast_out_array[MAX_NUMBER_OF_CAN_DRIVERS];

// further stuff for outgoing messages
const uavcan::TransferPriority TwoLowerThanHighest(uavcan::TransferPriority::NumericallyMin + 2);
const uavcan::TransferPriority OneHigherThanDefault((1U << uavcan::TransferPriority::BitLen) / 2 - 1);
//OWEND


//--------------   AP_UAVCAN methods  --------------------

AP_UAVCAN::AP_UAVCAN() :
    _node_allocator(
        UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        _gps_nodes[i] = UINT8_MAX;
        _gps_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        _baro_nodes[i] = UINT8_MAX;
        _baro_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        _mag_nodes[i] = UINT8_MAX;
        _mag_node_taken[i] = 0;
        _mag_node_max_sensorid_count[i] = 1;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        _gps_listener_to_node[i] = UINT8_MAX;
        _gps_listeners[i] = nullptr;

        _baro_listener_to_node[i] = UINT8_MAX;
        _baro_listeners[i] = nullptr;

        _mag_listener_to_node[i] = UINT8_MAX;
        _mag_listeners[i] = nullptr;
        _mag_listener_sensor_ids[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        _bi_id[i] = UINT8_MAX;
        _bi_id_taken[i] = 0;
        _bi_BM_listener_to_id[i] = UINT8_MAX;
        _bi_BM_listeners[i] = nullptr;
    }

    SRV_sem = hal.util->new_semaphore();
    _led_out_sem = hal.util->new_semaphore();

//OW
    // --- uc4h.GenericBatteryInfo ---
    for (uint8_t i = 0; i < AP_UAVCAN_UC4HGENERICBATTERYINFO_MAX_NUMBER; i++) {
        _uc4hgenericbatteryinfo.id[i] = UINT8_MAX;
        _uc4hgenericbatteryinfo.id_taken[i] = 0;
    }
    for (uint8_t li = 0; li < AP_UAVCAN_MAX_LISTENERS; li++) {
        _uc4hgenericbatteryinfo.listener_to_id[li] = UINT8_MAX;
        _uc4hgenericbatteryinfo.listeners[li] = nullptr;
    }

    // --- EscStatus ---
    _escstatus.listener = nullptr;
    for (uint8_t i = 0; i < AP_UAVCAN_ESCSTATUS_MAX_NUMBER; i++) {
        _escstatus.id[i] = UINT8_MAX;
    }

    // --- uc4h.Notify ---
    _uc4hnotify.to_send = false;
    _uc4hnotify.sem = hal.util->new_semaphore();

    // --- tunnel.Broadcast in ---
    // nothing to do
    // --- tunnel.Broadcast out ---
    _tunnelbroadcast_out.sem = hal.util->new_semaphore();
//OWEND

    debug_uavcan(2, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

bool AP_UAVCAN::try_init(void)
{
    if (_parent_can_mgr == nullptr) {
        return false;
    }

    if (_initialized) {
        return true;
    }
    
    if (!_parent_can_mgr->is_initialized()) {
        return false;
    }
    
    _uavcan_i = UINT8_MAX;
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (_parent_can_mgr == hal.can_mgr[i]) {
            _uavcan_i = i;
            break;
        }
    }

    if(_uavcan_i == UINT8_MAX) {
        return false;
    }

    auto *node = get_node();

    if (node == nullptr) {
        return false;
    }
    
    if (node->isStarted()) {
        return false;
    }
    
    uavcan::NodeID self_node_id(_uavcan_node);
    node->setNodeID(self_node_id);

    char ndname[20];
    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", _uavcan_i);

    uavcan::NodeStatusProvider::NodeName name(ndname);
    node->setName(name);

    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    node->setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;
    node->setHardwareVersion(hw_version);

    const int node_start_res = node->start();
    if (node_start_res < 0) {
        debug_uavcan(1, "UAVCAN: node start problem\n\r");
    }

    uavcan::Subscriber<uavcan::equipment::gnss::Fix> *gnss_fix;
    gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix>(*node);

    const int gnss_fix_start_res = gnss_fix->start(gnss_fix_cb_arr[_uavcan_i]);
    if (gnss_fix_start_res < 0) {
        debug_uavcan(1, "UAVCAN GNSS subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary> *gnss_aux;
    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary>(*node);
    const int gnss_aux_start_res = gnss_aux->start(gnss_aux_cb_arr[_uavcan_i]);
    if (gnss_aux_start_res < 0) {
        debug_uavcan(1, "UAVCAN GNSS Aux subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength> *magnetic;
    magnetic = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength>(*node);
    const int magnetic_start_res = magnetic->start(magnetic_cb_arr[_uavcan_i]);
    if (magnetic_start_res < 0) {
        debug_uavcan(1, "UAVCAN Compass subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2> *magnetic2;
    magnetic2 = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2>(*node);
    const int magnetic_start_res_2 = magnetic2->start(magnetic_cb_2_arr[_uavcan_i]);
    if (magnetic_start_res_2 < 0) {
        debug_uavcan(1, "UAVCAN Compass for multiple mags subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure> *air_data_sp;
    air_data_sp = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure>(*node);
    const int air_data_sp_start_res = air_data_sp->start(air_data_sp_cb_arr[_uavcan_i]);
    if (air_data_sp_start_res < 0) {
        debug_uavcan(1, "UAVCAN Baro subscriber start problem\n\r");
        return false;
    }
    
    uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature> *air_data_st;
    air_data_st = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature>(*node);
    const int air_data_st_start_res = air_data_st->start(air_data_st_cb_arr[_uavcan_i]);
    if (air_data_st_start_res < 0) {
        debug_uavcan(1, "UAVCAN Temperature subscriber start problem\n\r");
        return false;
    }
    
    uavcan::Subscriber<uavcan::equipment::power::BatteryInfo> *battery_info_st;
    battery_info_st = new uavcan::Subscriber<uavcan::equipment::power::BatteryInfo>(*node);
    const int battery_info_start_res = battery_info_st->start(battery_info_st_cb_arr[_uavcan_i]);
    if (battery_info_start_res < 0) {
        debug_uavcan(1, "UAVCAN BatteryInfo subscriber start problem\n\r");
        return false;
    }

    act_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*node);
    act_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    act_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    esc_raw[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*node);
    esc_raw[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    esc_raw[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

    rgb_led[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::indication::LightsCommand>(*node);
    rgb_led[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    rgb_led[_uavcan_i]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    _led_conf.devices_count = 0;

//OW
    uavcan::Subscriber<uavcan::olliw::uc4h::GenericBatteryInfo>* uc4hgenericbatteryinfo_sub;
    uc4hgenericbatteryinfo_sub = new uavcan::Subscriber<uavcan::olliw::uc4h::GenericBatteryInfo>(*node);
    const int uc4hgenericbatteryinfo_start_res = uc4hgenericbatteryinfo_sub->start(uc4hgenericbatteryinfo_cb[_uavcan_i]);
    if (uc4hgenericbatteryinfo_start_res < 0) {
        debug_uavcan(1, "UAVCAN Uc4hGenericBatteryInfo subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::equipment::esc::Status>* escstatus_sub;
    escstatus_sub = new uavcan::Subscriber<uavcan::equipment::esc::Status>(*node);
    const int escstatus_start_res = escstatus_sub->start(escstatus_cb[_uavcan_i]);
    if (escstatus_start_res < 0) {
        debug_uavcan(1, "UAVCAN EscStatus subscriber start problem\n\r");
        return false;
    }

    uavcan::Subscriber<uavcan::tunnel::Broadcast>* tunnelbroadcast_sub;
    tunnelbroadcast_sub = new uavcan::Subscriber<uavcan::tunnel::Broadcast>(*node);
    const int tunnelbroadcast_start_res = tunnelbroadcast_sub->start(tunnelbroadcast_cb[_uavcan_i]);
    if (tunnelbroadcast_start_res < 0) {
        debug_uavcan(1, "UAVCAN tunnel::Broadcast subscriber start problem\n\r");
        return false;
    }

    uc4hnotify_array[_uavcan_i] = new uavcan::Publisher<uavcan::olliw::uc4h::Notify>(*node);
    uc4hnotify_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    uc4hnotify_array[_uavcan_i]->setPriority(uavcan::TransferPriority::MiddleLower);

    tunnelbroadcast_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::tunnel::Broadcast>(*node);
    tunnelbroadcast_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
    tunnelbroadcast_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::MiddleLower); //this will be overwritten later
//OWEND

    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    node->setModeOperational();

    _initialized = true;

    debug_uavcan(1, "UAVCAN: init done\n\r");

    return true;
}

void AP_UAVCAN::SRV_sem_take()
{
    SRV_sem->take_blocking();
}

void AP_UAVCAN::SRV_sem_give()
{
    SRV_sem->give();
}

void AP_UAVCAN::SRV_send_servos(void)
{
    uint8_t starting_servo = 0;
    bool repeat_send;

    do {
        repeat_send = false;
        uavcan::equipment::actuator::ArrayCommand msg;

        SRV_sem_take();
        
        uint8_t i;
        // UAVCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < UAVCAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan::equipment::actuator::Command cmd;

            /*
             * Servo output uses a range of 1000-2000 PWM for scaling.
             * This converts output PWM from [1000:2000] range to [-1:1] range that
             * is passed to servo as unitless type via UAVCAN.
             * This approach allows for MIN/TRIM/MAX values to be used fully on
             * autopilot side and for servo it should have the setup to provide maximum
             * physically possible throws at [-1:1] limits.
             */

            if (_SRV_conf[starting_servo].servo_pending && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                cmd.actuator_id = starting_servo + 1;

                // TODO: other types
                cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

                // TODO: failsafe, safety
                cmd.command_value = constrain_float(((float) _SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);

                msg.commands.push_back(cmd);

                i++;
            }
        }

        SRV_sem_give();
        
        if (i > 0) {
            act_out_array[_uavcan_i]->broadcast(msg);

            if (i == 15) {
                repeat_send = true;
            }
        }
    } while (repeat_send);
}

void AP_UAVCAN::SRV_send_esc(void)
{
    static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
    uavcan::equipment::esc::RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
            max_esc_num = i + 1;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        k = 0;

        SRV_sem_take();
        
        for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
            if ((((uint32_t) 1) << i) & _esc_bm) {
                // TODO: ESC negative scaling for reverse thrust and reverse rotation
                float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[i].pulse) + 1.0) / 2.0;

                scaled = constrain_float(scaled, 0, cmd_max);

                esc_msg.cmd.push_back(static_cast<int>(scaled));
            } else {
                esc_msg.cmd.push_back(static_cast<unsigned>(0));
            }

            k++;
        }
        SRV_sem_give();

        esc_raw[_uavcan_i]->broadcast(esc_msg);
    }
}

void AP_UAVCAN::do_cyclic(void)
{
    if (!_initialized) {
        hal.scheduler->delay_microseconds(1000);
        return;
    }

    auto *node = get_node();

    const int error = node->spin(uavcan::MonotonicDuration::fromMSec(1));

    if (error < 0) {
        hal.scheduler->delay_microseconds(100);
        return;
    }

    if (_SRV_armed) {
        bool sent_servos = false;
            
        if (_servo_bm > 0) {
            // if we have any Servos in bitmask
            uint32_t now = AP_HAL::micros();
            const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
            if (now - _SRV_last_send_us >= servo_period_us) {
                _SRV_last_send_us = now;
                SRV_send_servos();
                sent_servos = true;
                for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                    _SRV_conf[i].servo_pending = false;
                }
            }
        }

        // if we have any ESC's in bitmask
        if (_esc_bm > 0 && !sent_servos) {
            SRV_send_esc();
        }
        
        for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
            _SRV_conf[i].esc_pending = false;
        }
    }

    if (led_out_sem_take()) {

        led_out_send();
        led_out_sem_give();
    }

//OW
    uint64_t current_time_ms = AP_HAL::millis64();
    tunnelbroadcast_out_do_cyclic(); //give it preference
    uc4h_do_cyclic(current_time_ms);
//OWEND
}

bool AP_UAVCAN::led_out_sem_take()
{
    bool sem_ret = _led_out_sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN LEDOut semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_UAVCAN::led_out_sem_give()
{
    _led_out_sem->give();
}

void AP_UAVCAN::led_out_send()
{
    if (_led_conf.broadcast_enabled && ((AP_HAL::micros64() - _led_conf.last_update) > (AP_UAVCAN_LED_DELAY_MILLISECONDS * 1000))) {
        uavcan::equipment::indication::LightsCommand msg;
        uavcan::equipment::indication::SingleLightCommand cmd;

        for (uint8_t i = 0; i < _led_conf.devices_count; i++) {
            if (_led_conf.devices[i].enabled) {
                cmd.light_id =_led_conf.devices[i].led_index;
                cmd.color = _led_conf.devices[i].rgb565_color;
                msg.commands.push_back(cmd);
            }
        }

        rgb_led[_uavcan_i]->broadcast(msg);
        _led_conf.last_update = AP_HAL::micros64();
    }
}

uavcan::ISystemClock & AP_UAVCAN::get_system_clock()
{
    return SystemClock::instance();
}

uavcan::ICanDriver * AP_UAVCAN::get_can_driver()
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return _parent_can_mgr->get_driver();
        }
    }
    return nullptr;
}

uavcan::Node<0> *AP_UAVCAN::get_node()
{
    if (_node == nullptr && get_can_driver() != nullptr) {
        _node = new uavcan::Node<0>(*get_can_driver(), get_system_clock(), _node_allocator);
    }

    return _node;
}

void AP_UAVCAN::SRV_set_safety_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _SRV_conf[i].safety_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::SRV_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _SRV_conf[i].failsafe_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::SRV_force_safety_on(void)
{
    _SRV_safety = true;
}

void AP_UAVCAN::SRV_force_safety_off(void)
{
    _SRV_safety = false;
}

void AP_UAVCAN::SRV_arm_actuators(bool arm)
{
    _SRV_armed = arm;
}

void AP_UAVCAN::SRV_write(uint16_t pulse_len, uint8_t ch)
{
    _SRV_conf[ch].pulse = pulse_len;
    _SRV_conf[ch].esc_pending = true;
    _SRV_conf[ch].servo_pending = true;
}

void AP_UAVCAN::SRV_push_servos()
{
    SRV_sem_take();

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i)) {
            SRV_write(SRV_Channels::srv_channel(i)->get_output_pwm(), i);
        }
    }

    SRV_sem_give();
    
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        SRV_arm_actuators(true);
    } else {
        SRV_arm_actuators(false);
    }
}

uint8_t AP_UAVCAN::find_gps_without_listener(void)
{
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr && _gps_nodes[i] != UINT8_MAX) {
            return _gps_nodes[i];
        }
    }

    return UINT8_MAX;
}

uint8_t AP_UAVCAN::register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }

    if (preferred_channel != 0 && preferred_channel <= AP_UAVCAN_MAX_GPS_NODES) {
        _gps_listeners[sel_place] = new_listener;
        _gps_listener_to_node[sel_place] = preferred_channel - 1;
        _gps_node_taken[_gps_listener_to_node[sel_place]]++;
        ret = preferred_channel;

        debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, preferred_channel);
    } else {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
            if (_gps_node_taken[i] == 0) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = i;
                _gps_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            _gps_listeners[sel_place] = new_listener;
            _gps_listener_to_node[sel_place] = i;
            _gps_node_taken[i]++;
            ret = i + 1;

            debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
            break;
        }
    }

    return ret;
}

void AP_UAVCAN::remove_gps_listener(AP_GPS_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == rem_listener) {
            _gps_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_gps_node_taken[_gps_listener_to_node[i]] > 0) {
                _gps_node_taken[_gps_listener_to_node[i]]--;
            }
            _gps_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_GPS::GPS_State *AP_UAVCAN::find_gps_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            return &_gps_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == UINT8_MAX) {
            _gps_nodes[i] = node;
            return &_gps_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_gps_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] != node) {
            continue;
        }
        for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
            if (_gps_listener_to_node[j] == i) {
                _gps_listeners[j]->handle_gnss_msg(_gps_node_state[i]);
            }
        }
    }
}

uint8_t AP_UAVCAN::register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }
    if (preferred_channel != 0 && preferred_channel < AP_UAVCAN_MAX_BARO_NODES) {
        _baro_listeners[sel_place] = new_listener;
        _baro_listener_to_node[sel_place] = preferred_channel - 1;
        _baro_node_taken[_baro_listener_to_node[sel_place]]++;
        ret = preferred_channel;

        debug_uavcan(2, "reg_Baro place:%d, chan: %d\n\r", sel_place, preferred_channel);
    } else {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
            if (_baro_node_taken[i] == 0) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = i;
                _baro_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
            if (_baro_nodes[i] != node) {
                continue;
            }
            _baro_listeners[sel_place] = new_listener;
            _baro_listener_to_node[sel_place] = i;
            _baro_node_taken[i]++;
            ret = i + 1;

            debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
            break;
        }
    }

    return ret;
}

void AP_UAVCAN::remove_baro_listener(AP_Baro_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] != rem_listener) {
            continue;
        }
        _baro_listeners[i] = nullptr;

        // Also decrement usage counter and reset listening node
        if (_baro_node_taken[_baro_listener_to_node[i]] > 0) {
            _baro_node_taken[_baro_listener_to_node[i]]--;
        }
        _baro_listener_to_node[i] = UINT8_MAX;
    }
}

AP_UAVCAN::Baro_Info *AP_UAVCAN::find_baro_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            return &_baro_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == UINT8_MAX) {

            _baro_nodes[i] = node;
            return &_baro_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_baro_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] != node) {
            continue;
        }
        for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
            if (_baro_listener_to_node[j] == i) {
                _baro_listeners[j]->handle_baro_msg(_baro_node_state[i].pressure, _baro_node_state[i].temperature);
            }
        }
    }
}

/*
 * Find discovered not taken baro node with smallest node ID
 */
uint8_t AP_UAVCAN::find_smallest_free_baro_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_node_taken[i] == 0) {
            ret = MIN(ret, _baro_nodes[i]);
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }
    if (preferred_channel != 0 && preferred_channel < AP_UAVCAN_MAX_MAG_NODES) {
        _mag_listeners[sel_place] = new_listener;
        _mag_listener_to_node[sel_place] = preferred_channel - 1;
        _mag_node_taken[_mag_listener_to_node[sel_place]]++;
        ret = preferred_channel;

        debug_uavcan(2, "reg_Compass place:%d, chan: %d\n\r", sel_place, preferred_channel);
    } else {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
            if (_mag_node_taken[i] == 0) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = i;
                _mag_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] != node) {
            continue;
        }
        _mag_listeners[sel_place] = new_listener;
        _mag_listener_to_node[sel_place] = i;
        _mag_listener_sensor_ids[sel_place] = 0;
        _mag_node_taken[i]++;
        ret = i + 1;

        debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
        break;
    }

    return ret;
}

void AP_UAVCAN::remove_mag_listener(AP_Compass_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] != rem_listener) {
            continue;
        }
        _mag_listeners[i] = nullptr;

        // Also decrement usage counter and reset listening node
        if (_mag_node_taken[_mag_listener_to_node[i]] > 0) {
            _mag_node_taken[_mag_listener_to_node[i]]--;
        }
        _mag_listener_to_node[i] = UINT8_MAX;
    }
}

AP_UAVCAN::Mag_Info *AP_UAVCAN::find_mag_node(uint8_t node, uint8_t sensor_id)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] != node) {
            continue;
        }
        if (_mag_node_max_sensorid_count[i] < sensor_id) {
            _mag_node_max_sensorid_count[i] = sensor_id;
            debug_uavcan(2, "AP_UAVCAN: Compass: found sensor id %d on node %d\n\r", (int)(sensor_id), (int)(node));
        }
        return &_mag_node_state[i];
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] != UINT8_MAX) {
            continue;
        }
        _mag_nodes[i] = node;
        _mag_node_max_sensorid_count[i] = (sensor_id ? sensor_id : 1);
        debug_uavcan(2, "AP_UAVCAN: Compass: register sensor id %d on node %d\n\r", (int)(sensor_id), (int)(node));
        return &_mag_node_state[i];
    }

    // If no space is left - return nullptr
    return nullptr;
}

/*
 * Find discovered mag node with smallest node ID and which is taken N times,
 * where N is less than its maximum sensor id.
 * This allows multiple AP_Compass_UAVCAN instanses listening multiple compasses
 * that are on one node.
 */
uint8_t AP_UAVCAN::find_smallest_free_mag_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_node_taken[i] < _mag_node_max_sensorid_count[i]) {
            ret = MIN(ret, _mag_nodes[i]);
        }
    }

    return ret;
}

void AP_UAVCAN::update_mag_state(uint8_t node, uint8_t sensor_id)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] != node) {
            continue;
        }
        for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
            if (_mag_listener_to_node[j] != i) {
                continue;
            }

            /*If the current listener has default sensor_id,
              while our sensor_id is not default, we have
              to assign our sensor_id to this listener*/
            if ((_mag_listener_sensor_ids[j] == 0) && (sensor_id != 0)) {
                bool already_taken = false;
                for (uint8_t k = 0; k < AP_UAVCAN_MAX_LISTENERS; k++) {
                    if (_mag_listener_sensor_ids[k] == sensor_id) {
                        already_taken = true;
                    }
                }
                if (!already_taken) {
                    debug_uavcan(2, "AP_UAVCAN: Compass: sensor_id updated to %d for listener %d\n", sensor_id, j);
                    _mag_listener_sensor_ids[j] = sensor_id;
                }
            }

            /*If the current listener has the sensor_id that we have,
              or our sensor_id is default, ask the listener to handle the measurements
              (the default one is used for the nodes that have only one compass*/
            if ((sensor_id == 0) || (_mag_listener_sensor_ids[j] == sensor_id)) {
                _mag_listeners[j]->handle_mag_msg(_mag_node_state[i].mag_vector);
            }
        }
    }
}

uint8_t AP_UAVCAN::register_BM_bi_listener_to_id(AP_BattMonitor_Backend* new_listener, uint8_t id)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_bi_BM_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place == UINT8_MAX) {
        return 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        if (_bi_id[i] != id) {
            continue;
        }
        _bi_BM_listeners[sel_place] = new_listener;
        _bi_BM_listener_to_id[sel_place] = i;
        _bi_id_taken[i]++;
        ret = i + 1;

        debug_uavcan(2, "reg_BI place:%d, chan: %d\n\r", sel_place, i);
        break;
    }

    return ret;
}

void AP_UAVCAN::remove_BM_bi_listener(AP_BattMonitor_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
       if (_bi_BM_listeners[i] != rem_listener) {
           continue;
       }

       _bi_BM_listeners[i] = nullptr;

       // Also decrement usage counter and reset listening node
       if (_bi_id_taken[_bi_BM_listener_to_id[i]] > 0) {
           _bi_id_taken[_bi_BM_listener_to_id[i]]--;
       }
       _bi_BM_listener_to_id[i] = UINT8_MAX;
    }
}

AP_UAVCAN::BatteryInfo_Info *AP_UAVCAN::find_bi_id(uint8_t id)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        if (_bi_id[i] == id) {
            return &_bi_id_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        if (_bi_id[i] == UINT8_MAX) {
            _bi_id[i] = id;
            return &_bi_id_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

uint8_t AP_UAVCAN::find_smallest_free_bi_id()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        if (_bi_id_taken[i] == 0) {
            ret = MIN(ret, _bi_id[i]);
        }
    }

    return ret;
}

void AP_UAVCAN::update_bi_state(uint8_t id)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BI_NUMBER; i++) {
        if (_bi_id[i] != id) {
            continue;
        }
        for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
            if (_bi_BM_listener_to_id[j] != i) {
                continue;
            }
            _bi_BM_listeners[j]->handle_bi_msg(_bi_id_state[i].voltage, _bi_id_state[i].current, _bi_id_state[i].temperature);
        }
    }
}

bool AP_UAVCAN::led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue) {
    
    if (_led_conf.devices_count >= AP_UAVCAN_MAX_LED_DEVICES) {
        return false;
    }
    if (!led_out_sem_take()) {
        return false;
    }
    
    uavcan::equipment::indication::RGB565 color;
    
    color.red = (red >> 3);
    color.green = (green >> 2);
    color.blue = (blue >> 3);

    // check if a device instance exists. if so, break so the instance index is remembered
    uint8_t instance = 0;
    for (; instance < _led_conf.devices_count; instance++) {
        if (!_led_conf.devices[instance].enabled || (_led_conf.devices[instance].led_index == led_index)) {
            break;
        }
    }
    
    // load into the correct instance.
    // if an existing instance was found in above for loop search,
    // then instance value is < _led_conf.devices_count.
    // otherwise a new one was just found so we increment the count.
    // Either way, the correct instance is the cirrent value of instance
    _led_conf.devices[instance].led_index = led_index;
    _led_conf.devices[instance].rgb565_color = color;
    _led_conf.devices[instance].enabled = true;
    if (instance == _led_conf.devices_count) {
        _led_conf.devices_count++;
    }

    _led_conf.broadcast_enabled = true;
    led_out_sem_give();
    return true;
}

AP_UAVCAN *AP_UAVCAN::get_uavcan(uint8_t iface)
{
    if (iface >= MAX_NUMBER_OF_CAN_INTERFACES || !hal.can_mgr[iface]) {
        return nullptr;
    }
    return hal.can_mgr[iface]->get_UAVCAN();
}

//OW
// my convention: i for AP_UAVCAN_GENERICBATTERYINFO_MAX_NUMBER, li for AP_UAVCAN_MAX_LISTENERS

//--- uc4h.GenericBatteryInfo ---
// incoming message, by device id

uint8_t AP_UAVCAN::uc4hgenericbatteryinfo_register_listener(AP_BattMonitor_Backend* new_listener, uint8_t id)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    //find first free place in listeners list
    for (uint8_t li = 0; li < AP_UAVCAN_MAX_LISTENERS; li++) {
        if (_uc4hgenericbatteryinfo.listeners[li] == nullptr) {
            sel_place = li;
            break;
        }
    }

    //no free place, abort
    if (sel_place == UINT8_MAX) {
        return ret;
    }

    //insert listener
    for (uint8_t i = 0; i < AP_UAVCAN_UC4HGENERICBATTERYINFO_MAX_NUMBER; i++) {
        if (_uc4hgenericbatteryinfo.id[i] != id) {
            continue;
        }
        _uc4hgenericbatteryinfo.listeners[sel_place] = new_listener;
        _uc4hgenericbatteryinfo.listener_to_id[sel_place] = i;
        _uc4hgenericbatteryinfo.id_taken[i]++;
        ret = i + 1;
        debug_uavcan(2, "reg_UC4HGENERICBATTERYINFO place:%d, chan: %d\n\r", sel_place, i);
        break;
    }

    return ret;
}

//is not used, since remove_BM_bi_listener() is also not used, AP_BattMonitor_UAVCAN doesn't have a destructor defined
void AP_UAVCAN::uc4hgenericbatteryinfo_remove_listener(AP_BattMonitor_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t li = 0; li < AP_UAVCAN_MAX_LISTENERS; li++) {
        if (_uc4hgenericbatteryinfo.listeners[li] != rem_listener) {
            continue;
        }
        _uc4hgenericbatteryinfo.listeners[li] = nullptr;

        // Also decrement usage counter and reset listening node
        if (_uc4hgenericbatteryinfo.id_taken[_uc4hgenericbatteryinfo.listener_to_id[li]] > 0) {
            _uc4hgenericbatteryinfo.id_taken[_uc4hgenericbatteryinfo.listener_to_id[li]]--;
        }
        _uc4hgenericbatteryinfo.listener_to_id[li] = UINT8_MAX;
    }
}

AP_UAVCAN::Uc4hGenericBatteryInfo_Data* AP_UAVCAN::uc4hgenericbatteryinfo_getptrto_data(uint8_t id)
{
    // check if id is already in list, and if it is, take it
    for (uint8_t i = 0; i < AP_UAVCAN_UC4HGENERICBATTERYINFO_MAX_NUMBER; i++) {
        if (_uc4hgenericbatteryinfo.id[i] == id) {
            _uc4hgenericbatteryinfo.data[i].i = i; //this avoids needing a 2nd loop in update_i()
            return &_uc4hgenericbatteryinfo.data[i];
        }
    }

    // if id is not yet in list, find the first free spot, and take that
    for (uint8_t i = 0; i < AP_UAVCAN_UC4HGENERICBATTERYINFO_MAX_NUMBER; i++) {
        if (_uc4hgenericbatteryinfo.id[i] != UINT8_MAX) {
            continue;
        }
        _uc4hgenericbatteryinfo.id[i] = id;
        _uc4hgenericbatteryinfo.data[i].i = i; //this avoids needing a 2nd loop in update_i()
        return &_uc4hgenericbatteryinfo.data[i];
    }

    return nullptr;
}

void AP_UAVCAN::uc4hgenericbatteryinfo_update_i(uint8_t i)
{
    for (uint8_t li = 0; li < AP_UAVCAN_MAX_LISTENERS; li++) {
        if (_uc4hgenericbatteryinfo.listener_to_id[li] != i) {
            continue;
        }

        _uc4hgenericbatteryinfo.listeners[li]->handle_uc4hgenericbatteryinfo_msg(
                _uc4hgenericbatteryinfo.data[i].voltage,
                _uc4hgenericbatteryinfo.data[i].current,
                _uc4hgenericbatteryinfo.data[i].charge_consumed_mAh,
                _uc4hgenericbatteryinfo.data[i].energy_consumed_Wh );
    }
}


//--- EscStatus ---
// incoming message, there is just one listener

uint8_t AP_UAVCAN::escstatus_register_listener(AP_BattMonitor_Backend* new_listener, uint8_t id)
{
    //find first free place in listeners list
    // we have just one listener
    //no free place, abort
    if (_escstatus.listener != nullptr) return 0;

    //insert listener
    // we have just one listener
    _escstatus.listener = new_listener;
    debug_uavcan(2, "reg_ESCSTATUS place:%d, chan: %d\n\r", 0, 0);

    return 1;
}

AP_UAVCAN::EscStatus_Data* AP_UAVCAN::escstatus_getptrto_data(uint8_t id)
{
    // I think the esc_index are continuous, by how ArduPilot works
    // so we could instead just directly jump with id into the data list, return &_escstatus.data[id], with id<8 overflow protection of course

    // check if id is already in list, and if it is take it
    for (uint8_t i = 0; i < AP_UAVCAN_ESCSTATUS_MAX_NUMBER; i++) {
        if (_escstatus.id[i] == id) {
            _escstatus.data[i].i = i; //this avoids needing a 2nd loop in update_i()
            return &_escstatus.data[i];
        }
    }

    // if id is not yet in list, find the first free spot, and take that
    for (uint8_t i = 0; i < AP_UAVCAN_ESCSTATUS_MAX_NUMBER; i++) {
        if (_escstatus.id[i] != UINT8_MAX) {
            continue;
        }
        _escstatus.id[i] = id;
        _escstatus.data[i].i = i; //this avoids needing a 2nd loop in update_i()
        return &_escstatus.data[i];
    }

    return nullptr;
}

void AP_UAVCAN::escstatus_update_i(uint8_t i)
{
    // technically, it could happen that the esc_index is not continuous, and one would need a better handling
    // however, I think, ArduPilot implicitly enforces continuous esc_index, so should be no problem
    uint8_t id = _escstatus.id[i];
    if (id >= 8) return;

    if (_escstatus.listener != nullptr) {
        _escstatus.listener->handle_escstatus_msg( id, _escstatus.data[i].voltage, _escstatus.data[i].current );
    }
}

//--- uc4h.Notify ---
// outgoing message

bool AP_UAVCAN::uc4hnotify_sem_take()
{
    bool sem_ret = _uc4hnotify.sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN Uc4h Out semaphore fail\n\r");
    }
    return sem_ret;
}


void AP_UAVCAN::uc4hnotify_sem_give()
{
    _uc4hnotify.sem->give();
}


// I don't like the current procedure in this library
// the msg is copied into two fields, so, double work for nothing, "performance killer"
void AP_UAVCAN::uc4hnotify_send(uint8_t type, uint8_t subtype, uint8_t* payload, uint8_t payload_len)
{
    if (_uc4hnotify.sem->take(1)) {

        _uc4hnotify.msg.type = type;
        _uc4hnotify.msg.subtype = subtype;
        if (payload_len > 64) payload_len = 64; //play it safe
        _uc4hnotify.msg.payload.resize(payload_len);
        for (uint8_t i = 0; i < payload_len; i++) {
            _uc4hnotify.msg.payload[i] = payload[i];
        }

        _uc4hnotify.priority = uavcan::TransferPriority::MiddleLower;

        _uc4hnotify.to_send = true;
        uc4hnotify_sem_give();
    }
}


void AP_UAVCAN::uc4h_do_cyclic(uint64_t current_time_ms)
{
    if (uc4hnotify_array[_uavcan_i] == nullptr) {
        return;
    }

    //always send only one message per cycle
    if (_uc4hnotify.to_send && uc4hnotify_sem_take()) {

        uc4hnotify_array[_uavcan_i]->setPriority(_uc4hnotify.priority);
        uc4hnotify_array[_uavcan_i]->broadcast(_uc4hnotify.msg);

        _uc4hnotify.to_send = false;
        uc4hnotify_sem_give();
    }
}

// --- tunnel.Broadcast ---
// outgoing message

bool AP_UAVCAN::tunnelbroadcast_out_sem_take()
{
    bool sem_ret = _tunnelbroadcast_out.sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN tunnelbroadcast Out semaphore fail\n\r");
    }
    return sem_ret;
}


void AP_UAVCAN::tunnelbroadcast_out_sem_give()
{
    _tunnelbroadcast_out.sem->give();
}


bool AP_UAVCAN::tunnelbroadcast_is_to_send(uint8_t tunnel_index)
{
    return _tunnelbroadcast_out.to_send[tunnel_index];
}


void AP_UAVCAN::tunnelbroadcast_send(uint8_t tunnel_index, uint8_t protocol, uint8_t channel_id, uint8_t* buffer, uint8_t buffer_len, uint8_t priority)
{
    if (_tunnelbroadcast_out.sem->take(1)) { //why 1 and not 10 here?

        //I for the heck can't figure out how to set the protocol C++ like
        // this stupid workaround seems to work though
        memset(&(_tunnelbroadcast_out.msg[tunnel_index].protocol), protocol, 1);

        _tunnelbroadcast_out.msg[tunnel_index].channel_id = channel_id;
        _tunnelbroadcast_out.msg[tunnel_index].buffer.resize(buffer_len);
        for (uint8_t i = 0; i < buffer_len; i++) {
            _tunnelbroadcast_out.msg[tunnel_index].buffer[i] = buffer[i];
        }

        _tunnelbroadcast_out.priority[tunnel_index] = uavcan::TransferPriority::MiddleLower;

        _tunnelbroadcast_out.to_send[tunnel_index] = true;
        tunnelbroadcast_out_sem_give();
    }
}

//one frame per cycle is ca 800 Hz x 60 bytes = 48000 bytes/s
// a CAN frame is 131 bits max => 7633 franmes/s max = ca 61 kbytes/s max
// => this consumes already 79% of the available bandwidth !!!
void AP_UAVCAN::tunnelbroadcast_out_do_cyclic(void)
{
    if (tunnelbroadcast_out_array[_uavcan_i] == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < TUNNELMANAGER_NUM_CHANNELS; i++) {
        if (_tunnelbroadcast_out.to_send[i] && tunnelbroadcast_out_sem_take()) {

            tunnelbroadcast_out_array[_uavcan_i]->setPriority(_tunnelbroadcast_out.priority[i]);
            tunnelbroadcast_out_array[_uavcan_i]->broadcast(_tunnelbroadcast_out.msg[i]);

            _tunnelbroadcast_out.to_send[i] = false;
            tunnelbroadcast_out_sem_give();
        }
    }
}
//OWEND
#endif // HAL_WITH_UAVCAN
